import os
import sys
import pandas as pd

def csv_to_json_compact(
    input_csv: str,
    output_json: str,
    group_col: str = "rd_percentage",
    sort_col: str = "pause",
    bw_col: str = "bandwidth_mean",
    lat_col: str = "latency_mean",
) -> None:
    df = pd.read_csv(input_csv)
   # ensure numeric
    for c in (group_col, sort_col, bw_col, lat_col):
        df[c] = pd.to_numeric(df[c], errors="coerce")

    # multiply all the bws by 1000:
    df[bw_col] = df[bw_col] 

    df[lat_col] = df[lat_col]

    # group & sort
    groups = {}
    for grp, sub in df.groupby(group_col):
        sub = sub.sort_values(sort_col)
        groups[int(grp)] = sub[[bw_col, lat_col]].values.tolist()

    # write manually
    with open(output_json, "w") as f:
        f.write("{\n")
        for i, (grp, pairs) in enumerate(sorted(groups.items())):
            f.write(f'    "{grp}": [\n')
            for j, (bw, lat) in enumerate(pairs):
                comma = "," if j < len(pairs) - 1 else ""
                f.write(f"        [{bw}, {lat}]{comma}\n")
            end_comma = "," if i < len(groups) - 1 else ""
            f.write(f"    ]{end_comma}\n")
        f.write("}\n")

    print(f"Done — wrote {len(groups)} groups to {output_json}")



def parse_bandwidth_file(filepath, config, rd_percentage_from_filename=None):
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        rd_counts = []
        wr_counts = []
        combined_counts = []
        runtimes = []
        counter_type = None
        
        for line in lines:
            if 'S1' in line:
                continue
            
            line_lower = line.lower()
            
            is_amd_combined = 'dram_channel_data_controller' in line_lower
            
            is_upi_rx = ('rxl_flits' in line_lower and 'all_data' in line_lower) or \
                        ('data_from_remote_dram' in line_lower) or \
                        ('amd_df' in line_lower and ('remote' in line_lower))
            is_upi_tx = ('txl_flits' in line_lower and 'all_data' in line_lower)
            
            is_cas_rd = ('cas_count' in line_lower and ('rd' in line_lower or 'read' in line_lower)) or \
                        ('amd_umc' in line_lower and ('rd' in line_lower or 'read' in line_lower)) or \
                        ('data_from_local_dram' in line_lower)
            is_cas_wr = ('cas_count' in line_lower and ('wr' in line_lower or 'write' in line_lower)) or \
                        ('amd_umc' in line_lower and ('wr' in line_lower or 'write' in line_lower))

            is_grace_rd = 'cmem_rd' in line_lower or 'remote_socket_rd_data' in line_lower
            is_grace_wr = 'cmem_wr' in line_lower or 'remote_socket_wr_total_bytes' in line_lower
            
            if is_amd_combined:
                if counter_type is None:
                    counter_type = 'AMD_COMBINED'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        combined_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        combined_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_upi_rx:
                if counter_type is None:
                    counter_type = 'UPI_FLITS'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        rd_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        rd_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_upi_tx:
                if counter_type is None:
                    counter_type = 'UPI_FLITS'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        wr_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        wr_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_cas_rd:
                if counter_type is None:
                    counter_type = 'CAS_COUNT'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        rd_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        rd_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_cas_wr:
                if counter_type is None:
                    counter_type = 'CAS_COUNT'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        wr_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        wr_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_grace_rd:
                if counter_type is None:
                    counter_type = 'NVIDIA_GRACE'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        rd_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        rd_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif is_grace_wr:
                if counter_type is None:
                    counter_type = 'NVIDIA_GRACE'
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'S0':
                    try:
                        wr_counts.append(int(parts[2]))
                    except ValueError:
                        pass
                elif len(parts) >= 2:
                    try:
                        wr_counts.append(int(parts[0]))
                    except ValueError:
                        pass
            elif 'seconds time elapsed' in line:
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            runtimes.append(float(parts[0]))
                        except ValueError:
                            pass
        
        if counter_type == 'AMD_COMBINED' and combined_counts and runtimes:
            min_len = min(len(combined_counts), len(runtimes))
            measurements = []
            for i in range(min_len):
                total = combined_counts[i]
                runtime = runtimes[i]
                
                if runtime <= 0:
                    continue
                
                rd_percentage_actual = (rd_percentage_from_filename / 100.0) if rd_percentage_from_filename is not None else 1.0
                
                try:
                    cache_line_size = int(config.get('CACHE_LINE_SIZE', 64))
                except (ValueError, TypeError):
                    cache_line_size = 64

                bandwidth = total * cache_line_size / runtime / 1000 / 1000 / 1000
                
                measurements.append({
                    'bandwidth': bandwidth,
                    'rd_percentage_actual': rd_percentage_actual,
                    'runtime': runtime
                })
            
            return measurements
        
        min_len = min(len(rd_counts), len(wr_counts), len(runtimes))
        
        if min_len > 0:
            measurements = []
            for i in range(min_len):
                rd = rd_counts[i]
                wr = wr_counts[i]
                runtime = runtimes[i]
                
                if runtime <= 0:
                    continue
                
                total_counts = rd + wr
                rd_percentage_actual = (rd / total_counts) if total_counts > 0 else 0
                
                try:
                    cache_line_size = int(config.get('CACHE_LINE_SIZE', 64))
                except (ValueError, TypeError):
                    cache_line_size = 64

                if counter_type == 'UPI_FLITS':
                    try:
                        scaling_factor = float(config.get('UPI_SCALING_FACTOR', 9.0))
                    except (ValueError, TypeError):
                        scaling_factor = 9.0
                    bandwidth = (total_counts / scaling_factor) * cache_line_size / runtime / 1000 / 1000 / 1000
                elif counter_type == 'NVIDIA_GRACE':
                    # NVIDIA Grace: Reads are 32B chunks, Writes are bytes
                    bandwidth = (rd * 32.0 + wr) / runtime / 1000 / 1000 / 1000
                    rd = rd*32.0
                    total_counts = (rd) + wr
                    rd_percentage_actual = (rd / total_counts) if total_counts > 0 else 0

                else:
                    bandwidth = total_counts * cache_line_size / runtime / 1000 / 1000 / 1000
                
                measurements.append({
                    'bandwidth': bandwidth,
                    'rd_percentage_actual': rd_percentage_actual,
                    'runtime': runtime
                })
            
            return measurements
        else:
            print(f"  Missing or mismatched data in {filepath}: rd={len(rd_counts)}, wr={len(wr_counts)}, time={len(runtimes)}, combined={len(combined_counts)}")
    except Exception as e:
        print(f"Error parsing {filepath}: {e}")
    
    return None

def parse_latency_file(filepath, config):
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        measurements = []
        
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if not line or 'S1' in line:
                i += 1
                continue
                
            if any(token in line for token in ['cycles:u', 'instructions:u', 'r1012:u', 'r2012:u', 'cycles', 'instructions', 'r1012', 'r2012']):
                cycles = None
                instructions = None
                tlb1miss = None
                tlb2miss = None
                accesses = None
                seconds_user = None
                
                j = i
                while j < len(lines) and j < i + 10:
                    block_line = lines[j].strip()
                    
                    if 'cycles:u' in block_line or ('cycles' in block_line and 'instructions' not in block_line):
                        parts = block_line.split()
                        for part in parts:
                            try:
                                cycles = int(part)
                                break
                            except ValueError:
                                continue
                                
                    elif 'instructions:u' in block_line or 'instructions' in block_line:
                        parts = block_line.split()
                        for part in parts:
                            try:
                                instructions = int(part)
                                break
                            except ValueError:
                                continue
                                
                    elif 'accesses' in block_line:
                        parts = block_line.split()
                        for part in parts:
                            try:
                                accesses = int(part)
                                break
                            except ValueError:
                                continue
                                
                    elif 'r1012:u' in block_line or 'r1012' in block_line:
                        parts = block_line.split()
                        for part in parts:
                            try:
                                tlb1miss = int(part)
                                break
                            except ValueError:
                                continue
                                
                    elif 'r2012:u' in block_line or 'r2012' in block_line:
                        parts = block_line.split()
                        for part in parts:
                            try:
                                tlb2miss = int(part)
                                break
                            except ValueError:
                                continue
                                
                    elif 'seconds time elapsed' in block_line:
                        parts = block_line.split()
                        for part in parts:
                            try:
                                seconds_user = float(part)
                                break
                            except ValueError:
                                continue
                    
                    j += 1
                
                if cycles is not None and seconds_user is not None and seconds_user > 0:
                    if tlb1miss is None: tlb1miss = 0
                    if tlb2miss is None: tlb2miss = 0
                    if accesses is None: accesses = instructions if instructions is not None else 0
                    
                    if accesses is None or accesses <= 0:
                        j = i + 1
                        i = j
                        continue
                    
                    cpu_freq_hz = cycles / seconds_user
                    cpu_freq_ghz = cpu_freq_hz / 1e9
                    
                    try:
                        tlb_hit_latency_ns = float(config.get('TLB_NS', 10.0))
                    except (ValueError, TypeError):
                        print("Warning: Invalid TLB_NS in config, using default 10.0ns")
                        tlb_hit_latency_ns = 10.0
                    
                    total_ns = seconds_user * 1e9
                    pagewalk_ns = (cpu_freq_ghz > 0.0) and (tlb2miss / cpu_freq_ghz) or 0.0
                    stlb_ns = tlb_hit_latency_ns * tlb1miss
                    effective_ns = total_ns - pagewalk_ns - stlb_ns
                    if effective_ns <= 0.0 or accesses <= 0:
                        latency = 0.0
                    else:
                        latency = effective_ns / accesses
                    
                    measurements.append({
                        'latency': latency,
                        'cycles': cycles,
                        'instructions': instructions,
                        'accesses': accesses,
                        'tlb1miss': tlb1miss,
                        'tlb2miss': tlb2miss
                    })
                
                i = j
            else:
                i += 1
        
        if measurements:
            return measurements
        else:
            print(f"  No complete measurements found in {filepath}")
            return None
    except Exception as e:
        print(f"Error parsing {filepath}: {e}")
        return None
    return None

def parse_bwlat_file(filepath, config):
    try:
        data_points = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            bw_mbps = float(parts[0])
                            latency_cycles = float(parts[1])
                            
                            bw_gbps = bw_mbps / 1000.0
                            
                            cpu_freq_ghz = float(config.get('CPU_FREQ'))
                            latency_ns = latency_cycles / cpu_freq_ghz
                            
                            data_points.append({
                                'bandwidth': bw_gbps,
                                'latency': latency_ns
                            })
                        except (ValueError, IndexError):
                            continue
        
        if data_points:
            return data_points
        else:
            print(f"  No valid data found in {filepath}")
    except Exception as e:
        print(f"Error parsing {filepath}: {e}")
    
    return None

def parse_filename(filename):
    parts = filename.replace('.txt', '').split('_')
    
    if filename.startswith('bwlat_') and len(parts) >= 2:
        try:
            rd_percentage = int(parts[1])
            return rd_percentage, 0
        except (ValueError, IndexError):
            pass
    elif len(parts) >= 3:
        try:
            rd_percentage = int(parts[1])
            pause = int(parts[2])
            return rd_percentage, pause
        except (ValueError, IndexError) as e:
            print(f"    Error parsing parts: {e}")
    
    return None, None
