import os
import sys
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from .parser import parse_bandwidth_file, parse_latency_file, parse_bwlat_file, parse_filename

def _file_contains_pcm_cxl_counters(filepath):
    try:
        with open(filepath, 'r') as f:
            for line in f:
                lower = line.lower()
                if 'pcm_cxl_read_mbps' in lower or 'pcm_cxl_write_mbps' in lower or 'pcm_cxl_total_mbps' in lower:
                    return True
    except OSError:
        return False
    return False

def process_data(measuring_dir):
    config_path = os.path.join(measuring_dir, 'plotter.txt')
    
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = dict(x.rstrip().split("=", 1) for x in f if '=' in x and not x.startswith('#'))
        #print(f"Loaded plotter config from {config_path}")
    else:
        print(f"Error: plotter.txt not found in {measuring_dir}")
        print("Please run generate_code first to create the configuration file")
        sys.exit(1)
    
    bw_data = []
    is_intel_pcm = False
    bw_dir = os.path.join(measuring_dir, 'bw')
    if os.path.exists(bw_dir):
        files = os.listdir(bw_dir)
        for filename in files:
            if filename.startswith('bw_') and filename.endswith('.txt'):
                filepath = os.path.join(bw_dir, filename)
                if not is_intel_pcm and _file_contains_pcm_cxl_counters(filepath):
                    is_intel_pcm = True
                rd_percentage, pause = parse_filename(filename)
                
                if rd_percentage is not None:
                    measurements = parse_bandwidth_file(filepath, config, rd_percentage_from_filename=rd_percentage)
                    if measurements:
                        for measurement in measurements:
                            measurement.update({
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            })
                            bw_data.append(measurement)
    
    lat_data = []
    lat_dir = os.path.join(measuring_dir, 'lat')
    if os.path.exists(lat_dir):
        files = os.listdir(lat_dir)
        for filename in files:
            if filename.startswith('lat_') and filename.endswith('.txt'):
                filepath = os.path.join(lat_dir, filename)
                rd_percentage, pause = parse_filename(filename)
                
                if rd_percentage is not None:
                    measurements = parse_latency_file(filepath, config)
                    if measurements:
                        for measurement in measurements:
                            measurement.update({
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            })
                            lat_data.append(measurement)
    
    if os.path.exists(measuring_dir):
        files = os.listdir(measuring_dir)
        for filename in files:
            if filename.startswith('bwlat_') and filename.endswith('.txt'):
                filepath = os.path.join(measuring_dir, filename)
                rd_percentage, pause = parse_filename(filename)
                
                if rd_percentage is not None:
                    data_points = parse_bwlat_file(filepath, config)
                    if data_points:
                        for point in data_points:
                            bw_metrics = {
                                'bandwidth': point['bandwidth'],
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            }
                            lat_metrics = {
                                'latency': point['latency'],
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            }
                            bw_data.append(bw_metrics)
                            lat_data.append(lat_metrics)
    curves_dir = os.path.join(measuring_dir, 'curves')
    if os.path.exists(curves_dir):
        files = os.listdir(curves_dir)
        for filename in files:
            if filename.startswith('bwlat_') and filename.endswith('.txt'):
                filepath = os.path.join(curves_dir, filename)
                rd_percentage, pause = parse_filename(filename)
                
                if rd_percentage is not None:
                    data_points = parse_bwlat_file(filepath, config)
                    if data_points:
                        for point in data_points:
                            bw_metrics = {
                                'bandwidth': point['bandwidth'],
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            }
                            lat_metrics = {
                                'latency': point['latency'],
                                'rd_percentage': rd_percentage,
                                'pause': pause
                            }
                            bw_data.append(bw_metrics)
                            lat_data.append(lat_metrics)
    
    df_bw = pd.DataFrame(bw_data) if bw_data else pd.DataFrame()
    df_lat = pd.DataFrame(lat_data) if lat_data else pd.DataFrame()
    # Only concat if df_lat has the latency column to avoid KeyError on empty/failed runs
    if not df_lat.empty and 'latency' in df_lat.columns:
        df = pd.concat([df_bw, df_lat[['latency']]], axis=1)
    else:
        df = df_bw.copy()

    if is_intel_pcm:
        config['IS_INTEL_PCM'] = 'true'

    return df_bw, df_lat, config

def calculate_mean_curves(df_bw, df_lat, step, mode):
    step = float(step)
    if df_bw.empty or df_lat.empty:
        return pd.DataFrame()

    df_bw = df_bw.copy()
    df_lat = df_lat.copy()

    if mode == 'perread':
        read_label = 'read_pct_rounded'

        actual_pct = df_bw['rd_percentage_actual'] * 100
        df_bw[read_label] = (np.round(actual_pct / step) * step).astype(int)
        df_bw[read_label] = df_bw[read_label].clip(0, 100)

        mapping = (df_bw[['rd_percentage', 'pause', read_label]]
                   .drop_duplicates(subset=['rd_percentage', 'pause'])
                   .set_index(['rd_percentage', 'pause'])[read_label])

        df_lat[read_label] = df_lat.set_index(['rd_percentage', 'pause']).index.map(mapping)

        if df_lat[read_label].isna().any():
            n_missing = df_lat[read_label].isna().sum()
            print(f"Warning: {n_missing} rows in df_lat could not be mapped to read_pct_rounded.")

        df_lat = df_lat.dropna(subset=[read_label])

    else:
        # perkernel groups by issued read ratio; other modes default to rounded read ratio.
        read_label = 'rd_percentage' if mode == 'perkernel' else 'read_pct_rounded'
        reads = df_bw['rd_percentage']
        df_bw[read_label] = (np.round(reads / step) * step).astype(int)
        df_bw[read_label] = df_bw[read_label].clip(0, 100)
        if read_label == 'rd_percentage':
            if 'rd_percentage' in df_lat.columns:
                df_lat['rd_percentage'] = (np.round(df_lat['rd_percentage'] / step) * step).astype(int)
                df_lat['rd_percentage'] = df_lat['rd_percentage'].clip(0, 100)
        else:
            mapping = (df_bw[['rd_percentage', 'pause', read_label]]
                       .drop_duplicates(subset=['rd_percentage', 'pause'])
                       .set_index(['rd_percentage', 'pause'])[read_label])
            df_lat[read_label] = df_lat.set_index(['rd_percentage', 'pause']).index.map(mapping)
            df_lat = df_lat.dropna(subset=[read_label])

    # === Common grouping logic ===
    bw_mean = df_bw.groupby([read_label, 'pause']).agg({
        'bandwidth': ['mean', 'std'],
        'rd_percentage_actual': 'mean'   # keep the true average actual %
    }).reset_index()

    pebs_lat_cols = [c for c in ('mean_ns', 'min_ns', 'median_ns', 'p90_ns', 
                                 'p95_ns', 'p99_ns', 'p99_9_ns', 'max_ns') 
                     if c in df_lat.columns]

    lat_agg = {'latency': ['mean', 'std']}
    for c in pebs_lat_cols:
        lat_agg[c] = 'mean'

    lat_mean = df_lat.groupby([read_label, 'pause']).agg(lat_agg).reset_index()

    # Flatten multi-index columns
    bw_mean.columns = [read_label, 'pause', 'bandwidth_mean', 'bandwidth_std', 'rd_percentage_actual_mean']
    lat_mean.columns = [read_label, 'pause', 'latency_mean', 'latency_std'] + pebs_lat_cols

    # Merge on the *rounded* key + pause
    df = bw_mean.merge(lat_mean, on=[read_label, 'pause'], how='inner')

    # Sort: usually you want increasing read % and then highest bandwidth first (or whatever you prefer)
    df = df.sort_values(by=[read_label, 'bandwidth_mean'], 
                        ascending=[True, False]).reset_index(drop=True)

    return df

def smooth_curves(df, mode, 
                  apply_smoothing=True, 
                  apply_corrections=True,
                  min_y_change_pct=5.0):
    """
    Smooth only the portion of the curve where latency has increased by at least 
    min_y_change_pct compared to the very first point (anchor point).
    """
    if mode == 'perread':
        read_label = 'read_pct_rounded'
    else:
        read_label = 'rd_percentage'

    if read_label not in df.columns:
        fallback_label = 'read_pct_rounded' if read_label == 'rd_percentage' else 'rd_percentage'
        if fallback_label in df.columns:
            read_label = fallback_label
        else:
            return {}

    if df.empty:
        return {}

    dfs_rw = {}
    
    for rd_pct in sorted(df[read_label].unique()):
        df_rw = df[df[read_label] == rd_pct].copy()
        
        if df_rw.empty:
            continue
            
        if 'bandwidth' in df_rw.columns:
            bw_col = 'bandwidth'
            lat_col = 'latency'
        else:
            bw_col = 'bandwidth_mean'
            lat_col = 'latency_mean'

        df_rw = df_rw.sort_values(bw_col).reset_index(drop=True)

        if apply_corrections and not df_rw.empty:
            min_lat = df_rw[lat_col].min()
            zero_row = df_rw.iloc[0].copy()
            zero_row[bw_col] = 0.0
            zero_row[lat_col] = min_lat
            df_rw = pd.concat([pd.DataFrame([zero_row]), df_rw], ignore_index=True)
            df_rw = df_rw.sort_values(bw_col).reset_index(drop=True)

        df_rw['bandwidth_smooth'] = df_rw[bw_col].copy()
        df_rw['latency_smooth'] = df_rw[lat_col].copy()

        if not apply_smoothing or len(df_rw) < 4:
            dfs_rw[rd_pct] = df_rw
            continue

        x = df_rw[bw_col].values
        y = df_rw[lat_col].values

        first_y = y[0]

        y_pct_diff = np.abs((y - first_y) / first_y) * 100 if first_y > 0 else np.zeros_like(y)

        mask = y_pct_diff >= min_y_change_pct

        if not np.any(mask):
            dfs_rw[rd_pct] = df_rw
            continue

        start_idx = max(0, np.where(mask)[0][0] - 1)

        mask_to_smooth = np.zeros(len(y), dtype=bool)
        mask_to_smooth[start_idx:] = True
        mask_to_smooth[0] = False

        try:
            smooth_y = savgol_filter(y, window_length=7, polyorder=3)
            smooth_x = savgol_filter(x, window_length=7, polyorder=3)

            df_rw.loc[mask_to_smooth, 'latency_smooth'] = smooth_y[mask_to_smooth]
            df_rw.loc[mask_to_smooth, 'bandwidth_smooth'] = smooth_x[mask_to_smooth]

        except (ValueError, Exception) as e:
            print(f"Smoothing failed for read_pct={rd_pct}: {e}")
            df_rw['bandwidth_smooth'] = df_rw[bw_col]
            df_rw['latency_smooth'] = df_rw[lat_col]

        dfs_rw[rd_pct] = df_rw

    return dfs_rw

def filter_incomplete_ratios(df, read_label, threshold_ratio=0.6, original_label="rd_percentage_actual_mean"):
    if df.empty:
        return df
    
    counts = df.groupby(read_label).size()
    
    if counts.empty:
        return df
        
    global_max = df["bandwidth_mean"].max()
    global_min = df["bandwidth_mean"].min()
    global_range = global_max - global_min
    avg_count = counts.mean()
    min_count_threshold = 0.5 * avg_count
    ratios_to_keep = []
    dropped_ratios = []
    for ratio, count in counts.items():
        group = df[df[read_label] == ratio] 
        group_min = group["bandwidth_mean"].min()
        group_max = group["bandwidth_mean"].max()
        group_range = group_max - group_min

        # overlap with global range (allowing 60% slack on max and 20% on min)
        min_ok = group_min <= global_min + 0.6 * global_range
        max_ok = group_max >= global_max - 0.35 * global_range
        if count > min_count_threshold: 
            dropped_ratios.append(ratio) 
        if min_ok and max_ok:
             ratios_to_keep.append(ratio)
        else:
            dropped_ratios.append(ratio)
    return df[df[read_label].isin(ratios_to_keep)]




# 
# 
# 
# 
