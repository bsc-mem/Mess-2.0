import os
import sys
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from .parser import parse_bandwidth_file, parse_latency_file, parse_bwlat_file, parse_filename

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
    bw_dir = os.path.join(measuring_dir, 'bw')
    if os.path.exists(bw_dir):
        files = os.listdir(bw_dir)
        for filename in files:
            if filename.startswith('bw_') and filename.endswith('.txt'):
                filepath = os.path.join(bw_dir, filename)
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
    
    return df_bw, df_lat, config

def calculate_mean_curves(df_bw, df_lat, step, mode):
    step = float(step)
    if df_bw.empty or df_lat.empty:
        return pd.DataFrame()
    
    if mode == 'perkernel':
        read_label = 'rd_percentage'
    else: 
        read_label = 'read_pct_rounded'

    if mode == 'perread':
        # Step 1: Create rounded read percentage in df_bw (based on actual measured ratio)
        actual_pct = df_bw['rd_percentage_actual'] * 100
        df_bw = df_bw.copy() # Avoid SettingWithCopy issues
        df_bw['read_pct_rounded'] = (np.round(actual_pct / step) * step).astype(int)
        df_bw['read_pct_rounded'] = df_bw['read_pct_rounded'].clip(0, 100)
    

        # Step 2: Transfer read_pct_rounded to df_lat using the common keys (rd_percentage, pause)
        # This assumes that for each (rd_percentage, pause), there's a unique corresponding actual read %
        mapping = df_bw[['rd_percentage', 'pause', 'read_pct_rounded']].drop_duplicates()
        df_lat = df_lat.merge(mapping, on=['rd_percentage', 'pause'], how='left')
    
        # Optional: sanity check
        if df_lat['read_pct_rounded'].isna().any():
            #print("Warning: Some rows in df_lat could not be matched with read_pct_rounded!")
            pass
    else: 
        reads= df_bw[read_label]
        df_bw[read_label] = (np.round( reads/ step) * step).astype(int)
        df_bw[read_label] = df_bw[read_label].clip(0, 100)

    # Step 3: Now group both dataframes using the same read_pct_rounded
    bw_mean = df_bw.groupby([read_label, 'pause']).agg({
        'bandwidth': ['mean', 'std'],
        'rd_percentage_actual': 'mean' 
        }).reset_index()
    lat_mean = df_lat.groupby([read_label, 'pause']).agg({
        'latency': ['mean', 'std']
        }).reset_index()
    # Flatten column names
    bw_mean.columns = [read_label, 'pause', 'bandwidth_mean', 'bandwidth_std', 'rd_percentage_actual_mean']
    lat_mean.columns = [read_label, 'pause', 'latency_mean', 'latency_std']
    # Step 4: Merge on the shared rounded percentage and pause
    df = bw_mean.merge(lat_mean, on=[read_label, 'pause'], how='inner')
    df = df.sort_values(
    by=[read_label, 'bandwidth_mean'],
    ascending=[True, False]   # or [True, True] depending on what you want
    ).reset_index(drop=True)
    return df

def smooth_curves(df, mode):
    if mode == 'perread':
        read_label = 'read_pct_rounded'
    else: 
        read_label = 'rd_percentage'
    if df.empty:
        return {}
    
    dfs_rw = {}
    for rd_pct in df[read_label].unique():
        df_rw = df[df[read_label] == rd_pct].copy()
        
        if 'bandwidth' in df_rw.columns:
            df_rw = df_rw.sort_values('bandwidth')
            min_lat = df_rw['latency'].min()
            # Create a 0-point row
            zero_row = df_rw.iloc[0].copy()
            zero_row['bandwidth'] = 0
            zero_row['latency'] = min_lat
            # Depending on index/other columns, we might need to be careful
            # Append and resort
            df_rw = pd.concat([pd.DataFrame([zero_row]), df_rw], ignore_index=True)
            df_rw = df_rw.sort_values('bandwidth')

        elif 'bandwidth_mean' in df_rw.columns:
            df_rw = df_rw.sort_values('bandwidth_mean')
            min_lat = df_rw['latency_mean'].min()
            # Create a 0-point row
            zero_row = df_rw.iloc[0].copy()
            zero_row['bandwidth_mean'] = 0
            zero_row['latency_mean'] = min_lat
            # Append and resort
            df_rw = pd.concat([pd.DataFrame([zero_row]), df_rw], ignore_index=True)
            df_rw = df_rw.sort_values('bandwidth_mean')
        
        if 'bandwidth' in df_rw.columns and 'latency' in df_rw.columns:
            df_rw['bandwidth_smooth'] = df_rw['bandwidth']
            df_rw['latency_smooth'] = df_rw['latency']
        else:
            if len(df_rw) >= 7:
                try:
                    x = df_rw['bandwidth_mean'].values
                    y = df_rw['latency_mean'].values
                    
                    smooth_x, smooth_y = savgol_filter((x, y), window_length=11, polyorder=3)
                    
                    df_rw['bandwidth_smooth'] = smooth_x
                    df_rw['latency_smooth'] = smooth_y
                    
                except (ValueError, Exception):
                    df_rw['bandwidth_smooth'] = df_rw['bandwidth_mean']
                    df_rw['latency_smooth'] = df_rw['latency_mean']
                    
            else:
                df_rw['bandwidth_smooth'] = df_rw['bandwidth_mean']
                df_rw['latency_smooth'] = df_rw['latency_mean']
        
        dfs_rw[rd_pct] = df_rw

    return dfs_rw

def filter_incomplete_ratios(df, read_label, threshold_ratio=0.6):
    if df.empty:
        return df
    
    counts = df.groupby(read_label).size()
    
    if counts.empty:
        return df
        
    mean_count = counts.mean()
    threshold = mean_count * threshold_ratio
    
    ratios_to_keep = []
    dropped_ratios = []
    for ratio, count in counts.items():
        if count >= threshold:
            ratios_to_keep.append(ratio)
        else:
            dropped_ratios.append(ratio)
            
    if dropped_ratios:
        #print(f"Filtered out {len(dropped_ratios)} ratios with insufficient data (<{threshold:.1f} points): {dropped_ratios}")
        pass
            
    return df[df[read_label].isin(ratios_to_keep)]
