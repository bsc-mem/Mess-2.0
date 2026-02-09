#!/usr/bin/env python3
import os
import sys
import argparse
import pandas as pd
from libs.parser import parse_bwlat_file, parse_filename, csv_to_json_compact
from libs.processor import process_data, calculate_mean_curves, smooth_curves, filter_incomplete_ratios
from libs.visualizer import plot_curves, plot_combined_curves

def process_directory(measuring_dir, step, mode, cmap_name='Blues', ignore_guards=False):
    """Process a single directory and return the smoothed curves dataframe"""
    if not os.path.exists(measuring_dir):
        #print(f"Error: Directory {measuring_dir} does not exist")
        return None, None, None

    processed_dir = os.path.join(measuring_dir, 'processed')
    os.makedirs(processed_dir, exist_ok=True)
    
    
    #print(f"\n\n==========================================\nProcessing measurement data from: {measuring_dir}")
    
    df_bw, df_lat, config = process_data(measuring_dir)
    
    if df_bw.empty or df_lat.empty:
        #print(f"No data found in {measuring_dir}")
        return None, None, None
    
    has_bwlat_files = False
    has_bw_lat_dirs = os.path.exists(os.path.join(measuring_dir, 'bw')) and os.path.exists(os.path.join(measuring_dir, 'lat'))
    
    if os.path.exists(measuring_dir):
        files = os.listdir(measuring_dir)
        has_bwlat_files = any(f.startswith('bwlat_') and f.endswith('.txt') for f in files)
    
    dfs_rw = {}
    df_for_csv = pd.DataFrame()

    if has_bwlat_files and not has_bw_lat_dirs:
        #print("Detected bwlat format")
        read_label = "read_pct_rounded"
        combined_data = []
        # Check root dir
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
                                combined_data.append({
                                    'bandwidth': point['bandwidth'],
                                    'latency': point['latency'],
                                    'rd_percentage': rd_percentage,
                                    'pause': pause
                                })
        
        # Check curves subdir
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
                                combined_data.append({
                                    'bandwidth': point['bandwidth'],
                                    'latency': point['latency'],
                                    'rd_percentage': rd_percentage,
                                    'pause': pause
                                })
        
        df_for_csv = pd.DataFrame(combined_data)
        if not ignore_guards:
            df_for_csv = filter_incomplete_ratios(df_for_csv)
        if not df_for_csv.empty:
            df_for_csv = df_for_csv.sort_values(['read_pct_rounded', 'bandwidth'])
        dfs_rw = smooth_curves(df_for_csv)
        if not dfs_rw:
            #print("No valid curves after smoothing")
            return None, None, None
        for key in dfs_rw:
            dfs_rw[key] = dfs_rw[key].sort_values('bandwidth_smooth').reset_index(drop=True)
        df_smoothed_all = pd.concat(
            [df.assign(read_pct_rounded=key) for key, df in dfs_rw.items()],
            ignore_index=True
        )
    else:
        #print("Detected old format")
        if mode == "perread": 
            read_label = "read_pct_rounded"
        if mode == "perkernel":  
            read_label = "rd_percentage" 
        df_for_csv = calculate_mean_curves(df_bw, df_lat,step,mode)

        if df_for_csv.empty:
            return None, None, None
        if not ignore_guards:
            df_for_csv = filter_incomplete_ratios(df_for_csv, read_label)
        dfs_rw = smooth_curves(df_for_csv, mode)
        if not dfs_rw:
            #print("No valid curves after smoothing")
            return None, None, None
        #for key in df_for_csv:
        #    df_for_csv[key] = df_for_csv[key].sort_values('bandwidth_mean').reset_index(drop=True)
        df_smoothed_all = pd.concat(
                [df.assign(read_pct_rounded=key) for key, df in dfs_rw.items()],
                ignore_index=True
                ) 
        
        
        #dfs_rw = smooth_curves(df_for_csv, mode)
    
    # Plot individual
    output_path = os.path.join(processed_dir, 'memory_curves.pdf')
    plot_curves(config, dfs_rw, output_path, cmap_name, ignore_guards=ignore_guards)
    
    csv_path = os.path.join(processed_dir, 'memory_curves.csv')
    df_smoothed_all.to_csv(csv_path, index=False)
    #print(f"Data saved to: {csv_path}")
    
    json_path = os.path.join(processed_dir, 'memory_curves.json')
    csv_to_json_compact(csv_path,json_path, read_label, "pause", "bandwidth_smooth","latency_smooth")
    #print(f"Data saved to: {json_path}")
    
    # Print summary statistics
    # Print summary statistics
    #print(f"Summary for {os.path.basename(os.path.normpath(measuring_dir))}:")
    summary = {}
    if 'bandwidth' in df_for_csv.columns:
        #print(f"  Bandwidth range: {df_for_csv['bandwidth'].min():.3f} - {df_for_csv['bandwidth'].max():.3f} GB/s")
        summary['bw_min'] = df_for_csv['bandwidth'].min()
        summary['bw_max'] = df_for_csv['bandwidth'].max()
    elif 'bandwidth_mean' in df_for_csv.columns:
        #print(f"  Bandwidth range: {df_for_csv['bandwidth_mean'].min():.3f} - {df_for_csv['bandwidth_mean'].max():.3f} GB/s")
        summary['bw_min'] = df_for_csv['bandwidth_mean'].min()
        summary['bw_max'] = df_for_csv['bandwidth_mean'].max()
        
    if 'latency' in df_for_csv.columns:
        #print(f"  Latency range: {df_for_csv['latency'].min():.1f} - {df_for_csv['latency'].max():.1f} ns")
        summary['lat_min'] = df_for_csv['latency'].min()
        summary['lat_max'] = df_for_csv['latency'].max()
    elif 'latency_mean' in df_for_csv.columns:
        #print(f"  Latency range: {df_for_csv['latency_mean'].min():.1f} - {df_for_csv['latency_mean'].max():.1f} ns")
        summary['lat_min'] = df_for_csv['latency_mean'].min()
        summary['lat_max'] = df_for_csv['latency_mean'].max()
    
    return dfs_rw, config, summary

def main():
    parser = argparse.ArgumentParser(
        description="Generate memory bandwidth-latency roofline plots",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('directories', nargs='+', help='One or more measurement directories')
    parser.add_argument('--mode', choices=['perread', 'perkernel'], default='perread',
                        help='Plotting mode: group curves by kernel (default) or by measured read ratio')
    parser.add_argument('--step', choices=['1', '2', '4', '5', '10', '50'], default='1',
                        help='Plotting step: plot curves in steps of 1, 2, 4, 5, 10 or 50')
    parser.add_argument('--progress', action='store_true',
                        help='Ignore validity checks and plot in-progress curves')


    args = parser.parse_args()
    directories = [os.path.abspath(d) for d in args.directories]
    mode = args.mode
    step = args.step
    ignore_guards = args.progress
    # Define a list of colormaps to cycle through
    colormaps = ['Blues', 'Reds', 'Greens', 'Oranges', 'Purples', 'Greys']
    curve_cmap = 'Blues'
    datasets = []
    
    
    summary_list = []
    
    print("Processing directories...", end="", flush=True)

    for i, measuring_dir in enumerate(directories):
        print(".", end="", flush=True)
        cmap_name = colormaps[i % len(colormaps)]
        dfs_rw, config, summary = process_directory(measuring_dir, step, mode, curve_cmap, ignore_guards=ignore_guards)
        
        if dfs_rw:
            label = os.path.basename(os.path.normpath(measuring_dir))
            datasets.append((label, dfs_rw, config, cmap_name, measuring_dir))
            if summary:
                summary['label'] = label
                summary['dir'] = measuring_dir
                summary_list.append(summary)
    print(" Done!")
    
    if not datasets:
        return

    shared_global_max_bw = 0.0
    shared_global_max_lat = 0.0
    for _, dfs_rw, cfg, _, _ in datasets:
        for _, df in dfs_rw.items():
            if not df.empty:
                if 'bandwidth_smooth' in df.columns and not df['bandwidth_smooth'].empty:
                    shared_global_max_bw = max(shared_global_max_bw, df['bandwidth_smooth'].max())
                if 'latency_smooth' in df.columns and not df['latency_smooth'].empty:
                    shared_global_max_lat = max(shared_global_max_lat, df['latency_smooth'].max())

    for _, dfs_rw, config, cmap_name, measuring_dir in datasets:
        output_path = os.path.join(measuring_dir, 'processed', 'memory_curves.pdf')
        plot_curves(config, dfs_rw, output_path, cmap_name, ignore_guards=ignore_guards)

    if len(datasets) > 1:
        #print("\nGenerating combined plot...")
        first_dir = directories[0]
        output_path = os.path.join(first_dir, 'processed', 'combined_memory_curves.pdf')
        plot_combined_curves(datasets, output_path, ignore_guards=ignore_guards)
    

    print("\n\n======================================================================================")
    print("                                 PLOTTING SUMMARY")
    print("======================================================================================")
    
    # Header
    print(f"{'Directory':<30} | {'Bandwidth Range (GB/s)':<25} | {'Latency Range (ns)':<25}")
    print("-" * 86)
    
    for item in summary_list:
        bw_range = f"{item.get('bw_min', 0):.2f} - {item.get('bw_max', 0):.2f}"
        lat_range = f"{item.get('lat_min', 0):.1f} - {item.get('lat_max', 0):.1f}"
        print(f"{item['label']:<30} | {bw_range:<25} | {lat_range:<25}")
        rel_dir = os.path.relpath(item['dir'], os.getcwd())
        print(f"   -> {rel_dir}/processed")

    if len(datasets) > 1:
        first_dir = directories[0]
        print("-" * 86)
        print(f"Combined plot saved to:")
        rel_first_dir = os.path.relpath(first_dir, os.getcwd())
        print(f"   -> {rel_first_dir}/processed/combined_memory_curves.pdf")

    print("======================================================================================\n")


if __name__ == '__main__':
    main()
