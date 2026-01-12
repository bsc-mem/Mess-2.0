#!/usr/bin/env python3
import os
import sys
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from scipy.interpolate import interp1d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from libs.visualizer import calculate_color, add_gradient_legend, filter_clustered_points, is_valid_curve
from libs.processor import process_data, calculate_mean_curves, smooth_curves, filter_incomplete_ratios

fontSizeMess = 24

def load_curves_from_csv(curves_path):
    df = pd.read_csv(curves_path)
    
    if 'read_pct_rounded' in df.columns:
        ratio_col = 'read_pct_rounded'
    elif 'rd_percentage' in df.columns:
        ratio_col = 'rd_percentage'
    else:
        raise ValueError(f"Could not find read ratio column in {curves_path}")
    
    if 'bandwidth_smooth' in df.columns:
        bw_col = 'bandwidth_smooth'
    elif 'bandwidth_mean' in df.columns:
        bw_col = 'bandwidth_mean'
    elif 'bandwidth' in df.columns:
        bw_col = 'bandwidth'
    else:
        raise ValueError(f"Could not find bandwidth column in {curves_path}")
    
    if 'latency_smooth' in df.columns:
        lat_col = 'latency_smooth'
    elif 'latency_mean' in df.columns:
        lat_col = 'latency_mean'
    elif 'latency' in df.columns:
        lat_col = 'latency'
    else:
        raise ValueError(f"Could not find latency column in {curves_path}")
    
    dfs_rw = {}
    for ratio in df[ratio_col].unique():
        dfs_rw[ratio] = df[df[ratio_col] == ratio].copy()
    
    return dfs_rw, ratio_col, bw_col, lat_col

def load_curves_from_dir(measuring_dir):
    df_bw, df_lat, config = process_data(measuring_dir)
    
    if df_bw.empty or df_lat.empty:
        raise ValueError(f"No data found in {measuring_dir}")
    
    df_for_csv = calculate_mean_curves(df_bw, df_lat, '1', 'perread')
    if df_for_csv.empty:
        raise ValueError(f"No valid curves in {measuring_dir}")
    
    df_for_csv = filter_incomplete_ratios(df_for_csv, 'read_pct_rounded')
    dfs_rw = smooth_curves(df_for_csv, 'perread')
    
    if not dfs_rw:
        raise ValueError(f"No valid curves after smoothing in {measuring_dir}")
    
    return dfs_rw, config

def load_profiler_data(profiler_path):
    df = pd.read_csv(profiler_path)
    
    required_cols = ['Bandwidth(GB/s)', 'ReadBytes', 'WriteBytes']
    for col in required_cols:
        if col not in df.columns:
            alt_names = {
                'Bandwidth(GB/s)': ['bandwidth', 'bw', 'Bandwidth'],
                'ReadBytes': ['read_bytes', 'ReadBytes', 'read'],
                'WriteBytes': ['write_bytes', 'WriteBytes', 'write']
            }
            found = False
            for alt in alt_names.get(col, []):
                if alt in df.columns:
                    df[col] = df[alt]
                    found = True
                    break
            if not found:
                raise ValueError(f"Could not find column {col} in profiler output")
    
    return df

def calculate_read_ratio(read_bytes, write_bytes):
    total = read_bytes + write_bytes
    if total == 0:
        return 50.0
    return (read_bytes / total) * 100.0

def find_latency_for_point(bw, read_ratio, curves_df, ratio_col, bw_col, lat_col):
    available_ratios = sorted(curves_df[ratio_col].unique())
    
    if not available_ratios:
        return np.nan
    
    closest_ratio = min(available_ratios, key=lambda x: abs(x - read_ratio))
    
    curve_data = curves_df[curves_df[ratio_col] == closest_ratio].sort_values(bw_col)
    
    if len(curve_data) < 2:
        return np.nan
    
    bw_values = curve_data[bw_col].values
    lat_values = curve_data[lat_col].values
    
    if bw < bw_values.min():
        return lat_values[0]
    if bw > bw_values.max():
        return lat_values[-1]
    
    try:
        interp_func = interp1d(bw_values, lat_values, kind='linear', fill_value='extrapolate')
        return float(interp_func(bw))
    except:
        return np.nan

def find_latency_for_point_dfs(bw, read_ratio, dfs_rw):
    available_ratios = sorted(dfs_rw.keys())
    
    if not available_ratios:
        return np.nan
    
    closest_ratio = min(available_ratios, key=lambda x: abs(x - read_ratio))
    
    curve_data = dfs_rw[closest_ratio].sort_values('bandwidth_smooth')
    
    if len(curve_data) < 2:
        return np.nan
    
    bw_values = curve_data['bandwidth_smooth'].values
    lat_values = curve_data['latency_smooth'].values
    
    if bw < bw_values.min():
        return lat_values[0]
    if bw > bw_values.max():
        return lat_values[-1]
    
    try:
        interp_func = interp1d(bw_values, lat_values, kind='linear', fill_value='extrapolate')
        return float(interp_func(bw))
    except:
        return np.nan

def plot_app_on_curves(dfs_rw, profiler_df, output_path, config=None, 
                       app_name='Application', cmap_name='Blues'):
    
    profiler_df = profiler_df.copy()
    profiler_df['read_ratio'] = profiler_df.apply(
        lambda row: calculate_read_ratio(row['ReadBytes'], row['WriteBytes']), axis=1
    )
    
    profiler_df['latency'] = profiler_df.apply(
        lambda row: find_latency_for_point_dfs(
            row['Bandwidth(GB/s)'], row['read_ratio'], dfs_rw
        ), axis=1
    )
    
    profiler_df = profiler_df.dropna(subset=['latency'])
    
    if profiler_df.empty:
        print("Warning: No valid points to plot after latency interpolation")
        return None
    
    fig, ax = plt.subplots(1, 1)
    
    if config:
        try:
            nvlink_bw_str = config.get("NVLINK_BW_GB_S", None)
            nvlink_bw = float(nvlink_bw_str) if nvlink_bw_str is not None else None

            if nvlink_bw is not None and nvlink_bw == 0:
                max_bw = 0
            elif nvlink_bw is not None and nvlink_bw > 0:
                max_bw = nvlink_bw
            else:
                binding = config.get("MEMORY_BINDING", "local").strip().lower()
                if binding == "remote":
                    upi_freq = float(config.get("UPI_FREQ", 16))
                    n_lanes = float(config.get("N_DATA_LANES", 20))
                    flit_bit = float(config.get("FLIT_BIT", 80))
                    data_flit_bit = float(config.get("DATA_FLIT_BIT", 64))
                    n_upi_channels = float(config.get("N_UPI_CHANNELS", 4))
                    
                    if data_flit_bit > 0 and flit_bit > 0:
                        max_bw = 2.0 * (upi_freq * n_lanes * (data_flit_bit / flit_bit) * n_upi_channels) / 8.0
                    else:
                        max_bw = 0
                else:
                    freq_str = config.get("MEM_FREQ", "")
                    if freq_str and "Could not detect" not in freq_str:
                        freq = float(freq_str.split()[0])
                    else:
                        freq = 0
                        
                    channels = float(config.get("N_CHANNELS", 2))
                    bus_width = float(config.get("BUS_WIDTH", 64)) 
                    
                    if bus_width > 0 and freq > 0:
                         max_bw = (bus_width / 8) * freq * channels / 1000.0
                    else:
                         max_bw = 0
        except (ValueError, AttributeError):
            max_bw = 0
    else:
        max_bw = 0
    
    ax.set_xlabel('Used Memory bandwidth [GB/s]', fontsize=fontSizeMess + 3)
    ax.set_ylabel('Memory access latency [ns]', fontsize=fontSizeMess + 3)

    sorting_list = []
    processed_dfs = {}
    skipped_ratios = {}
    
    global_max_bw = 0.0
    global_max_lat = 0.0
    
    for rw, df in dfs_rw.items():
        if not df.empty:
            df_clean = filter_clustered_points(df.copy(), 5.0)
            processed_dfs[rw] = df_clean
            
            if 'bandwidth_smooth' in df_clean.columns and not df_clean['bandwidth_smooth'].empty:
                global_max_bw = max(global_max_bw, df_clean['bandwidth_smooth'].max())
            if 'latency_smooth' in df_clean.columns and not df_clean['latency_smooth'].empty:
                global_max_lat = max(global_max_lat, df_clean['latency_smooth'].max())
                
            sort_value = df_clean['bandwidth_smooth'].max() 
            sorting_list.append((rw, sort_value))

    if max_bw > 0:
        limit_bw = max_bw
        if global_max_bw > limit_bw:
             limit_bw = global_max_bw * 1.1
    else:
        limit_bw = global_max_bw * 1.1 if global_max_bw > 0 else 321

    limit_lat = max(700, global_max_lat * 1.1) if global_max_lat > 0 else 700
    
    ax.set_xlim([0, limit_bw])
    ax.set_ylim([0, limit_lat])

    sorted_ratios = [rw for rw, _ in sorted(sorting_list, key=lambda x: x[1])]
    for rw in sorted_ratios:
        df = processed_dfs[rw]
        if not df.empty:
            is_valid, reason = is_valid_curve(
                df,
                min_points=5,
                max_cluster_fraction=0.70,
                gap_fraction=0.5)
            if is_valid: 
                ax.plot(df['bandwidth_smooth'], df['latency_smooth'], 
                    color=calculate_color(rw, cmap_name), linewidth=2, 
                    label=f'Rd:Wr {rw}:0')
            else: 
                if reason not in skipped_ratios:
                    skipped_ratios[reason] = []
                skipped_ratios[reason].append(rw)

    for reason, ratios in skipped_ratios.items():
        print(f"Warning: Ratios {ratios} not plotted: {reason}")

    ax.scatter(
        profiler_df['Bandwidth(GB/s)'],
        profiler_df['latency'],
        c='black',
        s=50,
        alpha=0.5,
        zorder=10,
        label=app_name
    )

    if config:
        is_nvidia_pmu = config.get("USE_NVIDIA_PMU", "False").lower() in ('true', '1', 't')
        nvlink_bw = float(config.get("NVLINK_BW_GB_S", 0))
        should_draw_max_bw_line = not (is_nvidia_pmu and nvlink_bw == 0)

        if max_bw > 0 and should_draw_max_bw_line:
            ax.axvline(x=max_bw, color='red', linewidth=2, linestyle=':')
            ax.text(x=max_bw, y=ax.get_ylim()[1] * 0.95, 
                   s=f'Max. theoretical BW = {max_bw:.0f} GB/s', 
                   horizontalalignment='right', fontsize=fontSizeMess - 4)
    
    ax.tick_params(axis='both', labelsize=fontSizeMess)
    
    base_x = 0.1
    base_y = 0.99
    add_gradient_legend(ax, None, 50, 100, colors=cmap_name, box_position=(base_x, base_y), box_size=(0.25, 0.03), fontsize=20)

    fig.set_size_inches([16, 9])
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    
    plt.close(fig)
    
    print(f"Plot saved to: {output_path}")
    print(f"Plot saved to: {png_path}")
    
    return profiler_df

def print_summary(profiler_df, app_name):
    print(f"\n=== {app_name} Summary ===")
    print(f"  Samples: {len(profiler_df)}")
    print(f"  Bandwidth: {profiler_df['Bandwidth(GB/s)'].min():.2f} - {profiler_df['Bandwidth(GB/s)'].max():.2f} GB/s")
    print(f"  Avg Bandwidth: {profiler_df['Bandwidth(GB/s)'].mean():.2f} GB/s")
    print(f"  Read Ratio: {profiler_df['read_ratio'].min():.1f}% - {profiler_df['read_ratio'].max():.1f}%")
    print(f"  Avg Read Ratio: {profiler_df['read_ratio'].mean():.1f}%")
    if 'latency' in profiler_df.columns:
        print(f"  Estimated Latency: {profiler_df['latency'].min():.1f} - {profiler_df['latency'].max():.1f} ns")
        print(f"  Avg Estimated Latency: {profiler_df['latency'].mean():.1f} ns")

def main():
    parser = argparse.ArgumentParser(
        description="Plot application profiler data on memory bandwidth-latency curves",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--curves', '-c', required=True,
                        help='Path to measurement directory (with bw/lat subdirs) or memory_curves.csv')
    parser.add_argument('--profiler', '-p', required=True,
                        help='Path to mess-profiler output CSV')
    parser.add_argument('--output', '-o', default=None,
                        help='Output PDF path (default: app_on_curves.pdf in profiler dir)')
    parser.add_argument('--name', '-n', default='Application',
                        help='Application name for the plot title')
    parser.add_argument('--cmap', default='Blues',
                        help='Colormap for the background curves')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.curves):
        print(f"Error: Curves path not found: {args.curves}")
        sys.exit(1)
    
    if not os.path.exists(args.profiler):
        print(f"Error: Profiler output not found: {args.profiler}")
        sys.exit(1)
    
    print(f"Loading curves from: {args.curves}")
    
    config = None
    if os.path.isdir(args.curves):
        dfs_rw, config = load_curves_from_dir(args.curves)
        print(f"  Found {len(dfs_rw)} read ratios from directory")
    else:
        dfs_rw, ratio_col, bw_col, lat_col = load_curves_from_csv(args.curves)
        print(f"  Found {len(dfs_rw)} read ratios from CSV")
    
    global_max_bw = 0.0
    for rw, df in dfs_rw.items():
        if 'bandwidth_smooth' in df.columns and not df.empty:
            global_max_bw = max(global_max_bw, df['bandwidth_smooth'].max())
    print(f"  Bandwidth range: 0.0 - {global_max_bw:.1f} GB/s")
    
    print(f"\nLoading profiler data from: {args.profiler}")
    profiler_df = load_profiler_data(args.profiler)
    print(f"  Found {len(profiler_df)} samples")
    
    if args.output:
        output_path = args.output
    else:
        profiler_dir = os.path.dirname(os.path.abspath(args.profiler))
        output_path = os.path.join(profiler_dir, 'app_on_curves.pdf')
    
    print(f"\nGenerating plot...")
    result_df = plot_app_on_curves(
        dfs_rw, profiler_df, output_path, 
        config=config, app_name=args.name, cmap_name=args.cmap
    )
    
    if result_df is not None:
        print_summary(result_df, args.name)
        
        csv_output = output_path.replace('.pdf', '.csv')
        result_df.to_csv(csv_output, index=False)
        print(f"\nEnriched data saved to: {csv_output}")

if __name__ == '__main__':
    main()
