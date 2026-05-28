#!/usr/bin/env python3
import argparse
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from libs.parser import parse_bandwidth_file, parse_latency_file, parse_filename
from libs.visualizer import calculate_color

def load_config(measuring_dir):
    config_path = os.path.join(measuring_dir, 'plotter.txt')
    config = {}
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            for x in f:
                if '=' in x and not x.startswith('#'):
                    k, v = x.rstrip().split("=", 1)
                    config[k.strip()] = v.strip()
    return config

def main():
    parser = argparse.ArgumentParser(description="Generate latency plots from measuring directory")
    parser.add_argument("directory", help="Measurement directory (e.g. measuring/multisequential)")
    args = parser.parse_args()

    measuring_dir = os.path.abspath(args.directory)
    output_dir = os.path.join(measuring_dir, "processed")
    
    if not os.path.exists(measuring_dir):
        print(f"Error: Directory '{measuring_dir}' does not exist.")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)
    config = load_config(measuring_dir)

    bw_dir = os.path.join(measuring_dir, 'bw')
    lat_dir = os.path.join(measuring_dir, 'lat')
    sampler_dir = os.path.join(measuring_dir, 'sampler')

    data_points = []
    sampler_data = {}

    if not (os.path.exists(bw_dir) and os.path.exists(lat_dir)):
        print(f"Error: 'bw' or 'lat' directory not found under '{measuring_dir}'.")
        sys.exit(1)

    print(f"Processing data in '{measuring_dir}'...")

    for bw_filename in os.listdir(bw_dir):
        if not bw_filename.endswith('.txt'):
            continue
            
        rd_pct, pause = parse_filename(bw_filename)
        if rd_pct is None:
            continue

        lat_filename = f"lat_{rd_pct}_{pause}.txt"
        sampler_filename = f"sampler_{rd_pct}_{pause}.csv"

        bw_path = os.path.join(bw_dir, bw_filename)
        lat_path = os.path.join(lat_dir, lat_filename)
        sampler_path = os.path.join(sampler_dir, sampler_filename)

        if not os.path.exists(lat_path):
            continue

        bw_measurements = parse_bandwidth_file(bw_path, config, rd_percentage_from_filename=rd_pct)
        if not bw_measurements:
            continue
        bw_avg = np.mean([m['bandwidth'] for m in bw_measurements])
        
        lat_measurements = parse_latency_file(lat_path, config)
        if not lat_measurements:
            continue
            
        mean_ns = np.mean([m.get('mean_ns', m.get('latency', 0.0)) for m in lat_measurements])
        p50_ns = np.mean([m.get('median_ns', 0.0) for m in lat_measurements])
        p99_9_ns = np.mean([m.get('p99_9_ns', 0.0) for m in lat_measurements])
        
        data_point = {
            'rd_pct': rd_pct,
            'pause': pause,
            'bandwidth': bw_avg,
            'mean_ns': mean_ns,
            'median_ns': p50_ns,
            'p99_9_ns': p99_9_ns,
            'corrected_mean_ns': np.nan
        }

        if os.path.exists(sampler_path):
            try:
                df = pd.read_csv(sampler_path)
                if 'latency_cycles' in df.columns:
                    cpu_freq = float(config.get('CPU_FREQ', 3.0))
                    df['latency_ns'] = df['latency_cycles'] / cpu_freq
                    sampler_data[(rd_pct, pause)] = df
                    
                    latencies = df['latency_ns'].values
                    q1 = np.percentile(latencies, 25)
                    q3 = np.percentile(latencies, 75)
                    iqr = q3 - q1
                    upper_bound = q3 + 3 * iqr
                    filtered_latencies = latencies[latencies <= upper_bound]
                    if len(filtered_latencies) > 0:
                        data_point['corrected_mean_ns'] = np.mean(filtered_latencies)
            except Exception as e:
                print(f"Warning: Error parsing {sampler_path}: {e}")

        data_points.append(data_point)

    if not data_points:
        print("No paired bandwidth and latency data found.")
        sys.exit(1)

    df_summary = pd.DataFrame(data_points)

    unique_ratios = sorted(df_summary['rd_pct'].unique())
    
    for rd_pct in unique_ratios:
        ratio_dir = os.path.join(output_dir, f"ratio_{rd_pct}")
        os.makedirs(ratio_dir, exist_ok=True)
        print(f"Generating plots for Read Ratio {rd_pct}% in '{ratio_dir}'...")

        df_ratio = df_summary[df_summary['rd_pct'] == rd_pct].copy()
        sampler_ratio = {k: v for k, v in sampler_data.items() if k[0] == rd_pct}

        print(f"  -> Average Latency vs. BW...")
        fig1, ax1 = plt.subplots(figsize=(8, 5))
        df_group = df_ratio.sort_values('bandwidth')
        ax1.plot(df_group['bandwidth'], df_group['mean_ns'], marker='o', label=f'Read {rd_pct}%')
        for _, row in df_group.iterrows():
            ax1.text(row['bandwidth'], row['mean_ns'], f"{row['mean_ns']:.1f}", 
                     fontsize=8, ha='left', va='bottom', alpha=0.8)
            
        ax1.set_xlabel('Bandwidth (GB/s)')
        ax1.set_ylabel('Average Latency (ns)')
        ax1.set_title(f'Average Latency vs. BW (Read {rd_pct}%)')
        ax1.grid(True, linestyle='--', alpha=0.7)
        ax1.legend()
        fig1.tight_layout()
        fig1.savefig(os.path.join(ratio_dir, 'avg_lat_vs_bw.png'), dpi=300)
        plt.close(fig1)

        if sampler_ratio:
            print(f"  -> Latency CDF...")
            fig2, ax2 = plt.subplots(figsize=(8, 5))
            
            pauses = [p for (_, p) in sampler_ratio.keys()]
            min_pause = min(pauses) if pauses else 0
            max_pause = max(pauses) if pauses else 1
            norm = plt.Normalize(min_pause, max_pause)
            cmap = plt.colormaps['Blues']
            
            sorted_pauses = sorted(list(set(pauses)))
            
            for (_, pause), df in sorted(sampler_ratio.items(), key=lambda x: x[0][1]):
                if df.empty or 'latency_ns' not in df.columns:
                    continue
                latencies = np.sort(df['latency_ns'].values)
                cdf = np.arange(1, len(latencies) + 1) / len(latencies)
                p99_9 = np.percentile(latencies, 99.9)
                valid_indices = latencies <= (p99_9 * 1.5)
                
                if len(sorted_pauses) > 1:
                    pause_index = sorted_pauses.index(pause)
                    intensity = 0.4 + 0.6 * (pause_index / (len(sorted_pauses) - 1))
                else:
                    intensity = 0.8
                color = cmap(intensity)
                
                ax2.plot(latencies[valid_indices], cdf[valid_indices], label=f'Pause {pause}', color=color, linewidth=1)
            
            ax2.set_xlim(left=0)
            ax2.set_ylim(bottom=0, top=1.05)
            ax2.set_xlabel('Latency (ns)')
            ax2.set_ylabel('CDF')
            ax2.set_title(f'Latency CDF (Read {rd_pct}%)')
            ax2.grid(True, linestyle='--', alpha=0.7)
            
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
            sm.set_array([])
            cbar = fig2.colorbar(sm, ax=ax2)
            cbar.set_label('Pause')
            
            fig2.tight_layout()
            fig2.savefig(os.path.join(ratio_dir, 'lat_cdf.png'), dpi=300)
            plt.close(fig2)

        print(f"  -> Diff of 99.9th and 50th Latency vs. BW...")
        fig3, ax3 = plt.subplots(figsize=(8, 5))
        df_ratio['tail_diff'] = df_ratio['p99_9_ns'] - df_ratio['median_ns']
        
        max_bw = df_ratio['bandwidth'].max()
        if max_bw > 0:
            df_ratio['bandwidth_pct'] = (df_ratio['bandwidth'] / max_bw) * 100
        else:
            df_ratio['bandwidth_pct'] = df_ratio['bandwidth']
            
        df_group = df_ratio.sort_values('bandwidth_pct')
        ax3.plot(df_group['bandwidth_pct'], df_group['tail_diff'], marker='^', label=f'Read {rd_pct}%')
            
        ax3.set_xlabel('Bandwidth (%)')
        ax3.set_ylabel('(p99.9 - p50) Lat (ns)')
        ax3.set_title(f'Diff of 99.9th and 50th Latency vs. BW (Read {rd_pct}%)')
        ax3.grid(True, linestyle='--', alpha=0.7)
        ax3.legend()
        fig3.tight_layout()
        fig3.savefig(os.path.join(ratio_dir, 'tail_lat_vs_bw.png'), dpi=300)
        plt.close(fig3)

        if sampler_ratio:
            print(f"  -> Latency Distribution Histogram...")
            fig4, ax4 = plt.subplots(figsize=(8, 5))
            
            pauses = [p for (_, p) in sampler_ratio.keys()]
            min_pause = min(pauses) if pauses else 0
            max_pause = max(pauses) if pauses else 1
            cmap = plt.colormaps['Blues']
            
            sorted_pauses = sorted(list(set(pauses)))
            
            for (_, pause), df in sorted(sampler_ratio.items(), key=lambda x: x[0][1]):
                if df.empty or 'latency_ns' not in df.columns:
                    continue
                latencies = df['latency_ns'].values
                p99_9 = np.percentile(latencies, 99.9)
                valid_indices = latencies <= (p99_9 * 1.5)
                
                if len(sorted_pauses) > 1:
                    pause_index = sorted_pauses.index(pause)
                    intensity = 0.4 + 0.6 * (pause_index / (len(sorted_pauses) - 1))
                else:
                    intensity = 0.8
                color = cmap(intensity)
                
                ax4.hist(latencies[valid_indices], bins=100, alpha=0.5, label=f'Pause {pause}', density=True, color=color)
                
            ax4.set_xlim(left=0)
            ax4.set_xlabel('Latency (ns)')
            ax4.set_ylabel('Density')
            ax4.set_title(f'Latency Distribution Histogram (Read {rd_pct}%)')
            ax4.grid(True, linestyle='--', alpha=0.7)
            if len(sampler_ratio) <= 10:
                ax4.legend()
            fig4.tight_layout()
            fig4.savefig(os.path.join(ratio_dir, 'lat_hist.png'), dpi=300)
            plt.close(fig4)

        if 'corrected_mean_ns' in df_ratio.columns and not df_ratio['corrected_mean_ns'].isna().all():
            print(f"  -> Corrected Average Latency vs. BW...")
            fig5, ax5 = plt.subplots(figsize=(8, 5))
            df_group = df_ratio.sort_values('bandwidth')
            df_group = df_group.dropna(subset=['corrected_mean_ns'])
            if not df_group.empty:
                ax5.plot(df_group['bandwidth'], df_group['corrected_mean_ns'], marker='o', label=f'Read {rd_pct}%')
                for _, row in df_group.iterrows():
                    ax5.text(row['bandwidth'], row['corrected_mean_ns'], f"{row['corrected_mean_ns']:.1f}", 
                             fontsize=8, ha='left', va='bottom', alpha=0.8)
            
            ax5.set_xlabel('Bandwidth (GB/s)')
            ax5.set_ylabel('Corrected Avg Latency (ns)')
            ax5.set_title(f'Corrected Avg Latency (IQR Filtered) vs. BW (Read {rd_pct}%)')
            ax5.grid(True, linestyle='--', alpha=0.7)
            ax5.legend()
            fig5.tight_layout()
            fig5.savefig(os.path.join(ratio_dir, 'corrected_avg_lat_vs_bw.png'), dpi=300)
            plt.close(fig5)

    print(f"Plots successfully generated and saved to '{output_dir}'.")

if __name__ == '__main__':
    main()