import os
import matplotlib.pyplot as plt
import warnings
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import matplotlib.patches as patches
import seaborn as sns
from matplotlib.lines import Line2D
from matplotlib.colors import Normalize

fontSizeMess = 24

def _is_intel_pcm_config(config):
    if not isinstance(config, dict):
        return False

    explicit_marker = str(config.get("IS_INTEL_PCM", "")).strip().lower()
    if explicit_marker in ("1", "true", "yes", "y"):
        return True

    for key in ("MEASURER", "BW_TOOL", "BW_MEASURER", "COUNTER_SOURCE"):
        value = str(config.get(key, "")).strip().lower()
        if "pcm" in value:
            return True

    for key in ("READ_EVENT", "WRITE_EVENT", "READ_EVENTS", "WRITE_EVENTS", "BW_EVENT"):
        value = str(config.get(key, "")).strip().lower()
        if "pcm_cxl/" in value or "pcm_cxl_" in value:
            return True

    return False

def calculate_color(rw, cmap_name='Blues'):
    """Map RW ratio to Blues using legacy intensity bounds (no near-white curves)."""
    cmap = plt.colormaps[cmap_name]

    rw_clipped = max(50.0, min(100.0, float(rw)))
    # Keep the same direction as legend labels: 100 on the left, 50 on the right.
    # Legacy plotters did not use the near-white end of the colormap.
    c = 0.2 + ((100.0 - rw_clipped) / 50.0) * 0.8

    return cmap(c)


def add_gradient_legend(ax, label, rw_min=0, rw_max=100, colors=None, box_position=(0.05, 0.95), box_size=(0.25, 0.03), fontsize=12):
    """
    Adds a horizontal gradient legend inside the axes at a fixed position.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        The axes to attach the legend to.
    label : str
        The dataset label for the gradient legend.
    rw_min, rw_max : float
        Minimum and maximum RW values to display.
    colors : list of colors
        List of colors for the gradient (if None, defaults to plt.cm.Blues).
    box_position : tuple (x0, y0)
        Position of the top-left corner of the box in axes fraction coordinates.
    box_size : tuple (width, height)
        Size of the legend box in axes fraction coordinates.
    fontsize : int
        Font size for the labels.
    """
    fig = ax.figure

    cmap = plt.get_cmap(colors)
    norm = Normalize(vmin=rw_min, vmax=rw_max)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])

    # Convert axes fraction to figure fraction
    bbox = ax.get_position()
    fig_x = bbox.x0 + box_position[0] * bbox.width
    fig_y = bbox.y0 + box_position[1] * bbox.height
    fig_width = box_size[0] * bbox.width
    fig_height = box_size[1] * bbox.height

    # Add a new axes for the gradient inside the plot
    cbar_ax = fig.add_axes([fig_x, fig_y, fig_width, fig_height])
    cbar = plt.colorbar(sm, cax=cbar_ax, orientation='horizontal')
    cbar.outline.set_visible(False)
    cbar.ax.set_xticks([])
    # Add labels at the ends
    cbar_ax.text(0.01, 0.5, f'RD:WR {rw_max}:{100-rw_max}', va='center', ha='right',
                 transform=cbar_ax.transAxes, fontsize=fontsize-3)
    cbar_ax.text(1.00, 0.5, f'RD:WR {rw_min}:{100-rw_min}', va='center', ha='left',
            transform=cbar_ax.transAxes, fontsize=fontsize-3)

     # Add dataset label on top of the gradient
    cbar_ax.text(0.5, 1.0, label, va='bottom', ha='center',
                 transform=cbar_ax.transAxes, fontsize=fontsize, color='black')




def filter_clustered_points(
    df,
    bw_tolerance_gb=2.0,
    min_latency_increase_percent=5.0,   
    bw_col='bandwidth_smooth',
    lat_col='latency_smooth'
):
    """
    Keep points where:
    - bandwidth increases by at least bw_tolerance_gb
    - AND smoothed latency increases by at least min_latency_increase_percent %
    """
    if df.empty or len(df) <= 1:
        return df
    
    if bw_col not in df.columns or lat_col not in df.columns:
        return df
    
    df = df.sort_values(bw_col).reset_index(drop=True)
    
    keep_idx = [0]
    last_bw = df.iloc[0][bw_col]
    last_lat = df.iloc[0][lat_col]
    
    threshold = min_latency_increase_percent / 100.0 
    
    for i in range(1, len(df)):
        current_bw = df.iloc[i][bw_col]
        current_lat = df.iloc[i][lat_col]
        
        bw_ok = current_bw - last_bw >= bw_tolerance_gb
        
        lat_ok = False
        if last_lat > 0 and current_lat > last_lat:
            increase_ratio = (current_lat - last_lat) / last_lat
            lat_ok = increase_ratio >= threshold
        
        if bw_ok and lat_ok:
            keep_idx.append(i)
            last_bw = current_bw
            last_lat = current_lat
    
    return df.iloc[keep_idx].reset_index(drop=True)

def is_valid_curve(df,
                   min_points=10,
                   max_cluster_fraction=0.4,
                   gap_fraction=0.5,        # NEW: max allowed jump in GB/s
                   bw_column='bandwidth_mean',
                   ignore_guards=False):

    if ignore_guards:
        return True, "valid (forced)"

    if bw_column not in df.columns:
        if 'bandwidth_smooth' in df.columns:
            bw_column = 'bandwidth_smooth'
        elif 'bandwidth_mean' in df.columns:
            bw_column = 'bandwidth_mean'
        elif 'bandwidth' in df.columns:
            bw_column = 'bandwidth'
        else:
            return False, "missing_bandwidth_column"

    bw = df[bw_column].dropna()

    # Check for large horizontal jumps
    bw_sorted = np.sort(bw)
    gaps = np.diff(bw_sorted)
    max_gap = gaps.max() if len(gaps) > 0 else 0
    span = bw.max() - bw.min()
    max_horizontal_gap = span * gap_fraction
    

    if max_gap > max_horizontal_gap:
        return False, f"large_jump ({max_gap:.1f} > {max_horizontal_gap} GB/s)"


    return True, "valid"


def _compute_theoretical_max_bw(config):
    try:
        if _is_intel_pcm_config(config):
            return 72.0

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

    return max_bw



def plot_curves(config, dfs_rw, output_path='memory_curves.pdf', cmap_name='Blues', limit_bw_override=None, limit_lat_override=None, ignore_guards=False, apply_corrections=True, y_label=None, draw_dots=False):
    """Plot bandwidth-latency curves and save to PDF"""
    if not dfs_rw:
        print("No data to plot")
        return
    
    fig, ax = plt.subplots(1, 1)
    
    max_bw = _compute_theoretical_max_bw(config)
    
    ax.set_xlabel('Used Memory bandwidth [GB/s]', fontsize=fontSizeMess + 3)
    ax.set_ylabel(y_label or 'Memory access latency [ns]', fontsize=fontSizeMess + 3)

    sorting_list = []
    processed_dfs = {}
    skipped_ratios = {}
    
    global_max_bw = 0.0
    global_max_lat = 0.0
    
    for rw, df in dfs_rw.items():
        if not df.empty:
            if apply_corrections:
                df_clean = filter_clustered_points(df.copy(), 1.0, bw_col='bandwidth_smooth')
            else:
                df_clean = df.copy()
            processed_dfs[rw] = df_clean
            
            if 'bandwidth_smooth' in df_clean.columns and not df_clean['bandwidth_smooth'].empty:
                global_max_bw = max(global_max_bw, df_clean['bandwidth_smooth'].max())
            if 'latency_smooth' in df_clean.columns and not df_clean['latency_smooth'].empty:
                global_max_lat = max(global_max_lat, df_clean['latency_smooth'].max())
                
            sort_value = df_clean['bandwidth_smooth'].max() 
            sorting_list.append((rw, sort_value))

    limit_bw = limit_bw_override
    if limit_bw is None:
        if max_bw > 0:
            limit_bw = max_bw * 1.1
        else:
            limit_bw = global_max_bw * 1.1 if global_max_bw > 0 else 321

    limit_lat = limit_lat_override
    if limit_lat is None:
        limit_lat = max(700, global_max_lat * 1.1) if global_max_lat > 0 else 700
    
    ax.set_xlim([0, limit_bw])
    ax.set_ylim([0, limit_lat])

    marker = 'o' if draw_dots else None
    markersize = 4 if draw_dots else None

    sorted_ratios = [rw for rw, _ in sorted(sorting_list, key=lambda x: x[0])]
    for rw in sorted_ratios:
        df = processed_dfs[rw]
        if not df.empty:
            if (not apply_corrections) or ignore_guards:
                ax.plot(df['bandwidth_smooth'], df['latency_smooth'], 
                    color=calculate_color(rw, cmap_name), linewidth=1,
                    marker=marker, markersize=markersize,
                    label=f'Rd:Wr {rw}:0')
            else: 
                is_valid, reason = is_valid_curve(
                    df,
                    min_points=5,
                    max_cluster_fraction=0.70,
                    gap_fraction=0.7,
                    ignore_guards=False)
                if is_valid:
                    ax.plot(df['bandwidth_smooth'], df['latency_smooth'], 
                        color=calculate_color(rw, cmap_name), linewidth=1,
                        marker=marker, markersize=markersize,
                        label=f'Rd:Wr {rw}:0')
                else:
                    if reason not in skipped_ratios:
                        skipped_ratios[reason] = []
                    skipped_ratios[reason].append(rw)

    for reason, ratios in skipped_ratios.items():
        #print(f"Warning: Ratios {ratios} not plotted: {reason}")
        pass

    
    is_nvidia_pmu = config.get("USE_NVIDIA_PMU", "False").lower() in ('true', '1', 't')
    nvlink_bw = float(config.get("NVLINK_BW_GB_S", 0))
    
    # Do not draw the line for NVIDIA counters if NVLINK_BW is not specified
    should_draw_max_bw_line = not (is_nvidia_pmu and nvlink_bw == 0)

    if max_bw > 0 and should_draw_max_bw_line:
        ax.axvline(x=max_bw, color=calculate_color(75, cmap_name), linewidth=2, linestyle=':')
        ax.text(x=max_bw, y=ax.get_ylim()[1] * 0.95, 
               s=f'Max. theoretical BW = {max_bw} GB/s', 
               horizontalalignment='right', fontsize=fontSizeMess - 4)
    
    ax.tick_params(axis='both', labelsize=fontSizeMess)
    
    base_x = 0.1
    base_y = 0.99
    add_gradient_legend(ax, None, 50, 100, colors=cmap_name, box_position=(base_x, base_y), box_size=(0.25, 0.03), fontsize=20)

    fig.set_size_inches([16, 9])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    plt.close()
    
    #print(f"Plot saved to: {output_path}")
    #print(f"Plot saved to: {png_path}")



def plot_combined_curves(datasets, output_path='combined_curves.pdf', limit_bw_override=None, limit_lat_override=None, ignore_guards=False, apply_corrections=True, draw_dots=False):
    """
    Plot multiple datasets on the same figure.
    datasets: list of tuples (label, dfs_rw, config, cmap_name)
    """
    if not datasets:
        print("No datasets to plot")
        return

    fig, ax = plt.subplots(1, 1)
    
    max_bw = 0.0
    global_max_bw = 0.0
    global_max_lat = 0.0
    for _, dfs_rw, cfg, _, _ in datasets:
        max_bw = max(max_bw, _compute_theoretical_max_bw(cfg))
        for _, df in dfs_rw.items():
            if not df.empty:
                if 'bandwidth_smooth' in df.columns and not df['bandwidth_smooth'].empty:
                    global_max_bw = max(global_max_bw, df['bandwidth_smooth'].max())
                if 'latency_smooth' in df.columns and not df['latency_smooth'].empty:
                    global_max_lat = max(global_max_lat, df['latency_smooth'].max())
    datasets = sorted(
        datasets,
        key=lambda d: max(
            (df['bandwidth_smooth'].max() for _, df in d[1].items() 
            if not df.empty and 'bandwidth_smooth' in df.columns and not df['bandwidth_smooth'].empty),
            default=0.0
        ),
        reverse=True  # largest bandwidth first
    )
    global_max_bw = 0.0
    global_max_lat = 0.0
    
    for _, dfs_rw, _, _, _ in datasets:
        for _, df in dfs_rw.items():
            if not df.empty:
                if 'bandwidth_smooth' in df.columns and not df['bandwidth_smooth'].empty:
                    global_max_bw = max(global_max_bw, df['bandwidth_smooth'].max())
                if 'latency_smooth' in df.columns and not df['latency_smooth'].empty:
                    global_max_lat = max(global_max_lat, df['latency_smooth'].max())
    
    limit_bw = limit_bw_override
    if limit_bw is None:
        if max_bw > 0:
            limit_bw = max_bw * 1.1
        else:
            limit_bw = global_max_bw * 1.1 if global_max_bw > 0 else 321

    limit_lat = limit_lat_override
    if limit_lat is None:
        limit_lat = max(700, global_max_lat * 1.1) if global_max_lat > 0 else 700
    
    ax.set_xlim([0, limit_bw])
    ax.set_ylim([0, limit_lat])
    ax.set_xlabel('Used Memory bandwidth [GB/s]', fontsize=fontSizeMess + 3)
    ax.set_ylabel('Memory access latency [ns]', fontsize=fontSizeMess + 3)
    # Starting position (top-left inside the axes)
    base_x = 0.1
    base_y = 0.99

    # Vertical spacing between boxes (in axes fraction)
    vertical_step = 0.08
    marker = 'o' if draw_dots else None
    markersize = 4 if draw_dots else None

    # Plot each dataset
    for i, (label, dfs_rw, _, cmap_name, _) in enumerate(datasets):
        y_pos = base_y - i * vertical_step
        add_gradient_legend(ax, label, 50, 100, colors=cmap_name,
                        box_position=(base_x, y_pos), box_size=(0.25, 0.03), fontsize=20)


        sorted_ratios = sorted(dfs_rw.keys())
        first_curve = True
        skipped_ratios = {}
        for rw in sorted_ratios:
            df = dfs_rw[rw]
            if not df.empty:
                if apply_corrections:
                    df = filter_clustered_points(df.copy(), 5.0)
                else:
                    df = df.copy()
                
                if (not apply_corrections) or ignore_guards:
                    curve_label = f'{label} (Rd:Wr {rw}:0)' if first_curve else None
                    first_curve = False
                    ax.plot(df['bandwidth_smooth'], df['latency_smooth'], color=calculate_color(rw, cmap_name),
                           linewidth=1, marker=marker, markersize=markersize, label=curve_label, alpha=1.0)
                else:
                    is_valid, reason = is_valid_curve(
                        df,
                        min_points=5,
                        max_cluster_fraction=0.70,
                        gap_fraction=0.5,
                        ignore_guards=False)
                    if is_valid:
                        curve_label = f'{label} (Rd:Wr {rw}:0)' if first_curve else None
                        first_curve = False
                        ax.plot(df['bandwidth_smooth'], df['latency_smooth'], color=calculate_color(rw, cmap_name),
                               linewidth=1, marker=marker, markersize=markersize, label=curve_label, alpha=1.0)
                    else:
                        if reason not in skipped_ratios:
                            skipped_ratios[reason] = []
                        skipped_ratios[reason].append(rw)
        
        for reason, ratios in skipped_ratios.items():
            ratio_values = [int(r) for r in ratios]
            ratio_values = [int(r) for r in ratios]
            #print(f"Warning: {label} - Ratios {ratio_values} not plotted: {reason}")
                
    # Check configs from all datasets for NVIDIA PMU and NVLINK BW
    is_any_nvidia_pmu = any(cfg.get("USE_NVIDIA_PMU", "False").lower() in ('true', '1', 't') for _, _, cfg, _, _ in datasets)
    any_nvlink_bw = any(float(cfg.get("NVLINK_BW_GB_S", 0)) > 0 for _, _, cfg, _, _ in datasets)

    should_draw_max_bw_line = not (is_any_nvidia_pmu and not any_nvlink_bw)

    if max_bw > 0 and should_draw_max_bw_line:
        ax.axvline(x=max_bw, color=calculate_color(75), linewidth=2, linestyle=':')
        ax.text(x=max_bw, y=limit_lat * 0.9, 
               s=f'Max. theoretical BW = {int(max_bw)} GB/s', 
               horizontalalignment='right', fontsize=fontSizeMess - 4)
    
    ax.tick_params(axis='both', labelsize=fontSizeMess)

  
    #custom_lines = [Line2D([0], [0], color=plt.colormaps[cmap](0.6), lw=4) for _, _, _, cmap, _ in datasets]
    #custom_labels = [label for label, _, _, _, _ in datasets]
    #ax.legend(custom_lines, custom_labels, fontsize=fontSizeMess)

    fig.set_size_inches([16, 9])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    plt.close()
    
    #print(f"Combined plot saved to: {output_path}")
    #print(f"Combined plot saved to: {png_path}")


def plot_combined_metric_curves(datasets, metric_col, y_label,
                                output_path='combined_metric_curves.pdf',
                                limit_bw_override=None, limit_metric_override=None,
                                ignore_guards=False, apply_corrections=True, draw_dots=False):
    """Plot one metric for multiple datasets on the same figure."""
    if not datasets:
        return

    fig, ax = plt.subplots(1, 1)

    max_bw = 0.0
    global_max_bw = 0.0
    global_max_metric = 0.0

    for _, dfs_rw, cfg, _, _ in datasets:
        max_bw = max(max_bw, _compute_theoretical_max_bw(cfg))
        for _, df in dfs_rw.items():
            if df.empty or 'bandwidth_plot' not in df.columns:
                continue
            value_col = 'metric_plot' if 'metric_plot' in df.columns else metric_col
            if value_col not in df.columns:
                continue
            df_local = df.dropna(subset=['bandwidth_plot', value_col])
            if df_local.empty:
                continue
            global_max_bw = max(global_max_bw, df_local['bandwidth_plot'].max())
            global_max_metric = max(global_max_metric, df_local[value_col].max())

    limit_bw = limit_bw_override
    if limit_bw is None:
        if max_bw > 0:
            limit_bw = max_bw * 1.1
        else:
            limit_bw = global_max_bw * 1.1 if global_max_bw > 0 else 321

    limit_metric = limit_metric_override
    if limit_metric is None:
        limit_metric = global_max_metric * 1.15 if global_max_metric > 0 else 1.0
        if limit_metric <= 0:
            limit_metric = 1.0

    ax.set_xlim([0, limit_bw])
    ax.set_ylim([0, limit_metric])
    ax.set_xlabel('Used Memory bandwidth [GB/s]', fontsize=fontSizeMess + 3)
    ax.set_ylabel(y_label, fontsize=fontSizeMess + 3)

    base_x = 0.1
    base_y = 0.99
    vertical_step = 0.08

    marker = 'o' if draw_dots else None
    markersize = 4 if draw_dots else None

    for i, (label, dfs_rw, _, cmap_name, _) in enumerate(datasets):
        y_pos = base_y - i * vertical_step
        add_gradient_legend(ax, label, 50, 100, colors=cmap_name,
                            box_position=(base_x, y_pos), box_size=(0.25, 0.03), fontsize=20)

        sorted_ratios = sorted(dfs_rw.keys())
        first_curve = True
        for rw in sorted_ratios:
            df = dfs_rw[rw]
            if df.empty or 'bandwidth_plot' not in df.columns:
                continue

            value_col = 'metric_plot' if 'metric_plot' in df.columns else metric_col
            if value_col not in df.columns:
                continue

            if apply_corrections:
                df = filter_clustered_points(df.copy(), 5.0, bw_col='bandwidth_plot')
            else:
                df = df.copy()
            df = df.dropna(subset=['bandwidth_plot', value_col])
            if df.empty:
                continue

            if (not apply_corrections) or ignore_guards:
                curve_label = f'{label} (Rd:Wr {rw}:0)' if first_curve else None
                first_curve = False
                ax.plot(df['bandwidth_plot'], df[value_col],
                        color=calculate_color(rw, cmap_name),
                        linewidth=1, marker=marker, markersize=markersize, label=curve_label, alpha=1.0)
            else:
                is_valid, _ = is_valid_curve(
                    df,
                    min_points=5,
                    max_cluster_fraction=0.70,
                    gap_fraction=0.5,
                    bw_column='bandwidth_plot',
                    ignore_guards=False)
                if is_valid:
                    curve_label = f'{label} (Rd:Wr {rw}:0)' if first_curve else None
                    first_curve = False
                    ax.plot(df['bandwidth_plot'], df[value_col],
                            color=calculate_color(rw, cmap_name),
                            linewidth=1, marker=marker, markersize=markersize, label=curve_label, alpha=1.0)

    is_any_nvidia_pmu = any(cfg.get("USE_NVIDIA_PMU", "False").lower() in ('true', '1', 't') for _, _, cfg, _, _ in datasets)
    any_nvlink_bw = any(float(cfg.get("NVLINK_BW_GB_S", 0)) > 0 for _, _, cfg, _, _ in datasets)
    should_draw_max_bw_line = not (is_any_nvidia_pmu and not any_nvlink_bw)

    if max_bw > 0 and should_draw_max_bw_line:
        ax.axvline(x=max_bw, color=calculate_color(75), linewidth=2, linestyle=':')
        ax.text(x=max_bw, y=limit_metric * 0.95,
                s=f'Max. theoretical BW = {int(max_bw)} GB/s',
                horizontalalignment='right', fontsize=fontSizeMess - 4)

    ax.tick_params(axis='both', labelsize=fontSizeMess)

    fig.set_size_inches([16, 9])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    plt.close()


def check_smooth_curves(df_rw,
                       gap_fraction=0.4,
                       bw_column='bandwidth_mean',
                       group_column='read_pct_rounded',
                       ignore_guards=False):

    if ignore_guards:
        return True, "valid (forced)"

    # Ensure we have the right bandwidth column
    if bw_column not in df_rw.columns:
        if 'bandwidth_smooth' in df_rw.columns:
            bw_column = 'bandwidth_smooth'
        elif 'bandwidth_mean' in df_rw.columns:
            bw_column = 'bandwidth_mean'
        elif 'bandwidth' in df_rw.columns:
            bw_column = 'bandwidth'
        else:
            return False, "missing_bandwidth_column"

    smooth = True
    problematic_groups = []
    # GROUP BY read percentage → each curve
    if 'read_pct_rounded' not in df_rw.columns: 
        group_column="rd_percentage"
    for read_pct, group in df_rw.groupby(group_column):

        bw = group[bw_column].dropna()

        if len(bw) < 2:
            continue  # nothing to compare

        # Sort values
        bw_sorted = np.sort(bw)

        # Compute gaps
        gaps = np.diff(bw_sorted)
        max_gap = gaps.max()
        span = bw_sorted.max() - bw_sorted.min()

        # Avoid division issues if flat curve
        if span == 0:
            continue

        max_allowed_gap = span * gap_fraction

        if max_gap > max_allowed_gap:
            smooth = False
            problematic_groups.append(read_pct)

    return smooth, problematic_groups

