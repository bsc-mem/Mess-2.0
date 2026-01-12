import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import matplotlib.patches as patches
import seaborn as sns
from matplotlib.lines import Line2D
from matplotlib.colors import LinearSegmentedColormap, Normalize

fontSizeMess = 24

def calculate_color(rw, cmap_name='Blues'):
    """Returns a color based on read-write ratio using specified colormap
       Higher rw → lighter color, lower rw → darker color.
       Contrast between darkest and lightest shades increased.
    """
    cmap = plt.colormaps[cmap_name]

    # Define the min and max positions in the colormap (0=darkest, 1=lightest)
    min_val = 0.1  # darkest used
    max_val = 1  # lightest used

    # Clip rw to 0-100 just in case
    rw_clipped = max(50.0, min(100.0, rw))

    # Normalize rw to 0-1
    normalized = rw_clipped / 100.0

    # Invert for higher rw → lighter color
    c = max_val - (max_val - min_val) * normalized

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




def filter_clustered_points(df, bw_tolerance_gb=8.0):
    """
    Keep only points that are at least `bw_tolerance_gb` apart in bandwidth.
    Preserves monotonicity and overall shape.
    """
    if df.empty or len(df) <= 1:
        return df

    df = df.sort_values('bandwidth_smooth').reset_index(drop=True)
    
    keep_idx = [0]  # Always keep first point
    last_bw = df.iloc[0]['bandwidth_smooth']

    for i in range(1, len(df)):
        current_bw = df.iloc[i]['bandwidth_smooth']
        if current_bw - last_bw >= bw_tolerance_gb:
            keep_idx.append(i)
            last_bw = current_bw

    # Always keep the last point (important for high-BW saturation)
    if keep_idx[-1] != len(df) - 1:
        keep_idx.append(len(df) - 1)

    return df.iloc[keep_idx].reset_index(drop=True)

def is_valid_curve(df,
                   min_points=5,
                   max_cluster_fraction=0.4,
                   gap_fraction=0.5,        # NEW: max allowed jump in GB/s
                   bw_column='bandwidth_mean'):
    """
    Comprehensive validity check:
    - Spans enough bandwidth
    - Not a tight cluster
    - No huge straight-line jumps (artifacts)
    """

    if df.empty or len(df) < min_points:
        return False, "too_few_points"

    bw = df[bw_column].dropna()
    if len(bw) < min_points:
        return False, "too_few_valid_bw"

    if (df['bandwidth_mean'].max() - df['bandwidth_mean'].min()) < 80: 
        return False, "cluster of dots, not valid as a curve"
    # Check for large horizontal jumps
    bw_sorted = np.sort(bw)
    gaps = np.diff(bw_sorted)
    max_gap = gaps.max() if len(gaps) > 0 else 0
    span = bw.max() - bw.min()
    max_horizontal_gap = span * gap_fraction
    

    if max_gap > max_horizontal_gap:
        return False, f"large_jump ({max_gap:.1f} > {max_horizontal_gap} GB/s)"

    # Check for clustering (optional but recommended)
    if max_cluster_fraction < 1.0:
        window = 50.0
        # Use rolling count on sorted values
        counts_in_windows = []
        for i in range(len(bw_sorted)):
            window_count = np.sum((bw_sorted >= bw_sorted[i]) & (bw_sorted <= bw_sorted[i] + window))
            counts_in_windows.append(window_count)

        if max(counts_in_windows) / len(bw) > max_cluster_fraction:
            return False, "too_clustered"

    return True, "valid"



def plot_curves(config, dfs_rw, output_path='memory_curves.pdf', cmap_name='Blues', limit_bw_override=None, limit_lat_override=None):
    """Plot bandwidth-latency curves and save to PDF"""
    if not dfs_rw:
        print("No data to plot")
        return
    
    fig, ax = plt.subplots(1, 1)
    
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

    limit_bw = limit_bw_override
    if limit_bw is None:
        if max_bw > 0:
            limit_bw = max_bw
            if global_max_bw > limit_bw:
                 limit_bw = global_max_bw * 1.1
        else:
            limit_bw = global_max_bw * 1.1 if global_max_bw > 0 else 321

    limit_lat = limit_lat_override
    if limit_lat is None:
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

    
    is_nvidia_pmu = config.get("USE_NVIDIA_PMU", "False").lower() in ('true', '1', 't')
    nvlink_bw = float(config.get("NVLINK_BW_GB_S", 0))
    
    # Do not draw the line for NVIDIA counters if NVLINK_BW is not specified
    should_draw_max_bw_line = not (is_nvidia_pmu and nvlink_bw == 0)

    if max_bw > 0 and should_draw_max_bw_line:
        ax.axvline(x=max_bw, color='red', linewidth=2, linestyle=':')
        ax.text(x=max_bw, y=ax.get_ylim()[1] * 0.95, 
               s=f'Max. theoretical BW = {max_bw} GB/s', 
               horizontalalignment='right', fontsize=fontSizeMess - 4)
    
    ax.tick_params(axis='both', labelsize=fontSizeMess)
    
    base_x = 0.1
    base_y = 0.99
    add_gradient_legend(ax, None, 50, 100, colors=cmap_name,box_position=(base_x, base_y), box_size=(0.25, 0.03), fontsize=20)

    fig.set_size_inches([16, 9])
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    plt.close()
    
    print(f"Plot saved to: {output_path}")
    print(f"Plot saved to: {png_path}")

def plot_combined_curves(datasets, output_path='combined_curves.pdf', limit_bw_override=None, limit_lat_override=None):
    """
    Plot multiple datasets on the same figure.
    datasets: list of tuples (label, dfs_rw, config, cmap_name)
    """
    if not datasets:
        print("No datasets to plot")
        return

    fig, ax = plt.subplots(1, 1)
    
    max_bw = 0
    global_max_bw = 0.0
    global_max_lat = 0.0
    try:
        for _, dfs_rw, cfg, _, _ in datasets:
            nvlink_bw_str = cfg.get("NVLINK_BW_GB_S", None)
            nvlink_bw = float(nvlink_bw_str) if nvlink_bw_str is not None else None

            if nvlink_bw is not None and nvlink_bw == 0:
                pass
            elif nvlink_bw is not None and nvlink_bw > 0:
                max_bw = max(max_bw, nvlink_bw)
            else:
                binding = cfg.get("MEMORY_BINDING", "local").strip().lower()
                if binding == "remote":
                    upi_freq = float(cfg.get("UPI_FREQ", 16))
                    n_lanes = float(cfg.get("N_DATA_LANES", 20))
                    flit_bit = float(cfg.get("FLIT_BIT", 80))
                    data_flit_bit = float(cfg.get("DATA_FLIT_BIT", 64))
                    n_upi_channels = float(cfg.get("N_UPI_CHANNELS", 4))
                    if data_flit_bit > 0 and flit_bit > 0:
                        max_bw = max(max_bw, 2.0 * (upi_freq * n_lanes * (data_flit_bit / flit_bit) * n_upi_channels) / 8.0)
                else:
                    freq_str = cfg.get("MEM_FREQ", "")
                    if freq_str and "Could not detect" not in freq_str:
                        freq = float(freq_str.split()[0])
                    else:
                        freq = 0
                    channels = float(cfg.get("N_CHANNELS", 2))
                    bus_width = float(cfg.get("BUS_WIDTH", 64))
                    if bus_width > 0 and freq > 0:
                        max_bw = max(max_bw, (bus_width / 8) * freq * channels / 1000.0)

            for _, df in dfs_rw.items():
                if not df.empty:
                    if 'bandwidth_smooth' in df.columns and not df['bandwidth_smooth'].empty:
                        global_max_bw = max(global_max_bw, df['bandwidth_smooth'].max())
                    if 'latency_smooth' in df.columns and not df['latency_smooth'].empty:
                        global_max_lat = max(global_max_lat, df['latency_smooth'].max())
    except (ValueError, AttributeError):
        max_bw = max_bw if max_bw else 0
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
            limit_bw = max_bw
            if global_max_bw > limit_bw:
                 limit_bw = global_max_bw * 1.1
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
                df = filter_clustered_points(df.copy(), 5.0)
                
                is_valid, reason = is_valid_curve(
                    df,
                    min_points=5,
                    max_cluster_fraction=0.70,
                    gap_fraction=0.5)
                if is_valid:
                    curve_label = f'{label} (Rd:Wr {rw}:0)' if first_curve else None
                    first_curve = False
                    ax.plot(df['bandwidth_smooth'], df['latency_smooth'], color=calculate_color(rw, cmap_name),
                           label=curve_label, alpha=1.0)
                    # Plot dots
                    #ax.scatter(
                    #    df['bandwidth_smooth'],
                    #    df['latency_smooth'],
                    #    s=12,          # dot size — ONLY here
                    #    color="Black",
                    #    alpha=0.8
                    #)
                    #for x, y,  pause_value in zip(
                    #df['bandwidth_smooth'],
                    #df['latency_smooth'],
                    #df['pause']              # same here
                    #):
                    #    ax.text(
                    #        x, y,
                    #        f"{rw},{pause_value}",
                    #        fontsize=6,
                    #        color="Black",
                    #        alpha=0.8,
                    #        ha='left',
                    #        va='bottom'
                    #        )
                else:
                    if reason not in skipped_ratios:
                        skipped_ratios[reason] = []
                    skipped_ratios[reason].append(rw)
        
        for reason, ratios in skipped_ratios.items():
            ratio_values = [int(r) for r in ratios]
            print(f"Warning: {label} - Ratios {ratio_values} not plotted: {reason}")
                
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
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches='tight', dpi=300)
    png_path = os.path.splitext(output_path)[0] + '.png'
    fig.savefig(png_path, bbox_inches='tight', dpi=300)
    plt.close()
    
    print(f"Combined plot saved to: {output_path}")
    print(f"Combined plot saved to: {png_path}")
