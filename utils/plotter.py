#!/usr/bin/env python3
import argparse
import os
import re

import pandas as pd
from scipy.signal import savgol_filter

from libs.parser import parse_bwlat_file, parse_filename, csv_to_json_compact
from libs.processor import process_data, calculate_mean_curves, smooth_curves, filter_incomplete_ratios
from libs.visualizer import plot_curves, plot_combined_curves,check_smooth_curves



COLORMAPS = ("Blues", "Reds", "Greens", "Oranges", "Purples", "Greys")

PEBS_LATENCY_SPECS = (
        ("mean_ns", "latency_mean.pdf", "Mean Latency [ns]"),
        ("min_ns", "latency_min.pdf", "Min Latency [ns]"),
        ("median_ns", "latency_median.pdf", "Median Latency [ns]"),
        ("p90_ns", "latency_p90.pdf", "P90 Latency [ns]"),
        ("p95_ns", "latency_p95.pdf", "P95 Latency [ns]"),
        ("p99_ns", "latency_p99.pdf", "P99 Latency [ns]"),
        ("p99_9_ns", "latency_p99_9.pdf", "P99.9 Latency [ns]"),
        ("max_ns", "latency_max.pdf", "Max Latency [ns]"),
        )


def _plot_percentile_curves(config, dfs_rw, processed_dir, cmap_name,
        ignore_guards=False, apply_corrections=True, draw_dots=False):
    """Generate separate BW-latency PDFs for each PEBS percentile."""
    sample_df = next(iter(dfs_rw.values()), None)
    if sample_df is None:
        return

    for metric_col, filename, y_label in PEBS_LATENCY_SPECS:
        if metric_col not in sample_df.columns:
            continue

        modified_dfs = {}
        for rw, df in dfs_rw.items():
            if metric_col in df.columns:
                df_mod = df.copy()
                df_mod['latency_smooth'] = df_mod[metric_col]
                modified_dfs[rw] = df_mod

        if modified_dfs:
            plot_curves(
                    config,
                    modified_dfs,
                    os.path.join(processed_dir, filename),
                    cmap_name,
                    ignore_guards=ignore_guards,
                    apply_corrections=apply_corrections,
                    y_label=y_label,
                    draw_dots=draw_dots,
                    )


def _parse_first_float(text):
    match = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
    if not match:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def _build_metric_curves(df_for_metrics, ratio_col, metric_col, apply_smoothing=True, apply_corrections=True):
    if df_for_metrics.empty or metric_col not in df_for_metrics.columns:
        return {}

    bw_col = None
    for candidate in ("bandwidth_mean", "bandwidth_smooth", "bandwidth"):
        if candidate in df_for_metrics.columns:
            bw_col = candidate
            break
    if bw_col is None:
        return {}

    required = [ratio_col, "pause", bw_col, metric_col]
    missing = [col for col in required if col not in df_for_metrics.columns]
    if missing:
        return {}

    dfs_metric = {}
    for rw in sorted(df_for_metrics[ratio_col].dropna().unique()):
        df_rw = df_for_metrics[df_for_metrics[ratio_col] == rw][required].copy()
        df_rw = df_rw.dropna(subset=[bw_col, metric_col])
        if df_rw.empty:
            continue

        #df_rw = df_rw.sort_values(bw_col).reset_index(drop=True)
        if apply_corrections:
            #min_metric = df_rw[metric_col].min()
            #zero_row = df_rw.iloc[0].copy()
            #zero_row[bw_col] = 0
            #zero_row[metric_col] = min_metric
            #df_rw = pd.concat([pd.DataFrame([zero_row]), df_rw], ignore_index=True)
            df_rw = df_rw.sort_values(bw_col).reset_index(drop=True)
            df_for_csv = filter_incomplete_ratios(df_rw, ratio_col)
        if apply_smoothing:
            try:
                x = df_for_csv[bw_col].to_numpy()
                y = df_for_csv[metric_col].to_numpy()
                smooth_x, smooth_y = savgol_filter((x, y), window_length=11, polyorder=3)
                df_rw["bandwidth_plot"] = x
                df_rw["metric_plot"] = y

            except (ValueError, Exception):
                df_rw["bandwidth_plot"] = df_rw[bw_col]
                df_rw["metric_plot"] = df_rw[metric_col]
        else:
            df_rw["bandwidth_plot"] = df_rw[bw_col]
            df_rw["metric_plot"] = df_rw[metric_col]

        dfs_metric[rw] = df_rw

    return dfs_metric


def _collect_bwlat_rows(measuring_dir, config):
    rows = []
    for source_dir in (measuring_dir, os.path.join(measuring_dir, "curves")):
        if not os.path.exists(source_dir):
            continue
        for filename in os.listdir(source_dir):
            if not (filename.startswith("bwlat_") and filename.endswith(".txt")):
                continue
            rd_percentage, pause = parse_filename(filename)
            if rd_percentage is None:
                continue
            filepath = os.path.join(source_dir, filename)
            data_points = parse_bwlat_file(filepath, config)
            if not data_points:
                continue
            for point in data_points:
                rows.append(
                        {
                            "bandwidth": point["bandwidth"],
                            "latency": point["latency"],
                            "rd_percentage": rd_percentage,
                            "read_pct_rounded": rd_percentage,
                            "pause": pause,
                            }
                        )
    return rows


def _build_curves_table(dfs_rw, read_label):
    return pd.concat([df.assign(**{read_label: key}) for key, df in dfs_rw.items()], ignore_index=True)


def _build_summary(df_for_csv):
    summary = {}
    if "bandwidth" in df_for_csv.columns:
        summary["bw_min"] = df_for_csv["bandwidth"].min()
        summary["bw_max"] = df_for_csv["bandwidth"].max()
    elif "bandwidth_mean" in df_for_csv.columns:
        summary["bw_min"] = df_for_csv["bandwidth_mean"].min()
        summary["bw_max"] = df_for_csv["bandwidth_mean"].max()

    if "latency" in df_for_csv.columns:
        summary["lat_min"] = df_for_csv["latency"].min()
        summary["lat_max"] = df_for_csv["latency"].max()
    elif "latency_mean" in df_for_csv.columns:
        summary["lat_min"] = df_for_csv["latency_mean"].min()
        summary["lat_max"] = df_for_csv["latency_mean"].max()

    return summary



def _load_metric_curves_from_processed(measuring_dir, apply_smoothing, apply_corrections):
    processed_csv = os.path.join(measuring_dir, "processed", "memory_curves.csv")
    if not os.path.exists(processed_csv):
        return {}

    try:
        df_processed = pd.read_csv(processed_csv)
    except Exception:
        return {}

    ratio_col = "read_pct_rounded" if "read_pct_rounded" in df_processed.columns else "rd_percentage"
    metric_curves = {}
    return metric_curves


def _plot_first_available_combined(first_dir, metric_datasets, specs, plot_fn, output_filename, ignore_guards, apply_corrections, draw_dots):
    for metric_col, y_label in specs:
        selected_sets = []
        for label, metrics_map, cfg, cmap_name, measuring_dir in metric_datasets:
            curves = metrics_map.get(metric_col, {})
            if curves:
                selected_sets.append((label, curves, cfg, cmap_name, measuring_dir))
        if len(selected_sets) >= 2:
            plot_fn(
                    selected_sets,
                    metric_col=metric_col,
                    y_label=y_label,
                    output_path=os.path.join(first_dir, "processed", output_filename),
                    ignore_guards=ignore_guards,
                    apply_corrections=apply_corrections,
                    draw_dots=draw_dots,
                    )
            return True
    return False


def process_directory(measuring_dir, step, mode, cmap_name="Blues", ignore_guards=False, apply_smoothing=True, apply_corrections=True, draw_dots=False):
    if not os.path.exists(measuring_dir):
        return None, None, None

    processed_dir = os.path.join(measuring_dir, "processed")
    os.makedirs(processed_dir, exist_ok=True)

    df_bw, df_lat, config = process_data(measuring_dir)
    if df_bw.empty or df_lat.empty:
        return None, None, None

    has_bw_lat_dirs = os.path.exists(os.path.join(measuring_dir, "bw")) and os.path.exists(os.path.join(measuring_dir, "lat"))
    has_bwlat_files = any(
            filename.startswith("bwlat_") and filename.endswith(".txt")
            for filename in os.listdir(measuring_dir)
            )


    if has_bwlat_files and not has_bw_lat_dirs:
        read_label = "read_pct_rounded"
        df_for_csv = pd.DataFrame(_collect_bwlat_rows(measuring_dir, config))
        if df_for_csv.empty:
            return None, None, None
        if apply_corrections and not ignore_guards:
            df_for_csv = filter_incomplete_ratios(df_for_csv, read_label)
        #if not df_for_csv.empty:
        #df_for_csv = df_for_csv.sort_values(["read_pct_rounded", "bandwidth"])

        if(check_smooth_curves(df_for_csv, gap_fraction=0.4,ignore_guards=False)):
            dfs_rw = smooth_curves(
                df_for_csv,
                mode,
                apply_smoothing=apply_smoothing,
                apply_corrections=apply_corrections,
            )
    else:
        read_label = "read_pct_rounded" if mode == "perread" else "rd_percentage"
        df_for_csv = calculate_mean_curves(df_bw, df_lat, step, mode)
        if df_for_csv.empty:
            return None, None, None

        if read_label not in df_for_csv.columns:
            fallback_label = "read_pct_rounded" if read_label == "rd_percentage" else "rd_percentage"
            if fallback_label in df_for_csv.columns:
                read_label = fallback_label

        if apply_corrections and not ignore_guards:
            df_for_csv = filter_incomplete_ratios(df_for_csv, read_label)

        if(check_smooth_curves(df_for_csv, gap_fraction=0.4,ignore_guards=False)):
            dfs_rw = smooth_curves(
                df_for_csv,
                mode,
                apply_smoothing=apply_smoothing,
                apply_corrections=apply_corrections,
            )

    if not dfs_rw:
        return None, None, None

    #dfs_rw = {key: df.sort_values("bandwidth_mean").reset_index(drop=True) for key, df in dfs_rw.items()}
    df_curves_all = _build_curves_table(dfs_rw, read_label)

    plot_curves(
        config,
        dfs_rw,
        os.path.join(processed_dir, "memory_curves.pdf"),
        cmap_name,
        ignore_guards=ignore_guards,
        apply_corrections=apply_corrections,
        draw_dots=draw_dots,
    )

    _plot_percentile_curves(
        config, dfs_rw, processed_dir, cmap_name,
        ignore_guards=ignore_guards,
        apply_corrections=apply_corrections,
        draw_dots=draw_dots,
    )


    csv_path = os.path.join(processed_dir, "memory_curves.csv")
    df_curves_all.to_csv(csv_path, index=False)

    json_path = os.path.join(processed_dir, "memory_curves.json")
    csv_to_json_compact(csv_path, json_path, read_label, "pause", "bandwidth_smooth", "latency_smooth")


    return dfs_rw, config, _build_summary(df_for_csv)


def main():
    parser = argparse.ArgumentParser(
            description="Generate memory bandwidth-latency roofline plots",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
            )
    parser.add_argument("directories", nargs="+", help="One or more measurement directories")
    parser.add_argument(
            "--mode",
        choices=["perread", "perkernel"],
        default="perread",
        help="Plotting mode: group curves by kernel (default) or by measured read ratio",
    )
    parser.add_argument(
        "--step",
        choices=["1", "2", "4", "5", "10", "50"],
        default="1",
        help="Plotting step: plot curves in steps of 1, 2, 4, 5, 10 or 50",
    )
    parser.add_argument(
        "--progress",
        action="store_true",
        help="Ignore validity checks and plot in-progress curves",
    )
    parser.add_argument(
        "--no-smooth",
        action="store_true",
        help="Disable Savitzky-Golay smoothing and use original curve points",
    )
    parser.add_argument(
        "--no-correct",
        action="store_true",
        help="Disable curve corrections/filters (ratio filtering, anchor insertion, and curve guards)",
    )
    parser.add_argument(
        "--dots",
        action="store_true",
        help="Add dot markers to the output curve plots",
    )

    args = parser.parse_args()

    directories = [os.path.abspath(directory) for directory in args.directories]
    ignore_guards = args.progress
    apply_smoothing = not args.no_smooth
    apply_corrections = not args.no_correct
    draw_dots = args.dots

    curve_cmap = "Blues"
    datasets = []
    metric_datasets = []
    summary_list = []

    print("Processing directories...", end="", flush=True)

    for index, measuring_dir in enumerate(directories):
        print(".", end="", flush=True)
        dataset_cmap = COLORMAPS[index % len(COLORMAPS)]
        dfs_rw, config, summary = process_directory(
            measuring_dir,
            args.step,
            args.mode,
            curve_cmap,
            ignore_guards=ignore_guards,
            apply_smoothing=apply_smoothing,
            apply_corrections=apply_corrections,
            draw_dots=draw_dots,
        )

        if not dfs_rw:
            continue

        label = os.path.basename(os.path.normpath(measuring_dir))
        datasets.append((label, dfs_rw, config, dataset_cmap, measuring_dir))
        metric_curves = _load_metric_curves_from_processed(measuring_dir, apply_smoothing, apply_corrections)
        metric_datasets.append((label, metric_curves, config, dataset_cmap, measuring_dir))

        if summary:
            summary["label"] = label
            summary["dir"] = measuring_dir
            summary_list.append(summary)

    print(" Done!")

    if not datasets:
        return

    for _, dfs_rw, config, cmap_name, measuring_dir in datasets:
        plot_curves(
            config,
            dfs_rw,
            os.path.join(measuring_dir, "processed", "memory_curves.pdf"),
            cmap_name,
            ignore_guards=ignore_guards,
            apply_corrections=apply_corrections,
            draw_dots=draw_dots,
        )


    if len(datasets) > 1:
        first_dir = directories[0]
        plot_combined_curves(
            datasets,
            os.path.join(first_dir, "processed", "combined_memory_curves.pdf"),
            ignore_guards=ignore_guards,
            apply_corrections=apply_corrections,
            draw_dots=draw_dots,
        )


    print("\n\n======================================================================================")
    print("                                 PLOTTING SUMMARY")
    print("======================================================================================")
    print(f"{'Directory':<30} | {'Bandwidth Range (GB/s)':<25} | {'Latency Range (ns)':<25}")
    print("-" * 86)

    for item in summary_list:
        bw_range = f"{item.get('bw_min', 0):.2f} - {item.get('bw_max', 0):.2f}"
        lat_range = f"{item.get('lat_min', 0):.1f} - {item.get('lat_max', 0):.1f}"
        print(f"{item['label']:<30} | {bw_range:<25} | {lat_range:<25}")
        rel_dir = os.path.relpath(item["dir"], os.getcwd())
        print(f"   -> {rel_dir}/processed")

    if len(datasets) > 1:
        first_dir = directories[0]
        rel_first_dir = os.path.relpath(first_dir, os.getcwd())
        print("-" * 86)
        print("Combined plot saved to:")
        print(f"   -> {rel_first_dir}/processed/combined_memory_curves.pdf")

    print("======================================================================================\n")


if __name__ == "__main__":
    main()
