#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# Basic helpers
# ============================================================

def nearest_index(values, q):
    values = np.asarray(values, dtype=float)
    return int(np.argmin(np.abs(values - float(q))))


def nearest_yaw_index(yaw_bins_rad, q_yaw):
    yaw_bins_rad = np.asarray(yaw_bins_rad, dtype=float)
    diffs = np.abs(((yaw_bins_rad - float(q_yaw) + np.pi) % (2.0 * np.pi)) - np.pi)
    return int(np.argmin(diffs))


# ============================================================
# Map loading
# ============================================================

def _validate_map_exists(path):
    if not os.path.exists(path):
        raise IOError("Map file does not exist: %s" % path)


def _validate_map_keys(data, path):
    required = ["x_bins", "y_bins", "yaw_bins_rad", "reachable"]
    for k in required:
        if k not in data:
            raise KeyError("Missing key '%s' in map file %s" % (k, path))


def _extract_map_arrays(data):
    x_bins = np.asarray(data["x_bins"], dtype=float)
    y_bins = np.asarray(data["y_bins"], dtype=float)
    yaw_bins_rad = np.asarray(data["yaw_bins_rad"], dtype=float)
    reachable = np.asarray(data["reachable"])
    fixed_z = float(data["fixed_z"]) if "fixed_z" in data else 1.0
    quality = np.asarray(data["quality"], dtype=float) if "quality" in data else None
    quality_threshold = float(data["quality_threshold"]) if "quality_threshold" in data else None
    return x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold


def _validate_reachable_shape(reachable):
    if reachable.ndim != 3:
        raise ValueError("reachable must be 3D")


def load_map(path):
    _validate_map_exists(path)

    data = np.load(path, allow_pickle=False)
    _validate_map_keys(data, path)

    x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold = \
        _extract_map_arrays(data)
    _validate_reachable_shape(reachable)

    return x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold


# ============================================================
# Summary printing
# ============================================================

def _reachable_stats(reachable):
    total = reachable.size
    reachable_count = int(np.sum(reachable > 0.5))
    pct = 100.0 * float(reachable_count) / float(total) if total > 0 else 0.0
    return total, reachable_count, pct


def print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                  quality=None, quality_threshold=None):
    total, reachable_count, pct = _reachable_stats(reachable)

    print("Reachability map summary")
    print("------------------------")
    print("x bins:   %d  [%.3f .. %.3f]" % (len(x_bins), x_bins[0], x_bins[-1]))
    print("y bins:   %d  [%.3f .. %.3f]" % (len(y_bins), y_bins[0], y_bins[-1]))
    print("yaw bins: %d  [%.1f .. %.1f] deg" % (
        len(yaw_bins_rad),
        np.degrees(yaw_bins_rad[0]),
        np.degrees(yaw_bins_rad[-1]),
    ))
    print("fixed_z:  %.3f" % fixed_z)
    print("shape:    %s" % (str(reachable.shape),))
    print("map type: %s" % ("quality" if quality is not None else "binary"))
    if quality is not None:
        print("quality:  min=%.3f  mean=%.3f  max=%.3f" % (
            float(np.min(quality)), float(np.mean(quality)), float(np.max(quality))
        ))
        if quality_threshold is not None:
            print("threshold: %.3f" % quality_threshold)
    print("reachable cells: %d / %d (%.2f%%)" % (reachable_count, total, pct))


# ============================================================
# Plot helpers
# ============================================================

def _show_or_save(save_path):
    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print("Saved figure to:", save_path)
    else:
        plt.show()


def _yaw_slice_image(data, iyaw):
    # data shape: [Nx, Ny, Nyaw]; image wants rows/cols, so transpose to [Ny, Nx]
    return data[:, :, iyaw].T


def _xy_extent(x_bins, y_bins):
    return [x_bins[0], x_bins[-1], y_bins[0], y_bins[-1]]


# ============================================================
# Public plotting functions
# ============================================================

def plot_yaw_slice(x_bins, y_bins, yaw_bins_rad, reachable, yaw_deg,
                   quality=None, save_path=None):
    yaw_rad = np.radians(yaw_deg)
    iyaw = nearest_yaw_index(yaw_bins_rad, yaw_rad)
    yaw_used_deg = np.degrees(yaw_bins_rad[iyaw])

    if quality is not None:
        data = quality
        cbar_label = "quality score"
        vmin, vmax = 0.0, 1.0
    else:
        data = reachable.astype(float)
        cbar_label = "reachable"
        vmin, vmax = 0.0, 1.0

    img = _yaw_slice_image(data, iyaw)
    extent = _xy_extent(x_bins, y_bins)

    plt.figure(figsize=(8, 6))
    plt.imshow(
        img,
        origin="lower",
        extent=extent,
        aspect="auto",
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
        cmap="viridis",
    )
    plt.xlabel("x in robot frame [m]")
    plt.ylabel("y in robot frame [m]")
    plt.title("Reachability slice at yaw = %.1f deg" % yaw_used_deg)
    plt.colorbar(label=cbar_label)

    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print("Saved yaw slice figure to:", save_path)
    else:
        plt.show()


def plot_xy_yaw_profile(x_bins, y_bins, yaw_bins_rad, reachable, x_query, y_query,
                        quality=None, save_path=None):
    ix = nearest_index(x_bins, x_query)
    iy = nearest_index(y_bins, y_query)

    x_used = x_bins[ix]
    y_used = y_bins[iy]

    if quality is not None:
        vals = quality[ix, iy, :]
        ylabel = "quality score"
    else:
        vals = reachable[ix, iy, :].astype(float)
        ylabel = "reachable"

    yaw_deg_arr = np.degrees(yaw_bins_rad)

    plt.figure(figsize=(8, 4))
    plt.plot(yaw_deg_arr, vals, marker="o")
    plt.xlabel("yaw [deg]")
    plt.ylabel(ylabel)
    plt.ylim(-0.05, 1.05)
    plt.title("Yaw profile at x=%.3f, y=%.3f" % (x_used, y_used))
    plt.grid(True)

    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print("Saved yaw profile figure to:", save_path)
    else:
        plt.show()


# ============================================================
# CLI helpers
# ============================================================

def _build_arg_parser():
    parser = argparse.ArgumentParser(description="Visualize offline reachability map.")
    parser.add_argument("--map", required=True, help="Path to reachability_map.npz")

    parser.add_argument("--print-summary", action="store_true",
                        help="Print basic info about the map")

    parser.add_argument("--yaw-slice-deg", type=float, default=None,
                        help="Show 2D x-y slice at nearest yaw bin")
    parser.add_argument("--yaw-slice-save", default="",
                        help="Optional path to save yaw slice figure instead of showing it")

    parser.add_argument("--xy-yaw-profile", action="store_true",
                        help="Plot yaw profile for one (x,y) cell")
    parser.add_argument("--x", type=float, default=0.50,
                        help="x query for yaw profile")
    parser.add_argument("--y", type=float, default=0.0,
                        help="y query for yaw profile")
    parser.add_argument("--xy-yaw-save", default="",
                        help="Optional path to save x-y yaw profile figure instead of showing it")

    return parser


def _run_requested_outputs(args, x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                           quality=None, quality_threshold=None):
    did_anything = False

    if args.print_summary:
        print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                      quality=quality, quality_threshold=quality_threshold)

    if args.yaw_slice_deg is not None:
        plot_yaw_slice(
            x_bins=x_bins,
            y_bins=y_bins,
            yaw_bins_rad=yaw_bins_rad,
            reachable=reachable,
            yaw_deg=args.yaw_slice_deg,
            quality=quality,
            save_path=args.yaw_slice_save if args.yaw_slice_save else None,
        )
        did_anything = True

    if args.xy_yaw_profile:
        plot_xy_yaw_profile(
            x_bins=x_bins,
            y_bins=y_bins,
            yaw_bins_rad=yaw_bins_rad,
            reachable=reachable,
            x_query=args.x,
            y_query=args.y,
            quality=quality,
            save_path=args.xy_yaw_save if args.xy_yaw_save else None,
        )
        did_anything = True

    return did_anything


def _print_default_usage_hint():
    print("")
    print("No plot option selected. Example usage:")
    print("  --yaw-slice-deg 0")
    print("  --xy-yaw-profile --x 0.5 --y 0.0")


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold = \
        load_map(args.map)

    did_anything = _run_requested_outputs(
        args=args,
        x_bins=x_bins,
        y_bins=y_bins,
        yaw_bins_rad=yaw_bins_rad,
        reachable=reachable,
        fixed_z=fixed_z,
        quality=quality,
        quality_threshold=quality_threshold,
    )

    if not args.print_summary and not did_anything:
        print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                      quality=quality, quality_threshold=quality_threshold)
        _print_default_usage_hint()


if __name__ == "__main__":
    main()