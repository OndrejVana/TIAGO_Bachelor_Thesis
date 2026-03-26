#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Visualise offline arm reachability maps.

Single-arm mode  (original behaviour, fully backward-compatible):
  --map PATH

Dual-arm mode (TIAGo++):
  --map-right PATH --map-left PATH

In dual-arm mode every plot shows three panels:
  Right arm | Left arm | Union (right OR left)
A difference plot is also available via --diff-slice-deg.
"""

from __future__ import print_function, division, unicode_literals

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


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
    for k in ["x_bins", "y_bins", "yaw_bins_rad", "reachable"]:
        if k not in data:
            raise KeyError("Missing key '%s' in map file %s" % (k, path))


def _extract_map_arrays(data):
    x_bins          = np.asarray(data["x_bins"],       dtype=float)
    y_bins          = np.asarray(data["y_bins"],       dtype=float)
    yaw_bins_rad    = np.asarray(data["yaw_bins_rad"], dtype=float)
    reachable       = np.asarray(data["reachable"])
    fixed_z         = float(data["fixed_z"])          if "fixed_z"          in data else 1.0
    quality         = np.asarray(data["quality"], dtype=float) if "quality" in data else None
    quality_threshold = float(data["quality_threshold"]) if "quality_threshold" in data else None
    return x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold


def load_map(path):
    _validate_map_exists(path)
    data = np.load(path, allow_pickle=False)
    _validate_map_keys(data, path)
    x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold = \
        _extract_map_arrays(data)
    if reachable.ndim != 3:
        raise ValueError("reachable must be 3-D [Nx, Ny, Nyaw] in %s" % path)
    return x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold


def _maps_compatible(ma, mb):
    """Return True when two maps share identical bin grids."""
    return (
        np.allclose(ma[0], mb[0]) and   # x_bins
        np.allclose(ma[1], mb[1]) and   # y_bins
        np.allclose(ma[2], mb[2])        # yaw_bins_rad
    )


# ============================================================
# Summary printing
# ============================================================

def _reachable_stats(reachable):
    total           = reachable.size
    reachable_count = int(np.sum(reachable > 0.5))
    pct             = 100.0 * float(reachable_count) / float(total) if total > 0 else 0.0
    return total, reachable_count, pct


def _print_one_summary(label, x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                        quality=None, quality_threshold=None):
    total, reachable_count, pct = _reachable_stats(reachable)
    header = "Reachability map summary — %s" % label
    print(header)
    print("-" * len(header))
    print("  x bins   : %d  [%.3f .. %.3f] m" % (len(x_bins),  x_bins[0],  x_bins[-1]))
    print("  y bins   : %d  [%.3f .. %.3f] m" % (len(y_bins),  y_bins[0],  y_bins[-1]))
    print("  yaw bins : %d  [%.1f .. %.1f] deg" % (
        len(yaw_bins_rad),
        np.degrees(yaw_bins_rad[0]),
        np.degrees(yaw_bins_rad[-1]),
    ))
    print("  fixed_z  : %.3f m" % fixed_z)
    print("  shape    : %s" % str(reachable.shape))
    print("  map type : %s" % ("quality" if quality is not None else "binary"))
    if quality is not None:
        print("  quality  : min=%.3f  mean=%.3f  max=%.3f" % (
            float(np.min(quality)), float(np.mean(quality)), float(np.max(quality))
        ))
        if quality_threshold is not None:
            print("  threshold: %.3f" % quality_threshold)
    print("  reachable: %d / %d (%.2f%%)" % (reachable_count, total, pct))
    print()


def print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                  quality=None, quality_threshold=None, label="single arm"):
    _print_one_summary(label, x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                       quality, quality_threshold)


def print_dual_summary(right_map, left_map):
    _print_one_summary("right arm", *right_map[:5], quality=right_map[5],
                        quality_threshold=right_map[6])
    _print_one_summary("left arm",  *left_map[:5],  quality=left_map[5],
                        quality_threshold=left_map[6])

    if _maps_compatible(right_map, left_map):
        r_reach = right_map[3] > 0.5
        l_reach = left_map[3]  > 0.5
        union   = r_reach | l_reach
        total   = union.size
        u_count = int(np.sum(union))
        r_only  = int(np.sum(r_reach & ~l_reach))
        l_only  = int(np.sum(~r_reach & l_reach))
        both    = int(np.sum(r_reach & l_reach))
        print("Combined (right OR left)")
        print("------------------------")
        print("  union reachable : %d / %d (%.1f%%)" % (u_count, total, 100.0 * u_count / total))
        print("  right arm only  : %d (%.1f%%)" % (r_only, 100.0 * r_only / total))
        print("  left arm only   : %d (%.1f%%)" % (l_only, 100.0 * l_only / total))
        print("  both arms       : %d (%.1f%%)" % (both,   100.0 * both   / total))
        print()
    else:
        print("(Maps have different grid extents — combined stats not available)")
        print()


# ============================================================
# Internal data helpers
# ============================================================

def _display_data(reachable, quality):
    """Return (data_array, colorbar_label, vmin, vmax)."""
    if quality is not None:
        return quality, "quality score", 0.0, 1.0
    return reachable.astype(float), "reachable", 0.0, 1.0


def _yaw_slice_image(data, iyaw):
    # data: [Nx, Ny, Nyaw] → image rows/cols: [Ny, Nx]
    return data[:, :, iyaw].T


def _xy_extent(x_bins, y_bins):
    return [x_bins[0], x_bins[-1], y_bins[0], y_bins[-1]]


def _draw_robot_marker(ax):
    """Draw a small robot silhouette at the origin of the robot frame."""
    ax.plot(0, 0, marker="^", color="red", markersize=9, zorder=5, label="robot")


# ============================================================
# Single-arm plots  (backward-compatible)
# ============================================================

def plot_yaw_slice(x_bins, y_bins, yaw_bins_rad, reachable, yaw_deg,
                   quality=None, save_path=None, label=""):
    yaw_rad     = np.radians(yaw_deg)
    iyaw        = nearest_yaw_index(yaw_bins_rad, yaw_rad)
    yaw_used    = np.degrees(yaw_bins_rad[iyaw])
    data, cbar_label, vmin, vmax = _display_data(reachable, quality)
    img         = _yaw_slice_image(data, iyaw)
    extent      = _xy_extent(x_bins, y_bins)

    title_suffix = (" — %s" % label) if label else ""
    fig, ax = plt.subplots(figsize=(7, 5))
    im = ax.imshow(img, origin="lower", extent=extent, aspect="auto",
                   interpolation="nearest", vmin=vmin, vmax=vmax, cmap="viridis")
    _draw_robot_marker(ax)
    ax.set_xlabel("x in robot frame [m]")
    ax.set_ylabel("y in robot frame [m]")
    ax.set_title("Reachability  yaw=%.1f°%s" % (yaw_used, title_suffix))
    fig.colorbar(im, ax=ax, label=cbar_label)
    fig.tight_layout()
    _show_or_save(save_path)


def plot_xy_yaw_profile(x_bins, y_bins, yaw_bins_rad, reachable, x_query, y_query,
                         quality=None, save_path=None, label=""):
    ix      = nearest_index(x_bins, x_query)
    iy      = nearest_index(y_bins, y_query)
    x_used  = x_bins[ix]
    y_used  = y_bins[iy]

    if quality is not None:
        vals   = quality[ix, iy, :]
        ylabel = "quality score"
    else:
        vals   = reachable[ix, iy, :].astype(float)
        ylabel = "reachable"

    yaw_deg_arr = np.degrees(yaw_bins_rad)
    title_suffix = (" — %s" % label) if label else ""

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(yaw_deg_arr, vals, marker="o")
    ax.set_xlabel("yaw [deg]")
    ax.set_ylabel(ylabel)
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("Yaw profile  x=%.3f, y=%.3f%s" % (x_used, y_used, title_suffix))
    ax.grid(True)
    fig.tight_layout()
    _show_or_save(save_path)


# ============================================================
# Dual-arm plots
# ============================================================

def _add_imshow(ax, img, extent, vmin, vmax, cmap="viridis"):
    return ax.imshow(img, origin="lower", extent=extent, aspect="auto",
                     interpolation="nearest", vmin=vmin, vmax=vmax, cmap=cmap)


def plot_dual_yaw_slice(right_map, left_map, yaw_deg, save_path=None):
    """
    Three-panel figure: right arm | left arm | union.
    Works even when the two maps have different grids (union panel is skipped).
    """
    x_r, y_r, yaw_r, reach_r, _, qual_r, _ = right_map
    x_l, y_l, yaw_l, reach_l, _, qual_l, _ = left_map

    iyaw_r    = nearest_yaw_index(yaw_r, np.radians(yaw_deg))
    iyaw_l    = nearest_yaw_index(yaw_l, np.radians(yaw_deg))
    yaw_r_deg = np.degrees(yaw_r[iyaw_r])
    yaw_l_deg = np.degrees(yaw_l[iyaw_l])

    data_r, lbl_r, vmin, vmax = _display_data(reach_r, qual_r)
    data_l, lbl_l, _,    _    = _display_data(reach_l, qual_l)
    compatible = _maps_compatible(right_map, left_map)
    ncols = 3 if compatible else 2

    fig = plt.figure(figsize=(5.5 * ncols, 5))
    gs  = gridspec.GridSpec(1, ncols, wspace=0.35)

    # — Right arm —
    ax_r = fig.add_subplot(gs[0])
    im_r = _add_imshow(ax_r, _yaw_slice_image(data_r, iyaw_r),
                       _xy_extent(x_r, y_r), vmin, vmax)
    _draw_robot_marker(ax_r)
    ax_r.set_title("Right arm  (yaw=%.1f°)" % yaw_r_deg)
    ax_r.set_xlabel("x [m]"); ax_r.set_ylabel("y [m]")
    fig.colorbar(im_r, ax=ax_r, label=lbl_r)

    # — Left arm —
    ax_l = fig.add_subplot(gs[1])
    im_l = _add_imshow(ax_l, _yaw_slice_image(data_l, iyaw_l),
                       _xy_extent(x_l, y_l), vmin, vmax, cmap="plasma")
    _draw_robot_marker(ax_l)
    ax_l.set_title("Left arm  (yaw=%.1f°)" % yaw_l_deg)
    ax_l.set_xlabel("x [m]"); ax_l.set_ylabel("y [m]")
    fig.colorbar(im_l, ax=ax_l, label=lbl_l)

    # — Union —
    if compatible:
        union_slice = np.maximum(data_r[:, :, iyaw_r], data_l[:, :, iyaw_l])
        ax_u = fig.add_subplot(gs[2])
        # Use a cmap supported by the older Matplotlib in ROS (no "cividis" there).
        im_u = _add_imshow(ax_u, union_slice.T, _xy_extent(x_r, y_r), vmin, vmax, cmap="YlGnBu")
        _draw_robot_marker(ax_u)
        ax_u.set_title("Union (right OR left)  yaw=%.1f°" % yaw_r_deg)
        ax_u.set_xlabel("x [m]"); ax_u.set_ylabel("y [m]")
        fig.colorbar(im_u, ax=ax_u, label="max quality")

    fig.suptitle("Dual-arm reachability — yaw slice ≈ %.0f°" % yaw_deg, fontsize=13)
    fig.tight_layout()
    _show_or_save(save_path)


def plot_dual_diff_slice(right_map, left_map, yaw_deg, save_path=None):
    """
    Difference map: right_quality − left_quality at a given yaw.
    Requires compatible grids.
    """
    x_r, y_r, yaw_r, reach_r, _, qual_r, _ = right_map
    x_l, y_l, yaw_l, reach_l, _, qual_l, _ = left_map

    if not _maps_compatible(right_map, left_map):
        print("Warning: maps have different grids — cannot compute difference slice.")
        return

    data_r = qual_r if qual_r is not None else reach_r.astype(float)
    data_l = qual_l if qual_l is not None else reach_l.astype(float)

    iyaw    = nearest_yaw_index(yaw_r, np.radians(yaw_deg))
    yaw_deg_used = np.degrees(yaw_r[iyaw])
    diff    = data_r[:, :, iyaw] - data_l[:, :, iyaw]

    fig, ax = plt.subplots(figsize=(7, 5))
    vabs    = max(abs(float(diff.min())), abs(float(diff.max())), 0.01)
    im      = ax.imshow(diff.T, origin="lower", extent=_xy_extent(x_r, y_r),
                        aspect="auto", interpolation="nearest",
                        vmin=-vabs, vmax=vabs, cmap="RdBu_r")
    _draw_robot_marker(ax)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Right − Left  quality difference  (yaw=%.1f°)" % yaw_deg_used)
    fig.colorbar(im, ax=ax, label="quality difference")
    fig.tight_layout()
    _show_or_save(save_path)


def plot_dual_yaw_profile(right_map, left_map, x_query, y_query, save_path=None):
    """
    Overlay yaw profiles for right and left arm at a single (x, y) cell.
    """
    x_r, y_r, yaw_r, reach_r, _, qual_r, _ = right_map
    x_l, y_l, yaw_l, reach_l, _, qual_l, _ = left_map

    ix_r = nearest_index(x_r, x_query); iy_r = nearest_index(y_r, y_query)
    ix_l = nearest_index(x_l, x_query); iy_l = nearest_index(y_l, y_query)

    vals_r = qual_r[ix_r, iy_r, :] if qual_r is not None else reach_r[ix_r, iy_r, :].astype(float)
    vals_l = qual_l[ix_l, iy_l, :] if qual_l is not None else reach_l[ix_l, iy_l, :].astype(float)
    ylabel = "quality score" if (qual_r is not None or qual_l is not None) else "reachable"

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(np.degrees(yaw_r), vals_r, marker="o", color="steelblue",  label="right arm")
    ax.plot(np.degrees(yaw_l), vals_l, marker="s", color="darkorange", label="left arm",
            linestyle="--")
    ax.set_xlabel("grasp yaw relative to robot [deg]")
    ax.set_ylabel(ylabel)
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("Dual-arm yaw profile  x≈%.3f m, y≈%.3f m" % (x_r[ix_r], y_r[iy_r]))
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    _show_or_save(save_path)


def plot_dual_coverage_overview(right_map, left_map, save_path=None):
    """
    Max-over-yaw coverage map: for each (x,y) cell show the best quality
    achievable by each arm across all yaw bins, plus the union.
    Useful for a quick overview without committing to a specific yaw angle.
    """
    x_r, y_r, yaw_r, reach_r, _, qual_r, qthr_r = right_map
    x_l, y_l, yaw_l, reach_l, _, qual_l, qthr_l = left_map

    data_r = qual_r if qual_r is not None else reach_r.astype(float)
    data_l = qual_l if qual_l is not None else reach_l.astype(float)

    best_r = np.max(data_r, axis=2)    # [Nx_r, Ny_r]
    best_l = np.max(data_l, axis=2)    # [Nx_l, Ny_l]
    compatible = _maps_compatible(right_map, left_map)

    ncols = 3 if compatible else 2
    fig = plt.figure(figsize=(5.5 * ncols, 5))
    gs  = gridspec.GridSpec(1, ncols, wspace=0.35)

    ax_r = fig.add_subplot(gs[0])
    im_r = _add_imshow(ax_r, best_r.T, _xy_extent(x_r, y_r), 0.0, 1.0)
    _draw_robot_marker(ax_r)
    ax_r.set_title("Right arm — max over yaw")
    ax_r.set_xlabel("x [m]"); ax_r.set_ylabel("y [m]")
    fig.colorbar(im_r, ax=ax_r, label="best quality")

    ax_l = fig.add_subplot(gs[1])
    im_l = _add_imshow(ax_l, best_l.T, _xy_extent(x_l, y_l), 0.0, 1.0, cmap="plasma")
    _draw_robot_marker(ax_l)
    ax_l.set_title("Left arm — max over yaw")
    ax_l.set_xlabel("x [m]"); ax_l.set_ylabel("y [m]")
    fig.colorbar(im_l, ax=ax_l, label="best quality")

    if compatible:
        union = np.maximum(best_r, best_l)
        ax_u  = fig.add_subplot(gs[2])
        im_u  = _add_imshow(ax_u, union.T, _xy_extent(x_r, y_r), 0.0, 1.0, cmap="YlGnBu")
        _draw_robot_marker(ax_u)
        ax_u.set_title("Union — max over both arms & yaw")
        ax_u.set_xlabel("x [m]"); ax_u.set_ylabel("y [m]")
        fig.colorbar(im_u, ax=ax_u, label="best quality")

    fig.suptitle("Dual-arm coverage overview (max quality over all yaw bins)", fontsize=13)
    fig.tight_layout()
    _show_or_save(save_path)


# ============================================================
# Shared output helper
# ============================================================

def _show_or_save(save_path):
    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print("Saved figure to:", save_path)
        plt.close()
    else:
        plt.show()


# ============================================================
# CLI
# ============================================================

def _build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Visualise offline reachability maps (single-arm or dual-arm TIAGo++).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # --- Map inputs ---
    src = parser.add_argument_group("map inputs")
    src.add_argument("--map", default="",
                     help="Single-arm map path (backward-compatible)")
    src.add_argument("--map-right", default="",
                     help="Right-arm map path (TIAGo++ dual-arm mode)")
    src.add_argument("--map-left", default="",
                     help="Left-arm map path (TIAGo++ dual-arm mode)")

    # --- Actions ---
    act = parser.add_argument_group("actions")
    act.add_argument("--print-summary", action="store_true",
                     help="Print map statistics")
    act.add_argument("--yaw-slice-deg", type=float, default=None,
                     help="Show x-y slice at nearest yaw bin (deg)")
    act.add_argument("--diff-slice-deg", type=float, default=None,
                     help="[dual-arm] Right − Left quality difference slice (deg)")
    act.add_argument("--coverage-overview", action="store_true",
                     help="[dual-arm] Max-over-yaw coverage overview")
    act.add_argument("--xy-yaw-profile", action="store_true",
                     help="Plot yaw profile for a given (x, y) cell")
    act.add_argument("--x", type=float, default=0.50,
                     help="x query for yaw profile [m]")
    act.add_argument("--y", type=float, default=0.00,
                     help="y query for yaw profile [m]")

    # --- Save prefix ---
    save = parser.add_argument_group("output")
    save.add_argument("--save-prefix", default="",
                      help="If set, save all figures as <prefix>_<type>.png instead of showing")
    # Legacy per-figure save flags (single-arm, backward-compatible)
    save.add_argument("--yaw-slice-save",  default="",
                      help="[single-arm] Path to save yaw slice figure")
    save.add_argument("--xy-yaw-save",     default="",
                      help="[single-arm] Path to save yaw profile figure")

    return parser


def _save_path(prefix, suffix, legacy_path=""):
    """Resolve output path: prefer --save-prefix, fall back to legacy flag."""
    if prefix:
        return "%s_%s.png" % (prefix.rstrip("_"), suffix)
    return legacy_path if legacy_path else None


def _run_single_arm(args, m):
    x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold = m
    did = False

    if args.print_summary:
        print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                      quality=quality, quality_threshold=quality_threshold)

    if args.yaw_slice_deg is not None:
        plot_yaw_slice(
            x_bins, y_bins, yaw_bins_rad, reachable,
            yaw_deg=args.yaw_slice_deg,
            quality=quality,
            save_path=_save_path(args.save_prefix, "yaw_slice", args.yaw_slice_save),
        )
        did = True

    if args.xy_yaw_profile:
        plot_xy_yaw_profile(
            x_bins, y_bins, yaw_bins_rad, reachable,
            x_query=args.x, y_query=args.y,
            quality=quality,
            save_path=_save_path(args.save_prefix, "yaw_profile", args.xy_yaw_save),
        )
        did = True

    return did


def _run_dual_arm(args, right_map, left_map):
    did = False

    if args.print_summary:
        print_dual_summary(right_map, left_map)

    if args.yaw_slice_deg is not None:
        plot_dual_yaw_slice(
            right_map, left_map,
            yaw_deg=args.yaw_slice_deg,
            save_path=_save_path(args.save_prefix, "dual_yaw_slice"),
        )
        did = True

    if args.diff_slice_deg is not None:
        plot_dual_diff_slice(
            right_map, left_map,
            yaw_deg=args.diff_slice_deg,
            save_path=_save_path(args.save_prefix, "diff_slice"),
        )
        did = True

    if args.coverage_overview:
        plot_dual_coverage_overview(
            right_map, left_map,
            save_path=_save_path(args.save_prefix, "coverage_overview"),
        )
        did = True

    if args.xy_yaw_profile:
        plot_dual_yaw_profile(
            right_map, left_map,
            x_query=args.x, y_query=args.y,
            save_path=_save_path(args.save_prefix, "dual_yaw_profile"),
        )
        did = True

    return did


def _print_default_usage_hint(dual=False):
    print("\nNo plot option selected. Example usage:")
    if dual:
        print("  --yaw-slice-deg -90")
        print("  --diff-slice-deg -90")
        print("  --coverage-overview")
        print("  --xy-yaw-profile --x 0.6 --y 0.2")
    else:
        print("  --yaw-slice-deg 0")
        print("  --xy-yaw-profile --x 0.5 --y 0.0")


def main():
    parser = _build_arg_parser()
    args   = parser.parse_args()

    dual_mode = bool(args.map_right or args.map_left)
    single_mode = bool(args.map)

    if dual_mode and single_mode:
        parser.error("Specify either --map (single-arm) or --map-right/--map-left (dual-arm), not both.")
    if not dual_mode and not single_mode:
        parser.error("Specify at least one of: --map, --map-right + --map-left.")
    if dual_mode and not (args.map_right and args.map_left):
        parser.error("Dual-arm mode requires both --map-right and --map-left.")

    if single_mode:
        m   = load_map(args.map)
        did = _run_single_arm(args, m)
        if not args.print_summary and not did:
            x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, qt = m
            print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                          quality=quality, quality_threshold=qt)
            _print_default_usage_hint(dual=False)

    else:
        right_map = load_map(args.map_right)
        left_map  = load_map(args.map_left)
        did       = _run_dual_arm(args, right_map, left_map)
        if not args.print_summary and not did:
            print_dual_summary(right_map, left_map)
            _print_default_usage_hint(dual=True)


if __name__ == "__main__":
    main()
