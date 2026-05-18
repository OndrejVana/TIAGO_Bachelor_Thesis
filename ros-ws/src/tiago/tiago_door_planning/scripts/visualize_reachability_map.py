#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division, unicode_literals

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def nearest_index(values, q):
    values = np.asarray(values, dtype=float)
    return int(np.argmin(np.abs(values - float(q))))


def nearest_yaw_index(yaw_bins_rad, q_yaw):
    yaw_bins_rad = np.asarray(yaw_bins_rad, dtype=float)
    diffs = np.abs(((yaw_bins_rad - float(q_yaw) + np.pi) % (2.0 * np.pi)) - np.pi)
    return int(np.argmin(diffs))

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
    wrist_roll_rad_bins = (
        np.asarray(data["wrist_roll_rad_bins"], dtype=float)
        if "wrist_roll_rad_bins" in data else None
    )
    return x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, quality_threshold, wrist_roll_rad_bins


def load_map(path):
    """
    Load a reachability map NPZ.

    Returns an 8-tuple:
        (x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
         quality, quality_threshold, wrist_roll_rad_bins)

    reachable / quality may be 3-D [Nx, Ny, Nyaw] (old maps) or
    4-D [Nx, Ny, Nyaw, Nroll] (new maps).  wrist_roll_rad_bins is None for
    old 3-D maps.
    """
    _validate_map_exists(path)
    data = np.load(path, allow_pickle=False)
    _validate_map_keys(data, path)
    return _extract_map_arrays(data)


def _apply_roll_slice(m8, roll_rad=None):
    """
    Convert an 8-tuple (possibly 4-D arrays) into the 7-tuple used by all
    plot / summary helpers (3-D arrays).

    If the arrays are already 3-D the tuple is passed through unchanged.
    roll_rad=None  → use index 0 and print a hint.

    Returns (7-tuple, roll_rad_used).
    """
    x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, qt, roll_bins = m8

    if reachable.ndim == 3:
        return (x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, qt), None

    if reachable.ndim != 4:
        raise ValueError("reachable must be 3-D or 4-D, got shape %s" % str(reachable.shape))

    if roll_bins is None:
        raise ValueError("4-D reachable but wrist_roll_rad_bins missing in NPZ")

    if roll_rad is None:
        iroll = 0
        print(
            "Note: 4-D map — no --wrist-roll-rad specified, using roll=%.4f rad (index 0).\n"
            "      Available: %s rad"
            % (float(roll_bins[0]), ", ".join("%.4f" % v for v in roll_bins))
        )
    else:
        iroll = nearest_index(roll_bins, roll_rad)

    roll_used = float(roll_bins[iroll])
    reachable_3d = reachable[:, :, :, iroll]
    quality_3d   = quality[:, :, :, iroll] if quality is not None else None

    return (x_bins, y_bins, yaw_bins_rad, reachable_3d, fixed_z, quality_3d, qt), roll_used


def _maps_compatible(ma, mb):
    """Return True when two maps share identical bin grids."""
    return (
        np.allclose(ma[0], mb[0]) and   # x_bins
        np.allclose(ma[1], mb[1]) and   # y_bins
        np.allclose(ma[2], mb[2])        # yaw_bins_rad
    )

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

def _display_data(reachable, quality):
    """Return (data_array, colorbar_label, vmin, vmax)."""
    if quality is not None:
        return quality, "quality score", 0.0, 1.0
    return reachable.astype(float), "reachable", 0.0, 1.0


def _yaw_slice_image(data, iyaw):
    return data[:, :, iyaw].T


def _xy_extent(x_bins, y_bins):
    return [x_bins[0], x_bins[-1], y_bins[0], y_bins[-1]]


def _draw_robot_marker(ax):
    """Draw a small robot silhouette at the origin of the robot frame."""
    ax.plot(0, 0, marker="^", color="red", markersize=9, zorder=5, label="robot")

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


def plot_roll_profile(x_bins, y_bins, yaw_bins_rad, reachable_4d, wrist_roll_bins,
                      x_query, y_query, yaw_deg, quality_4d=None, save_path=None, label=""):
    """
    Plot quality vs wrist roll at a fixed (x, y, yaw) cell.
    Only meaningful for 4-D maps.
    """
    ix   = nearest_index(x_bins, x_query)
    iy   = nearest_index(y_bins, y_query)
    iyaw = nearest_yaw_index(yaw_bins_rad, np.radians(yaw_deg))
    x_used   = x_bins[ix]
    y_used   = y_bins[iy]
    yaw_used = np.degrees(yaw_bins_rad[iyaw])

    if quality_4d is not None:
        vals   = quality_4d[ix, iy, iyaw, :]
        ylabel = "quality score"
    else:
        vals   = reachable_4d[ix, iy, iyaw, :].astype(float)
        ylabel = "reachable"

    roll_deg = np.degrees(wrist_roll_bins)
    title_suffix = (" — %s" % label) if label else ""

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(range(len(roll_deg)), vals, tick_label=["%.1f°" % r for r in roll_deg])
    ax.set_xlabel("wrist roll")
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, 1.05)
    ax.set_title("Roll profile  x=%.3f, y=%.3f, yaw=%.1f°%s" % (
        x_used, y_used, yaw_used, title_suffix))
    ax.grid(True, axis="y")
    fig.tight_layout()
    _show_or_save(save_path)


def plot_dual_yaw_grid(right_map, left_map, yaw_degs, pitch_deg=None, save_path=None):
    """
    2-row x N-col grid: right arm (top row) and left arm (bottom row),
    one column per yaw value in yaw_degs.

    A single shared colorbar sits to the right of the grid.
    pitch_deg is used only for the figure suptitle (it is baked into the map).
    """
    x_r, y_r, yaw_r, reach_r, _, qual_r, _ = right_map
    x_l, y_l, yaw_l, reach_l, _, qual_l, _ = left_map

    data_r, cbar_lbl, vmin, vmax = _display_data(reach_r, qual_r)
    data_l, _,        _,    _    = _display_data(reach_l, qual_l)

    n = len(yaw_degs)
    fig_w = 3.6 * n + 0.9   # extra room for the colorbar
    fig_h = 6.5
    fig = plt.figure(figsize=(fig_w, fig_h))

    # Leave a narrow strip on the right for the colorbar
    gs = gridspec.GridSpec(2, n + 1,
                           width_ratios=[1.0] * n + [0.05],
                           wspace=0.30, hspace=0.35,
                           left=0.07, right=0.97,
                           top=0.88, bottom=0.08)

    im_last = None   # keep last imshow handle for the colorbar
    ext_r   = _xy_extent(x_r, y_r)
    ext_l   = _xy_extent(x_l, y_l)

    for col, yaw_deg in enumerate(yaw_degs):
        iyaw_r    = nearest_yaw_index(yaw_r, np.radians(yaw_deg))
        iyaw_l    = nearest_yaw_index(yaw_l, np.radians(yaw_deg))
        yaw_r_deg = np.degrees(yaw_r[iyaw_r])
        yaw_l_deg = np.degrees(yaw_l[iyaw_l])

        # -- right arm --
        ax_r = fig.add_subplot(gs[0, col])
        im = _add_imshow(ax_r, _yaw_slice_image(data_r, iyaw_r), ext_r, vmin, vmax)
        _draw_robot_marker(ax_r)
        ax_r.set_title("yaw = %.0f$^\\circ$" % yaw_r_deg, fontsize=11)
        if col == 0:
            ax_r.set_ylabel("Right arm\ny [m]", fontsize=10)
        else:
            ax_r.set_yticklabels([])
        ax_r.set_xticklabels([])
        im_last = im

        # -- left arm --
        ax_l = fig.add_subplot(gs[1, col])
        _add_imshow(ax_l, _yaw_slice_image(data_l, iyaw_l), ext_l, vmin, vmax,
                    cmap="plasma")
        _draw_robot_marker(ax_l)
        ax_l.set_xlabel("x [m]", fontsize=10)
        if col == 0:
            ax_l.set_ylabel("Left arm\ny [m]", fontsize=10)
        else:
            ax_l.set_yticklabels([])

    # shared colorbar in the extra column
    cax = fig.add_subplot(gs[:, n])
    fig.colorbar(im_last, cax=cax, label=cbar_lbl)

    pitch_str = ("  |  pitch = %.0f$^\\circ$" % pitch_deg) if pitch_deg is not None else ""
    fig.suptitle(
        "Dual-arm reachability — grasp yaw sweep%s" % pitch_str,
        fontsize=13,
    )

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

    def _style(ax, title):
        ax.set_title(title, fontsize=15)
        ax.set_xlabel("x [m]", fontsize=14)
        ax.set_ylabel("y [m]", fontsize=14)
        ax.tick_params(labelsize=13)

    def _cbar(fig, im, ax, label):
        cb = fig.colorbar(im, ax=ax)
        cb.set_label(label, fontsize=14)
        cb.ax.tick_params(labelsize=13)

    if compatible:
        # Row 0: right + left side by side; Row 1: union centred
        fig = plt.figure(figsize=(13, 12))
        gs  = gridspec.GridSpec(2, 4, wspace=0.55, hspace=0.45)

        ax_r = fig.add_subplot(gs[0, 0:2])
        ax_l = fig.add_subplot(gs[0, 2:4])
        ax_u = fig.add_subplot(gs[1, 1:3])

        im_r = _add_imshow(ax_r, best_r.T, _xy_extent(x_r, y_r), 0.0, 1.0)
        _draw_robot_marker(ax_r)
        _style(ax_r, "Right arm - max over yaw")
        _cbar(fig, im_r, ax_r, "best quality")

        im_l = _add_imshow(ax_l, best_l.T, _xy_extent(x_l, y_l), 0.0, 1.0, cmap="plasma")
        _draw_robot_marker(ax_l)
        _style(ax_l, "Left arm - max over yaw")
        _cbar(fig, im_l, ax_l, "best quality")

        union = np.maximum(best_r, best_l)
        im_u  = _add_imshow(ax_u, union.T, _xy_extent(x_r, y_r), 0.0, 1.0, cmap="YlGnBu")
        _draw_robot_marker(ax_u)
        _style(ax_u, "Union - best of either arm")
        _cbar(fig, im_u, ax_u, "best quality")
    else:
        fig = plt.figure(figsize=(13, 6))
        gs  = gridspec.GridSpec(1, 2, wspace=0.45)

        ax_r = fig.add_subplot(gs[0])
        ax_l = fig.add_subplot(gs[1])

        im_r = _add_imshow(ax_r, best_r.T, _xy_extent(x_r, y_r), 0.0, 1.0)
        _draw_robot_marker(ax_r)
        _style(ax_r, "Right arm - max over yaw")
        _cbar(fig, im_r, ax_r, "best quality")

        im_l = _add_imshow(ax_l, best_l.T, _xy_extent(x_l, y_l), 0.0, 1.0, cmap="plasma")
        _draw_robot_marker(ax_l)
        _style(ax_l, "Left arm - max over yaw")
        _cbar(fig, im_l, ax_l, "best quality")

    fig.suptitle("Dual-arm reachability - max quality over all grasp yaw angles",
                 fontsize=16)
    fig.tight_layout()
    _show_or_save(save_path)

def _show_or_save(save_path):
    if save_path:
        dirpath = os.path.dirname(save_path)
        if dirpath and not os.path.isdir(dirpath):
            os.makedirs(dirpath)
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print("Saved figure to:", save_path)
        plt.close()
    else:
        plt.show()

def _default_map_path(arm):
    """Return the package-relative map path, or '' if rospkg is not available."""
    try:
        import rospkg
        pkg = rospkg.RosPack().get_path('tiago_door_planning')
        return os.path.join(pkg, 'maps', 'reachability_map_4D_new_%s.npz' % arm)
    except Exception:
        return ''


def _build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Visualise offline reachability maps (single-arm or dual-arm TIAGo++).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # --- Map inputs ---
    src = parser.add_argument_group("map inputs")
    src.add_argument("--map", default="",
                     help="Single-arm map path (backward-compatible)")
    src.add_argument("--map-right", default=_default_map_path('right'),
                     help="Right-arm map path (TIAGo++ dual-arm mode). "
                          "Defaults to maps/reachability_map_4D_new_right.npz inside the package.")
    src.add_argument("--map-left", default=_default_map_path('left'),
                     help="Left-arm map path (TIAGo++ dual-arm mode). "
                          "Defaults to maps/reachability_map_4D_new_left.npz inside the package.")

    # --- 4-D roll selection ---
    # -3.14159 is the runtime value from planner.yaml (reachability_wrist_roll_rad)
    src.add_argument("--wrist-roll-rad", type=float, default=-3.14159,
                     help="Wrist roll slice for 4-D maps (rad). "
                          "Must match one of wrist_roll_rad_bins in the NPZ "
                          "(available: 0.0, -1.5708, -3.14159). "
                          "Matches planner/reachability_wrist_roll_rad in planner.yaml.")

    # --- Actions ---
    act = parser.add_argument_group("actions")
    act.add_argument("--print-summary", action="store_true",
                     help="Print map statistics")
    act.add_argument("--yaw-slice-deg", type=float, default=None,
                     help="Show x-y slice at nearest yaw bin (deg)")
    act.add_argument("--diff-slice-deg", type=float, default=None,
                     help="[dual-arm] Right − Left quality difference slice (deg)")
    act.add_argument("--yaw-grid", action="store_true",
                     help="[dual-arm] 2-row x N-col grid: right/left arm at multiple yaw slices")
    act.add_argument("--yaw-grid-degs", type=float, nargs="+", default=[0, 20, 40, 60],
                     metavar="DEG",
                     help="Yaw values (deg) for --yaw-grid columns (default: 0 20 40 60)")
    act.add_argument("--pitch-deg", type=float, default=None,
                     help="Grasp pitch baked into the map, used only for the figure title (deg)")
    act.add_argument("--coverage-overview", action="store_true",
                     help="[dual-arm] Max-over-yaw coverage overview")
    act.add_argument("--xy-yaw-profile", action="store_true",
                     help="Plot yaw profile for a given (x, y) cell")
    act.add_argument("--roll-profile", action="store_true",
                     help="[4-D maps] Plot quality vs wrist roll at a fixed (x, y, yaw) cell")
    act.add_argument("--x", type=float, default=0.50,
                     help="x query for yaw/roll profile [m]")
    act.add_argument("--y", type=float, default=0.00,
                     help="y query for yaw/roll profile [m]")

    # --- Save prefix ---
    save = parser.add_argument_group("output")
    save.add_argument("--save-prefix", default=_default_save_prefix(),
                      help="Save all figures as <prefix>_<type>.png. "
                           "Defaults to <pkg>/reachability_<type>.png.")
    # Legacy per-figure save flags (single-arm, backward-compatible)
    save.add_argument("--yaw-slice-save",  default="",
                      help="[single-arm] Path to save yaw slice figure")
    save.add_argument("--xy-yaw-save",     default="",
                      help="[single-arm] Path to save yaw profile figure")

    return parser


def _default_save_prefix():
    try:
        import rospkg
        pkg = rospkg.RosPack().get_path('tiago_door_planning')
        return os.path.join(pkg, 'reachability')
    except Exception:
        return ''


def _save_path(prefix, suffix, legacy_path=""):
    """Resolve output path: prefer --save-prefix, fall back to legacy flag."""
    if prefix:
        return "%s_%s.png" % (prefix.rstrip("_"), suffix)
    return legacy_path if legacy_path else None


def _run_single_arm(args, m, m8_raw=None):
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

    if args.roll_profile and m8_raw is not None:
        x_b, y_b, yaw_b, reach4, _, qual4, _, roll_bins = m8_raw
        if reach4.ndim == 4 and roll_bins is not None:
            yaw_deg = args.yaw_slice_deg if args.yaw_slice_deg is not None else 0.0
            plot_roll_profile(
                x_b, y_b, yaw_b, reach4, roll_bins,
                x_query=args.x, y_query=args.y, yaw_deg=yaw_deg,
                quality_4d=qual4,
                save_path=_save_path(args.save_prefix, "roll_profile"),
            )
            did = True
        else:
            print("Note: --roll-profile requires a 4-D map with wrist_roll_rad_bins.")

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

    if args.yaw_grid:
        plot_dual_yaw_grid(
            right_map, left_map,
            yaw_degs=args.yaw_grid_degs,
            pitch_deg=args.pitch_deg,
            save_path=_save_path(args.save_prefix, "yaw_grid"),
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


def _print_default_usage_hint(dual=False, has_roll=False):
    print("\nNo plot option selected. Example usage:")
    if dual:
        print("  --yaw-slice-deg -90")
        print("  --diff-slice-deg -90")
        print("  --coverage-overview")
        print("  --xy-yaw-profile --x 0.6 --y 0.2")
    else:
        print("  --yaw-slice-deg 0")
        print("  --xy-yaw-profile --x 0.5 --y 0.0")
    if has_roll:
        print("  --roll-profile --x 0.5 --y 0.0 --yaw-slice-deg 0")
        print("  --wrist-roll-rad -3.14159  (select roll slice for other plots)")


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
        m8  = load_map(args.map)
        m7, roll_used = _apply_roll_slice(m8, args.wrist_roll_rad)
        if roll_used is not None:
            print("Using wrist roll slice: %.4f rad (%.1f°)" % (roll_used, np.degrees(roll_used)))
        did = _run_single_arm(args, m7, m8_raw=m8)
        has_roll = m8[7] is not None
        if not args.print_summary and not did:
            x_bins, y_bins, yaw_bins_rad, reachable, fixed_z, quality, qt = m7
            print_summary(x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                          quality=quality, quality_threshold=qt)
            _print_default_usage_hint(dual=False, has_roll=has_roll)

    else:
        m8_r = load_map(args.map_right)
        m8_l = load_map(args.map_left)
        right_map, roll_r = _apply_roll_slice(m8_r, args.wrist_roll_rad)
        left_map,  roll_l = _apply_roll_slice(m8_l, args.wrist_roll_rad)
        if roll_r is not None:
            print("Right arm: using wrist roll slice %.4f rad (%.1f°)" % (roll_r, np.degrees(roll_r)))
        if roll_l is not None:
            print("Left arm:  using wrist roll slice %.4f rad (%.1f°)" % (roll_l, np.degrees(roll_l)))
        has_roll = m8_r[7] is not None or m8_l[7] is not None
        did = _run_dual_arm(args, right_map, left_map)
        if not args.print_summary and not did:
            print_dual_summary(right_map, left_map)
            _print_default_usage_hint(dual=True, has_roll=has_roll)


if __name__ == "__main__":
    main()
