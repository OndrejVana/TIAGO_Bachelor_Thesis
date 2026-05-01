# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np


class DoorGeom(object):
    def __init__(self, width_m, thickness_m, open_angle_rad):
        self.width_m = width_m
        self.thickness_m = thickness_m
        self.open_angle_rad = open_angle_rad


def _as_xy_array(points):
    """
    Convert sequence of 2D points to NumPy array of shape (N, 2).
    """
    return np.array(points, dtype=float)


def _polygon_edges(poly_array):
    """
    Return edge start and end points for a closed polygon.
    """
    p1 = poly_array
    p2 = np.roll(poly_array, -1, axis=0)
    return p1, p2


def _clamp_grid_index(value, max_value):
    """
    Clamp grid index into valid inclusive range [0, max_value - 1].
    """
    return max(0, min(max_value - 1, value))


def _world_to_grid_index(x, y, ox, oy, res):
    """
    Convert world coordinates to grid indices.
    """
    ix = int((x - ox) / res)
    iy = int((y - oy) / res)
    return ix, iy


def _compute_search_window(cx, cy, radius, ox, oy, res):
    """
    Compute conservative grid search window around a circle.
    """
    mnx = cx - radius
    mxx = cx + radius
    mny = cy - radius
    mxy = cy + radius

    ix0 = int((mnx - ox) / res) - 1
    iy0 = int((mny - oy) / res) - 1
    ix1 = int((mxx - ox) / res) + 1
    iy1 = int((mxy - oy) / res) + 1

    return ix0, iy0, ix1, iy1


def _circle_contains_any_points(cx, cy, radius, px, py):
    """
    Check if any points (px, py) lie inside or on the circle.
    """
    dx = px - float(cx)
    dy = py - float(cy)
    rr = float(radius) * float(radius)
    return bool(np.any(dx * dx + dy * dy <= rr))


def door_polygon(hinge_xy, hinge_yaw, angle, geom, opening_sign=1.0):
    """
    Rectangle anchored at hinge, extending width along door direction.
    Door direction yaw = hinge_yaw + angle.
    Thickness is perpendicular.
    """
    hinge = np.array(hinge_xy, dtype=float)
    yaw = hinge_yaw + float(opening_sign) * angle

    u = np.array([np.cos(yaw), np.sin(yaw)])
    v = np.array([-u[1], u[0]])

    w = geom.width_m
    half_t = geom.thickness_m * 0.5

    corners = np.array([
        hinge + v * half_t,
        hinge - v * half_t,
        hinge + u * w - v * half_t,
        hinge + u * w + v * half_t
    ])

    return [tuple(corner) for corner in corners]


def point_in_poly(pt, poly):
    """Ray-casting algorithm (checks for two crossings with horizontal ray). Returns True if point is inside polygon."""
    poly_array = _as_xy_array(poly)
    x, y = pt

    p1, p2 = _polygon_edges(poly_array)

    y1 = p1[:, 1]
    y2 = p2[:, 1]
    x1 = p1[:, 0]
    x2 = p2[:, 0]

    crosses_y = (y1 > y) != (y2 > y)
    xinters = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1

    crossing_count_is_odd = np.sum((x < xinters) & crosses_y) % 2 == 1
    return bool(crossing_count_is_odd)


def poly_aabb(poly):
    """Compute axis-aligned bounding box using numpy."""
    poly_array = _as_xy_array(poly)
    min_coords = np.min(poly_array, axis=0)
    max_coords = np.max(poly_array, axis=0)

    return (
        float(min_coords[0]),
        float(min_coords[1]),
        float(max_coords[0]),
        float(max_coords[1])
    )


def _closest_points_on_polygon_edges(cx, cy, poly):
    """
    Compute closest points on each polygon edge to the circle center.
    """
    poly_array = _as_xy_array(poly)
    center = np.array([cx, cy], dtype=float)

    p1, p2 = _polygon_edges(poly_array)

    edges = p2 - p1
    to_center = center - p1

    edge_lengths_sq = np.sum(edges * edges, axis=1) + 1e-12
    t = np.sum(to_center * edges, axis=1) / edge_lengths_sq
    t = np.clip(t, 0.0, 1.0)

    closest_points = p1 + t[:, np.newaxis] * edges
    return closest_points


def circle_intersects_poly(cx, cy, r, poly):
    """Check circle-polygon intersection using vectorized numpy operations."""
    if point_in_poly((cx, cy), poly):
        return True

    center = np.array([cx, cy], dtype=float)
    closest_points = _closest_points_on_polygon_edges(cx, cy, poly)

    distances_sq = np.sum((center - closest_points) ** 2, axis=1)
    return bool(np.any(distances_sq <= r * r))



def extract_occ_info(occ):
    """
    Extract commonly used OccupancyGrid info fields: (res, ox, oy, width, height).
    """
    res = occ.info.resolution
    ox = occ.info.origin.position.x
    oy = occ.info.origin.position.y
    w = occ.info.width
    h = occ.info.height
    return res, ox, oy, w, h


def _occupied_cell_centers_in_window(occ, occ_thresh, ix0, iy0, ix1, iy1, ox, oy, res, w):
    """
    Return world coordinates of occupied cell centers within a grid window.
    """
    data = np.array(occ.data, dtype=np.int16)

    iy_range = np.arange(iy0, iy1 + 1)
    ix_range = np.arange(ix0, ix1 + 1)
    ix_grid, iy_grid = np.meshgrid(ix_range, iy_range)

    ix_flat = ix_grid.ravel()
    iy_flat = iy_grid.ravel()

    indices = iy_flat * w + ix_flat
    occ_mask = data[indices] >= occ_thresh

    if not np.any(occ_mask):
        return None, None

    wx = ox + (ix_flat[occ_mask] + 0.5) * res
    wy = oy + (iy_flat[occ_mask] + 0.5) * res
    return wx, wy


def circle_hits_occupancy(cx, cy, radius, occ, occ_thresh):
    """
    Check whether a circular robot footprint intersects occupied cells
    in an OccupancyGrid.

    Args:
        cx, cy: circle center in world/map coordinates
        radius: robot footprint radius
        occ: nav_msgs/OccupancyGrid
        occ_thresh: occupancy threshold

    Returns:
        True if any occupied cell center lies inside the circle.
    """
    if occ is None:
        return False

    res, ox, oy, w, h = extract_occ_info(occ)

    ix0, iy0, ix1, iy1 = _compute_search_window(cx, cy, radius, ox, oy, res)

    ix0 = _clamp_grid_index(ix0, w)
    ix1 = _clamp_grid_index(ix1, w)
    iy0 = _clamp_grid_index(iy0, h)
    iy1 = _clamp_grid_index(iy1, h)

    if ix1 < ix0 or iy1 < iy0:
        return False

    wx, wy = _occupied_cell_centers_in_window(
        occ, occ_thresh, ix0, iy0, ix1, iy1, ox, oy, res, w
    )

    if wx is None:
        return False

    return _circle_contains_any_points(cx, cy, radius, wx, wy)