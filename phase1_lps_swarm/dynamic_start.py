#!/usr/bin/env python3
"""
dynamic_start.py — B-419 anchor-aware dynamic placement helper
==============================================================
Kaustubh Sharma | IIIT-Delhi (Prof. Sanjit Kaul) | Lab B-419, IRAS Hub

Shared by all single-drone trajectory scripts so that a shape is built around
WHEREVER the drone is placed, and always stays inside the LPS anchor hull —
no hardcoded absolute centres, no corners landing on an anchor.

Safe zone is derived from anchors_updated_positions.yaml (8 anchors, TDoA2):
  anchors span X[0.0, 5.04], Y[-0.8, 7.4]; the four LOW anchors (z=0.30)
  bracket Y[1.0, 5.8], which is the best-covered floor band. We keep ~0.6 m
  off the X walls and stay inside the low-anchor Y band. Tighten if you want a
  smaller envelope — every script imports these same numbers.
"""
import math

SAFE_X_MIN, SAFE_X_MAX = 0.6, 4.4
SAFE_Y_MIN, SAFE_Y_MAX = 1.2, 5.6

# The point shapes curve toward / centre on = middle of the safe zone.
BIAS_CENTER_X = (SAFE_X_MIN + SAFE_X_MAX) / 2.0    # 2.5
BIAS_CENTER_Y = (SAFE_Y_MIN + SAFE_Y_MAX) / 2.0    # 3.4

MARGIN = 0.05


def clamp_xy(x, y, m=MARGIN):
    """Hard-clamp a waypoint into the safe zone (belt-and-suspenders)."""
    return (max(SAFE_X_MIN + m, min(SAFE_X_MAX - m, x)),
            max(SAFE_Y_MIN + m, min(SAFE_Y_MAX - m, y)))


def in_safe_zone(x, y, m=0.1):
    return (SAFE_X_MIN + m <= x <= SAFE_X_MAX - m and
            SAFE_Y_MIN + m <= y <= SAFE_Y_MAX - m)


def fit_center(spawn_x, spawn_y, reach_x, reach_y, m=MARGIN):
    """Pick a shape centre as close to the drone's spawn as possible such that
    a shape extending +/- reach_x in X and +/- reach_y in Y fits fully inside
    the safe zone. If the shape is wider than the zone on an axis, centre it on
    that axis. This is what makes every shape 'placeable anywhere' while never
    leaving the anchor hull."""
    lo_x, hi_x = SAFE_X_MIN + m + reach_x, SAFE_X_MAX - m - reach_x
    lo_y, hi_y = SAFE_Y_MIN + m + reach_y, SAFE_Y_MAX - m - reach_y
    cx = BIAS_CENTER_X if lo_x > hi_x else max(lo_x, min(hi_x, spawn_x))
    cy = BIAS_CENTER_Y if lo_y > hi_y else max(lo_y, min(hi_y, spawn_y))
    return cx, cy


def inward_unit(x, y):
    """Unit vector from (x, y) toward the anchor-volume centre — used by
    directional shapes (e.g. the parabola) so they travel INTO the hull rather
    than toward a wall. Falls back to +X if already at the centre."""
    dx, dy = BIAS_CENTER_X - x, BIAS_CENTER_Y - y
    n = math.hypot(dx, dy)
    return (1.0, 0.0) if n < 0.05 else (dx / n, dy / n)
