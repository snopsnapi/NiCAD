"""Golden-data generator for the FitSpline C++ port.

Reimplements rt_spline.interpolate() from the Nikaishi ReverseTools workbench
(C:\\Users\\SnopS\\FreeCADAddons\\ReverseTools\\rt_spline.py) without FreeCAD:
instead of building a Part.BSplineCurve it returns the raw spline data
(degree, full knot vector, poles, parameter of each point). The C++ test
(tests/test_fit_spline.cpp) must reproduce these numbers to 1e-9.

Writes tests/data/spline_golden.json.
"""

import json
import os

import numpy as np
from scipy.interpolate import BSpline


def interpolate(points, constraints, degree):
    """B-spline through ``points`` (n,3).

    ``constraints`` maps a point index to (d1, d2): first and second derivative
    with respect to the curve parameter in [0, 1]; d2 may be None.
    Returns dict(degree, knots, poles, params).
    """
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)]) / seg.sum()

    sites, kinds, rhs = [], [], []
    multiplicity = 1
    for k in range(len(pts)):
        sites.append(t[k]); kinds.append(0); rhs.append(pts[k])
        d1, d2 = constraints.get(k, (None, None))
        if d1 is not None:
            sites.append(t[k]); kinds.append(1); rhs.append(d1)
            multiplicity = max(multiplicity, 2)
            if d2 is not None:
                sites.append(t[k]); kinds.append(2); rhs.append(d2)
                multiplicity = 3
    n = len(sites)
    p = min(max(int(degree), multiplicity), n - 1)
    if len(pts) == 2:
        p = n - 1  # a two-point blend stays a single span

    tau = np.array(sites)
    inner = [tau[j:j + p].mean() for j in range(1, n - p)]
    knots = np.concatenate([np.zeros(p + 1), inner, np.ones(p + 1)])

    basis = BSpline(knots, np.eye(n), p)
    derivs = [basis, basis.derivative(1) if p >= 1 else None,
              basis.derivative(2) if p >= 2 else None]
    a = np.empty((n, n))
    for row, (site, kind) in enumerate(zip(sites, kinds)):
        a[row] = derivs[kind](site)
    b = np.array(rhs)
    try:
        poles = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        poles = np.linalg.lstsq(a, b, rcond=None)[0]

    return dict(degree=int(p), knots=[float(k) for k in knots],
                poles=[[float(c) for c in q] for q in poles],
                params=[float(v) for v in t])


def main():
    v = lambda *c: [float(x) for x in c]

    cases = [
        ("two_points", 3,
         [v(0.13, -2.71, 4.55), v(9.87, 3.19, -1.42)],
         {}),
        ("three_points_deg3", 3,
         [v(-3.17, 8.42, 1.05), v(5.93, -1.64, 6.28), v(12.3, 7.11, -4.71)],
         {}),
        ("five_3d", 3,
         [v(1.23, -4.56, 7.89), v(-8.12, 3.45, 0.67), v(6.78, 9.01, -2.34),
          v(-0.55, -7.77, 5.43), v(11.11, 2.22, -9.99)],
         {}),
        ("seven_deg5", 5,
         [v(0.31, 1.27, -0.84), v(4.72, -3.19, 2.95), v(-6.55, 8.08, 1.31),
          v(9.94, 5.55, -7.27), v(-2.46, -9.13, 4.48), v(7.35, 0.93, -5.62),
          v(-11.02, 6.66, 3.21)],
         {}),
        ("planar_six", 3,
         [v(-7.4, 3.6, 0.0), v(1.9, 12.35, 0.0), v(10.6, 4.75, 0.0),
          v(8.15, -6.9, 0.0), v(-2.3, -11.55, 0.0), v(-12.85, -2.05, 0.0)],
         {}),
        ("end_tangents", 3,
         [v(2.47, -5.31, 0.88), v(7.62, 3.94, -6.15), v(-3.86, 9.27, 2.73),
          v(-9.48, -1.42, -8.06), v(5.15, 6.38, 4.91)],
         {0: (v(12.5, 3.3, -1.1), None),
          4: (v(9.2, -7.6, 2.8), None)}),
        ("g2_middle", 3,
         [v(-1.37, 4.59, -8.24), v(6.81, -2.08, 3.36), v(-4.92, 11.73, 0.47),
          v(13.28, -9.86, 6.02), v(-8.44, 1.95, -3.68), v(3.07, 7.64, 9.35)],
         {2: (v(-6.4, 2.9, 5.5), v(0.73, -1.62, 0.28))}),
        ("g1_all_ends_deg5", 5,
         [v(0.77, -3.33, 5.91), v(-10.24, 6.18, -1.76), v(8.93, 12.46, 3.84),
          v(5.62, -8.71, -7.29), v(-6.08, 2.95, 10.13), v(14.39, -4.62, 0.59)],
         {0: (v(15.7, -2.4, 6.3), None),
          5: (v(-11.8, 4.9, -3.7), None)}),
    ]

    out = []
    for name, degree, points, constraints in cases:
        res = interpolate(points, constraints, degree)
        out.append(dict(
            name=name,
            degree_in=degree,
            points=points,
            constraints=[dict(index=int(k), d1=c[0], d2=c[1])
                         for k, c in sorted(constraints.items())],
            out=res,
        ))

    here = os.path.dirname(os.path.abspath(__file__))
    dest = os.path.normpath(os.path.join(here, "..", "..", "tests", "data",
                                         "spline_golden.json"))
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, "w") as f:
        json.dump(out, f)
    print("wrote", dest, os.path.getsize(dest), "bytes,", len(out), "cases")


if __name__ == "__main__":
    main()
