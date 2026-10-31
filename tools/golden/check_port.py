"""Self-check for the FitSpline C++ port.

Re-implements the C++ algorithm 1:1 in Python: own FindSpan (A2.1), own
DersBasisFuns (A2.3) and own Gaussian elimination with partial pivoting -
no scipy basis evaluation, no numpy.linalg.solve. Compares the result with
tests/data/spline_golden.json (produced by spline_golden.py via scipy) at
1e-9, so a logic error in the hand-written basis/linear solver surfaces
locally instead of on CI.
"""

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GOLDEN = os.path.normpath(os.path.join(HERE, "..", "..", "tests", "data",
                                       "spline_golden.json"))


def find_span(p, u, knots):
    """A2.1. knots: full vector, len = poleCount + p + 1 (0-based)."""
    n = len(knots) - p - 2  # last non-zero span index
    if u >= knots[-1]:
        return n  # last non-zero interval (scipy convention)
    if u <= knots[0]:
        return p
    low, high = p, n + 1
    mid = (low + high) // 2
    while u < knots[mid] or u >= knots[mid + 1]:
        if u < knots[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def ders_basis_funs(span, u, p, nd, knots):
    """A2.3. Returns ders[k][j], k = 0..nd, j = 0..p."""
    ders = [[0.0] * (p + 1) for _ in range(nd + 1)]
    ndu = [[0.0] * (p + 1) for _ in range(p + 1)]
    left = [0.0] * (p + 1)
    right = [0.0] * (p + 1)
    ndu[0][0] = 1.0
    for j in range(1, p + 1):
        left[j] = u - knots[span + 1 - j]
        right[j] = knots[span + j] - u
        saved = 0.0
        for r in range(j):
            ndu[j][r] = right[r + 1] + left[j - r]
            temp = ndu[r][j - 1] / ndu[j][r]
            ndu[r][j] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        ndu[j][j] = saved
    for j in range(p + 1):
        ders[0][j] = ndu[j][p]

    a = [[0.0] * (p + 1) for _ in range(2)]
    for r in range(p + 1):
        s1, s2 = 0, 1
        a[0][0] = 1.0
        for k in range(1, nd + 1):
            d = 0.0
            rk = r - k
            pk = p - k
            if r >= k:
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk]
                d = a[s2][0] * ndu[rk][pk]
            j1 = 1 if rk >= -1 else -rk
            j2 = k - 1 if r - 1 <= pk else p - r
            for j in range(j1, j2 + 1):
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j]
                d += a[s2][j] * ndu[rk + j][pk]
            if r <= pk:
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r]
                d += a[s2][k] * ndu[r][pk]
            ders[k][r] = d
            s1, s2 = s2, s1
    r = p
    for k in range(1, nd + 1):
        for j in range(p + 1):
            ders[k][j] *= r
        r *= p - k
    return ders


def gauss_solve(a, b):
    """Gaussian elimination with partial pivoting. a: n x n, b: n x m.
    Returns (x, ok); ok=False when the matrix is singular."""
    n = len(a)
    m = len(b[0])
    aug = [list(a[i]) + list(b[i]) for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[piv][col]) < 1e-14:
            return None, False
        aug[col], aug[piv] = aug[piv], aug[col]
        d = aug[col][col]
        for r in range(col + 1, n):
            f = aug[r][col] / d
            if f == 0.0:
                continue
            for c in range(col, n + m):
                aug[r][c] -= f * aug[col][c]
    x = [[0.0] * m for _ in range(n)]
    for i in range(n - 1, -1, -1):
        for j in range(m):
            s = aug[i][n + j] - sum(aug[i][c] * x[c][j] for c in range(i + 1, n))
            x[i][j] = s / aug[i][i]
    return x, True


def interpolate(points, constraints, degree):
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
        p = n - 1

    tau = np.array(sites)
    inner = [float(tau[j:j + p].mean()) for j in range(1, n - p)]
    knots = [0.0] * (p + 1) + inner + [1.0] * (p + 1)

    a = np.zeros((n, n))
    for row, (site, kind) in enumerate(zip(sites, kinds)):
        span = find_span(p, site, knots)
        ders = ders_basis_funs(span, site, p, kind, knots)
        for j in range(p + 1):
            a[row][span - p + j] = ders[kind][j]
    x, ok = gauss_solve(a, [[float(c) for c in r] for r in rhs])
    if not ok:
        return None
    return dict(degree=p, knots=knots, poles=x, params=[float(v) for v in t])


def evaluate(curve, t, deriv=0):
    p = curve["degree"]
    knots = curve["knots"]
    poles = curve["poles"]
    u = min(max(t, knots[0]), knots[-1])
    span = find_span(p, u, knots)
    ders = ders_basis_funs(span, u, p, deriv, knots)
    out = [0.0, 0.0, 0.0]
    for j in range(p + 1):
        for c in range(3):
            out[c] += ders[deriv][j] * poles[span - p + j][c]
    return out


def close(a, b, tol):
    return abs(a - b) <= tol * (1.0 + abs(b))


def main():
    with open(GOLDEN) as f:
        cases = json.load(f)
    worst = 0.0
    for case in cases:
        cons = {c["index"]: (c["d1"], c["d2"]) for c in case["constraints"]}
        got = interpolate(case["points"], cons, case["degree_in"])
        assert got is not None, case["name"] + ": singular"
        ref = case["out"]
        assert got["degree"] == ref["degree"], (case["name"], "degree")
        for i, (g, r) in enumerate(zip(got["knots"], ref["knots"])):
            assert close(g, r, 1e-9), (case["name"], "knot", i, g, r)
            worst = max(worst, abs(g - r) / (1 + abs(r)))
        for i, (g, r) in enumerate(zip(got["params"], ref["params"])):
            assert close(g, r, 1e-9), (case["name"], "param", i, g, r)
            worst = max(worst, abs(g - r) / (1 + abs(r)))
        assert len(got["poles"]) == len(ref["poles"])
        for i, (g, r) in enumerate(zip(got["poles"], ref["poles"])):
            for c in range(3):
                assert close(g[c], r[c], 1e-9), (case["name"], "pole", i, c, g, r)
                worst = max(worst, abs(g[c] - r[c]) / (1 + abs(r[c])))
        # spot-check: curve passes through the points, d1/d2 constraints hold
        for i, pt in enumerate(case["points"]):
            ev = evaluate(got, got["params"][i], 0)
            for c in range(3):
                assert abs(ev[c] - pt[c]) < 1e-9, (case["name"], "value", i)
        for c in case["constraints"]:
            ev1 = evaluate(got, got["params"][c["index"]], 1)
            for k in range(3):
                assert abs(ev1[k] - c["d1"][k]) < 1e-7, (case["name"], "d1")
            if c["d2"] is not None:
                ev2 = evaluate(got, got["params"][c["index"]], 2)
                for k in range(3):
                    assert abs(ev2[k] - c["d2"][k]) < 1e-6, (case["name"], "d2")
        print("ok", case["name"], "degree", got["degree"])
    print("ALL %d CASES MATCH GOLDEN (worst rel err %.2e)" % (len(cases), worst))


if __name__ == "__main__":
    main()
