#include "Form.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <sstream>

namespace materializr {
namespace form {

namespace {

gp_Vec toVec(const gp_Pnt& p) { return gp_Vec(p.XYZ()); }
gp_Pnt toPnt(const gp_Vec& v) { return gp_Pnt(v.XYZ()); }

// Newell normal of a polygon (unnormalised; length ~ 2 * area).
gp_Vec newell(const std::vector<gp_Pnt>& p) {
    gp_Vec n(0, 0, 0);
    for (size_t i = 0; i < p.size(); ++i) {
        const gp_Pnt& a = p[i];
        const gp_Pnt& b = p[(i + 1) % p.size()];
        n.SetX(n.X() + (a.Y() - b.Y()) * (a.Z() + b.Z()));
        n.SetY(n.Y() + (a.Z() - b.Z()) * (a.X() + b.X()));
        n.SetZ(n.Z() + (a.X() - b.X()) * (a.Y() + b.Y()));
    }
    return n;
}

// Flip `face` so its normal agrees with `hint`.
void orient(const Cage& c, std::vector<int>& face, const gp_Vec& hint) {
    std::vector<gp_Pnt> p;
    for (int v : face) p.push_back(c.verts[static_cast<size_t>(v)]);
    if (newell(p).Dot(hint) < 0.0) std::reverse(face.begin(), face.end());
}

struct Topo {
    std::map<Edge, std::vector<int>> edgeFaces;          // edge -> faces
    std::vector<std::vector<Edge>> vertEdges;            // vertex -> edges
    std::vector<std::vector<int>> vertFaces;             // vertex -> faces
};

Topo topology(const Cage& c) {
    Topo t;
    t.vertEdges.resize(c.verts.size());
    t.vertFaces.resize(c.verts.size());
    for (size_t f = 0; f < c.faces.size(); ++f) {
        const auto& fv = c.faces[f];
        for (size_t i = 0; i < fv.size(); ++i) {
            const Edge e = edgeKey(fv[i], fv[(i + 1) % fv.size()]);
            auto& list = t.edgeFaces[e];
            if (list.empty()) {
                t.vertEdges[static_cast<size_t>(e.first)].push_back(e);
                t.vertEdges[static_cast<size_t>(e.second)].push_back(e);
            }
            list.push_back(static_cast<int>(f));
            t.vertFaces[static_cast<size_t>(fv[i])].push_back(static_cast<int>(f));
        }
    }
    return t;
}

bool sharp(const Cage& c, const Topo& t, const Edge& e) {
    auto it = t.edgeFaces.find(e);
    return c.creases.count(e) || it == t.edgeFaces.end() || it->second.size() < 2;
}

int other(const Edge& e, int v) { return e.first == v ? e.second : e.first; }

} // namespace

// ── Cage basics ─────────────────────────────────────────────────────────────

bool Cage::valid() const {
    if (verts.empty() || faces.empty()) return false;
    for (const auto& f : faces) {
        if (f.size() < 3) return false;
        for (int v : f)
            if (v < 0 || v >= static_cast<int>(verts.size())) return false;
    }
    return true;
}

std::vector<Edge> Cage::edges() const {
    std::set<Edge> s;
    for (const auto& f : faces)
        for (size_t i = 0; i < f.size(); ++i) s.insert(edgeKey(f[i], f[(i + 1) % f.size()]));
    return std::vector<Edge>(s.begin(), s.end());
}

std::vector<int> Cage::edgeFaces(const Edge& e) const {
    std::vector<int> out;
    for (size_t f = 0; f < faces.size(); ++f) {
        const auto& fv = faces[f];
        for (size_t i = 0; i < fv.size(); ++i)
            if (edgeKey(fv[i], fv[(i + 1) % fv.size()]) == e) { out.push_back(static_cast<int>(f)); break; }
    }
    return out;
}

bool Cage::closed() const {
    const Topo t = topology(*this);
    for (const auto& [e, fs] : t.edgeFaces)
        if (fs.size() < 2) return false;
    return !t.edgeFaces.empty();
}

void Cage::transformVertices(const std::vector<int>& ids, const gp_Trsf& tr) {
    std::set<int> done;
    for (int v : ids)
        if (v >= 0 && v < static_cast<int>(verts.size()) && done.insert(v).second)
            verts[static_cast<size_t>(v)].Transform(tr);
}

void Cage::setCrease(const std::vector<Edge>& es, bool on) {
    for (const Edge& e : es) {
        if (on) creases.insert(edgeKey(e.first, e.second));
        else creases.erase(edgeKey(e.first, e.second));
    }
}

gp_Vec Cage::faceNormal(int f) const {
    if (f < 0 || f >= static_cast<int>(faces.size())) return gp_Vec();
    std::vector<gp_Pnt> p;
    for (int v : faces[static_cast<size_t>(f)]) p.push_back(verts[static_cast<size_t>(v)]);
    gp_Vec n = newell(p);
    return n.Magnitude() > 1e-12 ? n.Normalized() : gp_Vec();
}

gp_Pnt Cage::faceCentre(int f) const {
    gp_Vec s(0, 0, 0);
    const auto& fv = faces[static_cast<size_t>(f)];
    for (int v : fv) s += toVec(verts[static_cast<size_t>(v)]);
    return toPnt(s / static_cast<double>(fv.size()));
}

gp_Vec Cage::vertexNormal(int v) const {
    gp_Vec n(0, 0, 0);
    for (size_t f = 0; f < faces.size(); ++f)
        if (std::find(faces[f].begin(), faces[f].end(), v) != faces[f].end())
            n += faceNormal(static_cast<int>(f));
    return n.Magnitude() > 1e-12 ? n.Normalized() : gp_Vec();
}

// ── Edits ───────────────────────────────────────────────────────────────────

namespace {
// Put `m` between a and b wherever a face has them as consecutive corners,
// skipping face `skip`.
void insertOnEdge(Cage& c, int a, int b, int m, int skip) {
    for (size_t f = 0; f < c.faces.size(); ++f) {
        if (static_cast<int>(f) == skip) continue;
        auto& fv = c.faces[f];
        for (size_t i = 0; i < fv.size(); ++i) {
            const int x = fv[i], y = fv[(i + 1) % fv.size()];
            if ((x == a && y == b) || (x == b && y == a)) {
                fv.insert(fv.begin() + static_cast<long>(i + 1), m);
                break;
            }
        }
    }
}
// A crease split by a new vertex m stays creased on both halves.
void splitCrease(Cage& c, int a, int b, int m) {
    if (c.creases.erase(edgeKey(a, b))) {
        c.creases.insert(edgeKey(a, m));
        c.creases.insert(edgeKey(m, b));
    }
}
} // namespace

void Cage::subdivideFace(int face) {
    if (face < 0 || face >= static_cast<int>(faces.size())) return;
    const std::vector<int> fv = faces[static_cast<size_t>(face)];
    const size_t n = fv.size();
    const int centre = static_cast<int>(verts.size());
    verts.push_back(faceCentre(face));
    std::vector<int> mids(n);
    for (size_t i = 0; i < n; ++i) {
        const int a = fv[i], b = fv[(i + 1) % n];
        mids[i] = static_cast<int>(verts.size());
        verts.push_back(toPnt((toVec(verts[static_cast<size_t>(a)]) +
                               toVec(verts[static_cast<size_t>(b)])) * 0.5));
        insertOnEdge(*this, a, b, mids[i], face);
        splitCrease(*this, a, b, mids[i]);
    }
    std::vector<std::vector<int>> quads;
    for (size_t i = 0; i < n; ++i)
        quads.push_back({fv[i], mids[i], centre, mids[(i + n - 1) % n]});
    faces[static_cast<size_t>(face)] = quads[0];
    for (size_t i = 1; i < n; ++i) faces.push_back(quads[i]);
}

void Cage::insertEdgeLoop(const Edge& start) {
    const Topo t = topology(*this);
    auto it0 = t.edgeFaces.find(edgeKey(start.first, start.second));
    if (it0 == t.edgeFaces.end()) return;
    // Walk the quad ring both ways from the start edge. Each step records
    // (face, entry position i): the face is entered across fv[i]-fv[i+1].
    struct Step { int face; int entry; };
    std::vector<Step> ring;
    std::set<int> seen;
    auto walk = [&](int face, Edge entry, bool front) {
        while (face >= 0 && !seen.count(face)) {
            const auto& fv = faces[static_cast<size_t>(face)];
            if (fv.size() != 4) break;
            int i = -1;
            for (int k = 0; k < 4; ++k)
                if (edgeKey(fv[static_cast<size_t>(k)], fv[static_cast<size_t>((k + 1) % 4)]) == entry) i = k;
            if (i < 0) break;
            seen.insert(face);
            if (front) ring.push_back({face, i});
            else ring.insert(ring.begin(), {face, i});
            const Edge opp = edgeKey(fv[static_cast<size_t>((i + 2) % 4)], fv[static_cast<size_t>((i + 3) % 4)]);
            const auto& fs = t.edgeFaces.at(opp);
            int next = -1;
            for (int g : fs) if (g != face) next = g;
            face = next;
            entry = opp;
        }
    };
    const Edge e0 = edgeKey(start.first, start.second);
    walk(it0->second[0], e0, true);
    if (it0->second.size() > 1) walk(it0->second[1], e0, false);
    if (ring.empty()) return;

    // One midpoint per crossed edge.
    std::map<Edge, int> mid;
    auto midOf = [&](int a, int b) {
        const Edge e = edgeKey(a, b);
        auto m = mid.find(e);
        if (m != mid.end()) return m->second;
        const int id = static_cast<int>(verts.size());
        verts.push_back(toPnt((toVec(verts[static_cast<size_t>(a)]) +
                               toVec(verts[static_cast<size_t>(b)])) * 0.5));
        mid[e] = id;
        return id;
    };
    std::vector<std::vector<int>> added;
    std::set<int> ringFaces;
    for (const Step& s : ring) ringFaces.insert(s.face);
    for (const Step& s : ring) {
        const std::vector<int> fv = faces[static_cast<size_t>(s.face)];
        const int i = s.entry;
        const int p0 = fv[static_cast<size_t>(i)], p1 = fv[static_cast<size_t>((i + 1) % 4)];
        const int p2 = fv[static_cast<size_t>((i + 2) % 4)], p3 = fv[static_cast<size_t>((i + 3) % 4)];
        const int m1 = midOf(p0, p1), m2 = midOf(p2, p3);
        faces[static_cast<size_t>(s.face)] = {p0, m1, m2, p3};
        added.push_back({m1, p1, p2, m2});
    }
    for (auto& f : added) faces.push_back(f);
    // Faces outside the ring that share a crossed edge get the midpoint too.
    for (const auto& [e, m] : mid) {
        for (size_t f = 0; f < faces.size(); ++f) {
            if (ringFaces.count(static_cast<int>(f))) continue;
            auto& fv = faces[f];
            for (size_t k = 0; k < fv.size(); ++k) {
                const int x = fv[k], y = fv[(k + 1) % fv.size()];
                if (edgeKey(x, y) == e) {
                    fv.insert(fv.begin() + static_cast<long>(k + 1), m);
                    break;
                }
            }
        }
        splitCrease(*this, e.first, e.second, m);
    }
}

std::vector<int> Cage::extrudeFaces(const std::vector<int>& sel, double dist) {
    std::set<int> region(sel.begin(), sel.end());
    for (auto it = region.begin(); it != region.end();)
        if (*it < 0 || *it >= static_cast<int>(faces.size())) it = region.erase(it); else ++it;
    if (region.empty()) return {};
    gp_Vec n(0, 0, 0);
    for (int f : region) n += faceNormal(f);
    if (n.Magnitude() < 1e-12) return {};
    n.Normalize();
    // Directed edges of the region; a border edge has no reverse inside it.
    std::set<std::pair<int, int>> directed;
    for (int f : region) {
        const auto& fv = faces[static_cast<size_t>(f)];
        for (size_t i = 0; i < fv.size(); ++i) directed.insert({fv[i], fv[(i + 1) % fv.size()]});
    }
    std::map<int, int> moved;
    auto copyOf = [&](int v) {
        auto it = moved.find(v);
        if (it != moved.end()) return it->second;
        const int id = static_cast<int>(verts.size());
        verts.push_back(verts[static_cast<size_t>(v)].Translated(n * dist));
        moved[v] = id;
        return id;
    };
    std::vector<std::vector<int>> sides;
    for (int f : region) {
        const auto& fv = faces[static_cast<size_t>(f)];
        for (size_t i = 0; i < fv.size(); ++i) {
            const int a = fv[i], b = fv[(i + 1) % fv.size()];
            if (directed.count({b, a})) continue;   // interior to the region
            sides.push_back({a, b, copyOf(b), copyOf(a)});
        }
    }
    for (int f : region)
        for (int& v : faces[static_cast<size_t>(f)]) v = copyOf(v);
    // Creases inside the region move with it.
    std::set<Edge> moveCreases;
    for (const Edge& e : creases)
        if (moved.count(e.first) && moved.count(e.second) &&
            directed.count({e.first, e.second}) + directed.count({e.second, e.first}) == 2)
            moveCreases.insert(e);
    for (const Edge& e : moveCreases) {
        creases.erase(e);
        creases.insert(edgeKey(moved[e.first], moved[e.second]));
    }
    for (auto& s : sides) faces.push_back(s);
    return std::vector<int>(region.begin(), region.end());
}

void Cage::deleteFaces(const std::vector<int>& sel) {
    std::set<int> drop(sel.begin(), sel.end());
    std::vector<std::vector<int>> keep;
    for (size_t f = 0; f < faces.size(); ++f)
        if (!drop.count(static_cast<int>(f))) keep.push_back(faces[f]);
    std::vector<int> remap(verts.size(), -1);
    std::vector<gp_Pnt> nv;
    for (auto& f : keep)
        for (int& v : f) {
            if (remap[static_cast<size_t>(v)] < 0) {
                remap[static_cast<size_t>(v)] = static_cast<int>(nv.size());
                nv.push_back(verts[static_cast<size_t>(v)]);
            }
            v = remap[static_cast<size_t>(v)];
        }
    std::set<Edge> nc;
    for (const Edge& e : creases)
        if (remap[static_cast<size_t>(e.first)] >= 0 && remap[static_cast<size_t>(e.second)] >= 0)
            nc.insert(edgeKey(remap[static_cast<size_t>(e.first)], remap[static_cast<size_t>(e.second)]));
    verts = std::move(nv);
    faces = std::move(keep);
    creases = std::move(nc);
}

// ── Symmetry ────────────────────────────────────────────────────────────────

gp_Pnt Cage::mirror(const gp_Pnt& p) const {
    if (symAxis < 0 || symAxis > 2) return p;
    gp_Pnt q = p;
    q.SetCoord(symAxis + 1, 2.0 * symPos - p.Coord(symAxis + 1));
    return q;
}

std::vector<int> Cage::mirrorPartners() const {
    if (symAxis < 0 || symAxis > 2) return {};
    Bnd_Box bb;
    for (const gp_Pnt& p : verts) bb.Add(p);
    const double tol = std::max(1e-9, std::sqrt(bb.IsVoid() ? 1.0 : bb.SquareExtent()) * 1e-5);
    // Bucket by a coarse key on the two in-plane coordinates.
    const int a = (symAxis + 1) % 3, b = (symAxis + 2) % 3;
    const double cell = tol * 4.0;
    std::map<std::pair<long long, long long>, std::vector<int>> grid;
    auto key = [&](const gp_Pnt& p, int da, int db) {
        return std::make_pair(static_cast<long long>(std::floor(p.Coord(a + 1) / cell)) + da,
                              static_cast<long long>(std::floor(p.Coord(b + 1) / cell)) + db);
    };
    for (size_t i = 0; i < verts.size(); ++i) grid[key(verts[i], 0, 0)].push_back(static_cast<int>(i));
    std::vector<int> partner(verts.size(), -1);
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt m = mirror(verts[i]);
        double best = tol;
        for (int da = -1; da <= 1; ++da)
            for (int db = -1; db <= 1; ++db) {
                auto it = grid.find(key(m, da, db));
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    const double d = verts[static_cast<size_t>(j)].Distance(m);
                    if (d <= best) { best = d; partner[i] = j; }
                }
            }
    }
    return partner;
}

void Cage::transformSymmetric(const Cage& start, const std::vector<int>& ids, const gp_Trsf& t) {
    std::set<int> moved;
    for (int v : ids)
        if (v >= 0 && v < static_cast<int>(verts.size()) && v < static_cast<int>(start.verts.size()))
            moved.insert(v);
    for (int v : moved) verts[static_cast<size_t>(v)] = start.verts[static_cast<size_t>(v)].Transformed(t);
    if (symAxis < 0) return;
    const std::vector<int> partner = start.mirrorPartners();
    for (int v : moved) {
        const int p = partner.empty() ? -1 : partner[static_cast<size_t>(v)];
        if (p == v) {
            // On the plane: stays on it.
            verts[static_cast<size_t>(v)].SetCoord(symAxis + 1, symPos);
        } else if (p >= 0 && !moved.count(p)) {
            verts[static_cast<size_t>(p)] = mirror(verts[static_cast<size_t>(v)]);
        }
    }
}

int Cage::mirrorFace(int f, const std::vector<int>& partner) const {
    if (f < 0 || f >= static_cast<int>(faces.size()) || partner.empty()) return -1;
    std::set<int> want;
    for (int v : faces[static_cast<size_t>(f)]) {
        const int p = partner[static_cast<size_t>(v)];
        if (p < 0) return -1;
        want.insert(p);
    }
    for (size_t g = 0; g < faces.size(); ++g) {
        if (faces[g].size() != want.size()) continue;
        std::set<int> have(faces[g].begin(), faces[g].end());
        if (have == want) return static_cast<int>(g);
    }
    return -1;
}

Edge Cage::mirrorEdge(const Edge& e, const std::vector<int>& partner) const {
    if (partner.empty()) return {-1, -1};
    const int a = partner[static_cast<size_t>(e.first)], b = partner[static_cast<size_t>(e.second)];
    if (a < 0 || b < 0) return {-1, -1};
    return edgeKey(a, b);
}

// ── Serialization ───────────────────────────────────────────────────────────

std::string Cage::serialize() const {
    std::ostringstream os;
    os.precision(10);
    for (const gp_Pnt& p : verts) os << "V " << p.X() << ' ' << p.Y() << ' ' << p.Z() << '\n';
    for (const auto& f : faces) {
        os << 'F';
        for (int v : f) os << ' ' << v;
        os << '\n';
    }
    for (const Edge& e : creases) os << "C " << e.first << ' ' << e.second << '\n';
    if (symAxis >= 0) os << "S " << symAxis << ' ' << symPos << '\n';
    return os.str();
}

bool Cage::deserialize(const std::string& s) {
    Cage c;
    std::istringstream is(s);
    std::string line;
    while (std::getline(is, line)) {
        std::istringstream ls(line);
        char k = 0;
        ls >> k;
        if (k == 'V') {
            double x, y, z;
            if (ls >> x >> y >> z) c.verts.emplace_back(x, y, z);
        } else if (k == 'F') {
            std::vector<int> f;
            int v;
            while (ls >> v) f.push_back(v);
            if (f.size() >= 3 && f.size() < 4096) c.faces.push_back(f);
        } else if (k == 'C') {
            int a, b;
            if (ls >> a >> b) c.creases.insert(edgeKey(a, b));
        } else if (k == 'S') {
            int ax;
            double pos;
            if ((ls >> ax >> pos) && ax >= 0 && ax <= 2) { c.symAxis = ax; c.symPos = pos; }
        }
        if (c.verts.size() > 1000000 || c.faces.size() > 1000000) return false;
    }
    if (!c.valid()) return false;
    *this = std::move(c);
    return true;
}

// ── Primitives ──────────────────────────────────────────────────────────────

Cage box(double sx, double sy, double sz, int segs) {
    segs = std::max(1, std::min(segs, 16));
    Cage c;
    std::map<std::tuple<int, int, int>, int> ids;
    auto id = [&](int i, int j, int k) {
        const auto key = std::make_tuple(i, j, k);
        auto it = ids.find(key);
        if (it != ids.end()) return it->second;
        const int n = static_cast<int>(c.verts.size());
        c.verts.emplace_back(sx * (static_cast<double>(i) / segs - 0.5),
                             sy * (static_cast<double>(j) / segs - 0.5),
                             sz * (static_cast<double>(k) / segs - 0.5));
        ids[key] = n;
        return n;
    };
    // Six sides: fixed axis d at 0 or segs, the other two axes swept.
    for (int d = 0; d < 3; ++d)
        for (int side = 0; side < 2; ++side) {
            const int fixedVal = side ? segs : 0;
            gp_Vec out(0, 0, 0);
            out.SetCoord(d + 1, side ? 1.0 : -1.0);
            for (int a = 0; a < segs; ++a)
                for (int b = 0; b < segs; ++b) {
                    auto at = [&](int u, int v) {
                        int ijk[3];
                        ijk[d] = fixedVal;
                        ijk[(d + 1) % 3] = u;
                        ijk[(d + 2) % 3] = v;
                        return id(ijk[0], ijk[1], ijk[2]);
                    };
                    std::vector<int> f = {at(a, b), at(a + 1, b), at(a + 1, b + 1), at(a, b + 1)};
                    orient(c, f, out);
                    c.faces.push_back(f);
                }
        }
    return c;
}

Cage plane(double sx, double sz, int segX, int segZ) {
    segX = std::max(1, std::min(segX, 64));
    segZ = std::max(1, std::min(segZ, 64));
    Cage c;
    for (int k = 0; k <= segZ; ++k)
        for (int i = 0; i <= segX; ++i)
            c.verts.emplace_back(sx * (static_cast<double>(i) / segX - 0.5), 0.0,
                                 sz * (static_cast<double>(k) / segZ - 0.5));
    auto at = [&](int i, int k) { return k * (segX + 1) + i; };
    for (int k = 0; k < segZ; ++k)
        for (int i = 0; i < segX; ++i) {
            std::vector<int> f = {at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1)};
            orient(c, f, gp_Vec(0, 1, 0));
            c.faces.push_back(f);
        }
    return c;
}

Cage cylinder(double r, double h, int around, int segsH) {
    around = std::max(3, std::min(around, 64));
    segsH = std::max(1, std::min(segsH, 64));
    Cage c;
    for (int k = 0; k <= segsH; ++k)
        for (int i = 0; i < around; ++i) {
            const double a = 2.0 * M_PI * i / around;
            c.verts.emplace_back(r * std::cos(a), h * (static_cast<double>(k) / segsH - 0.5),
                                 r * std::sin(a));
        }
    auto at = [&](int i, int k) { return k * around + (i % around); };
    for (int k = 0; k < segsH; ++k)
        for (int i = 0; i < around; ++i) {
            std::vector<int> f = {at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1)};
            const double a = 2.0 * M_PI * (i + 0.5) / around;
            orient(c, f, gp_Vec(std::cos(a), 0, std::sin(a)));
            c.faces.push_back(f);
        }
    std::vector<int> bottom, top;
    for (int i = 0; i < around; ++i) { bottom.push_back(at(i, 0)); top.push_back(at(i, segsH)); }
    orient(c, bottom, gp_Vec(0, -1, 0));
    orient(c, top, gp_Vec(0, 1, 0));
    c.faces.push_back(bottom);
    c.faces.push_back(top);
    return c;
}

Cage sphere(double r, int segs) {
    Cage c = box(2.0, 2.0, 2.0, std::max(1, segs));
    for (gp_Pnt& p : c.verts) {
        gp_Vec v = toVec(p);
        if (v.Magnitude() > 1e-12) p = toPnt(v.Normalized() * r);
    }
    return c;
}

// ── Subdivision ─────────────────────────────────────────────────────────────

Subdivided subdivide(const Cage& c) {
    Subdivided out;
    const Topo t = topology(c);
    const size_t nV = c.verts.size(), nF = c.faces.size();
    std::map<Edge, int> edgeIdx;
    for (const auto& [e, fs] : t.edgeFaces) edgeIdx[e] = static_cast<int>(edgeIdx.size());
    Cage& m = out.mesh;
    m.verts.resize(nV + nF + edgeIdx.size());

    std::vector<gp_Vec> fp(nF);
    for (size_t f = 0; f < nF; ++f) {
        fp[f] = toVec(c.faceCentre(static_cast<int>(f)));
        m.verts[nV + f] = toPnt(fp[f]);
    }
    for (const auto& [e, idx] : edgeIdx) {
        const gp_Vec a = toVec(c.verts[static_cast<size_t>(e.first)]);
        const gp_Vec b = toVec(c.verts[static_cast<size_t>(e.second)]);
        gp_Vec p = (a + b) * 0.5;
        if (!sharp(c, t, e)) {
            const auto& fs = t.edgeFaces.at(e);
            p = (a + b + fp[static_cast<size_t>(fs[0])] + fp[static_cast<size_t>(fs[1])]) * 0.25;
        }
        m.verts[nV + nF + static_cast<size_t>(idx)] = toPnt(p);
    }
    for (size_t v = 0; v < nV; ++v) {
        const gp_Vec P = toVec(c.verts[v]);
        const auto& es = t.vertEdges[v];
        const auto& fs = t.vertFaces[v];
        std::vector<int> sharpNb;
        for (const Edge& e : es) if (sharp(c, t, e)) sharpNb.push_back(other(e, static_cast<int>(v)));
        gp_Vec np = P;
        // Valence 2 (a sheet's corner) or 3+ sharp edges: a corner, fixed.
        if (es.size() <= 2 || fs.empty() || sharpNb.size() >= 3) {
            np = P;
        } else if (sharpNb.size() == 2) {
            np = (P * 6.0 + toVec(c.verts[static_cast<size_t>(sharpNb[0])]) +
                  toVec(c.verts[static_cast<size_t>(sharpNb[1])])) / 8.0;
        } else {
            const double n = static_cast<double>(es.size());
            gp_Vec F(0, 0, 0), R(0, 0, 0);
            std::set<int> uniqF(fs.begin(), fs.end());
            for (int f : uniqF) F += fp[static_cast<size_t>(f)];
            F /= static_cast<double>(uniqF.size());
            for (const Edge& e : es)
                R += (P + toVec(c.verts[static_cast<size_t>(other(e, static_cast<int>(v)))])) * 0.5;
            R /= n;
            np = (F + R * 2.0 + P * (n - 3.0)) / n;
        }
        m.verts[v] = toPnt(np);
    }
    out.children.resize(nF);
    for (size_t f = 0; f < nF; ++f) {
        const auto& fv = c.faces[f];
        const size_t n = fv.size();
        for (size_t j = 0; j < n; ++j) {
            const int a = fv[j], b = fv[(j + 1) % n], z = fv[(j + n - 1) % n];
            const int ej = static_cast<int>(nV + nF) + edgeIdx.at(edgeKey(a, b));
            const int ez = static_cast<int>(nV + nF) + edgeIdx.at(edgeKey(z, a));
            out.children[f].push_back(static_cast<int>(m.faces.size()));
            m.faces.push_back({a, ej, static_cast<int>(nV + f), ez});
        }
    }
    for (const Edge& e : c.creases) {
        auto it = edgeIdx.find(e);
        if (it == edgeIdx.end()) continue;
        const int mid = static_cast<int>(nV + nF) + it->second;
        m.creases.insert(edgeKey(e.first, mid));
        m.creases.insert(edgeKey(mid, e.second));
    }
    return out;
}

std::vector<gp_Pnt> limitPositions(const Cage& c) {
    const Topo t = topology(c);
    std::vector<gp_Pnt> out(c.verts.size());
    for (size_t v = 0; v < c.verts.size(); ++v) {
        const gp_Vec P = toVec(c.verts[v]);
        const auto& es = t.vertEdges[v];
        std::vector<int> sharpNb;
        for (const Edge& e : es) if (sharp(c, t, e)) sharpNb.push_back(other(e, static_cast<int>(v)));
        if (es.size() <= 2 || sharpNb.size() >= 3) { out[v] = c.verts[v]; continue; }
        if (sharpNb.size() == 2) {
            out[v] = toPnt((P * 4.0 + toVec(c.verts[static_cast<size_t>(sharpNb[0])]) +
                            toVec(c.verts[static_cast<size_t>(sharpNb[1])])) / 6.0);
            continue;
        }
        // Smooth (quad mesh): (n^2 P + 4 sum e_i + sum f_i) / (n (n + 5)).
        const double n = static_cast<double>(es.size());
        gp_Vec E(0, 0, 0), F(0, 0, 0);
        for (const Edge& e : es) E += toVec(c.verts[static_cast<size_t>(other(e, static_cast<int>(v)))]);
        bool quads = true;
        for (int f : t.vertFaces[v]) {
            const auto& fv = c.faces[static_cast<size_t>(f)];
            if (fv.size() != 4) { quads = false; break; }
            const size_t k = static_cast<size_t>(std::find(fv.begin(), fv.end(), static_cast<int>(v)) - fv.begin());
            F += toVec(c.verts[static_cast<size_t>(fv[(k + 2) % 4])]);
        }
        if (!quads) { out[v] = c.verts[v]; continue; }
        out[v] = toPnt((P * (n * n) + E * 4.0 + F) / (n * (n + 5.0)));
    }
    return out;
}

Cage subdivideLevels(const Cage& c, int levels, bool limit) {
    Cage cur = c;
    for (int i = 0; i < levels; ++i) cur = subdivide(cur).mesh;
    if (limit && levels > 0) cur.verts = limitPositions(cur);
    return cur;
}

TopoDS_Shape displayShape(const Cage& c, int levels) {
    if (!c.valid()) return TopoDS_Shape();
    const Cage m = subdivideLevels(c, std::max(1, levels), true);
    BRep_Builder b;
    TopoDS_Compound comp;
    b.MakeCompound(comp);
    int n = 0;
    auto tri = [&](const gp_Pnt& p, const gp_Pnt& q, const gp_Pnt& r) {
        if (p.Distance(q) < 1e-9 || q.Distance(r) < 1e-9 || r.Distance(p) < 1e-9) return;
        if (gp_Vec(p, q).Crossed(gp_Vec(p, r)).Magnitude() < 1e-14) return;
        try {
            BRepBuilderAPI_MakePolygon poly(p, q, r, Standard_True);
            if (!poly.IsDone()) return;
            BRepBuilderAPI_MakeFace mf(poly.Wire(), Standard_True);
            if (mf.IsDone()) { b.Add(comp, mf.Face()); ++n; }
        } catch (...) {}
    };
    for (const auto& f : m.faces) {
        if (f.size() != 4) continue;
        const gp_Pnt& p0 = m.verts[static_cast<size_t>(f[0])];
        const gp_Pnt& p1 = m.verts[static_cast<size_t>(f[1])];
        const gp_Pnt& p2 = m.verts[static_cast<size_t>(f[2])];
        const gp_Pnt& p3 = m.verts[static_cast<size_t>(f[3])];
        tri(p0, p1, p2);
        tri(p0, p2, p3);
    }
    return n ? TopoDS_Shape(comp) : TopoDS_Shape();
}

std::vector<float> displayTriangles(const Cage& c, int levels) {
    std::vector<float> out;
    if (!c.valid()) return out;
    const Cage m = subdivideLevels(c, std::max(1, levels), true);
    // Smooth normals: area-weighted face normals summed at each vertex.
    // Creased edges keep separate normals per side - handled by giving each
    // quad corner the vertex normal only when no crease touches it.
    std::vector<gp_Vec> vn(m.verts.size(), gp_Vec(0, 0, 0));
    std::vector<gp_Vec> fn(m.faces.size());
    for (size_t f = 0; f < m.faces.size(); ++f) {
        std::vector<gp_Pnt> p;
        for (int v : m.faces[f]) p.push_back(m.verts[static_cast<size_t>(v)]);
        fn[f] = newell(p);
        for (int v : m.faces[f]) vn[static_cast<size_t>(v)] += fn[f];
    }
    std::set<int> creaseVerts;
    for (const Edge& e : m.creases) { creaseVerts.insert(e.first); creaseVerts.insert(e.second); }
    out.reserve(m.faces.size() * 36);
    auto push = [&](int v, const gp_Vec& faceN) {
        const gp_Pnt& p = m.verts[static_cast<size_t>(v)];
        gp_Vec n = creaseVerts.count(v) ? faceN : vn[static_cast<size_t>(v)];
        if (n.Magnitude() > 1e-12) n.Normalize();
        out.insert(out.end(), {float(p.X()), float(p.Y()), float(p.Z()),
                               float(n.X()), float(n.Y()), float(n.Z())});
    };
    for (size_t f = 0; f < m.faces.size(); ++f) {
        const auto& q = m.faces[f];
        if (q.size() != 4) continue;
        push(q[0], fn[f]); push(q[1], fn[f]); push(q[2], fn[f]);
        push(q[0], fn[f]); push(q[2], fn[f]); push(q[3], fn[f]);
    }
    return out;
}

bool rayHit(const std::vector<float>& tris, const gp_Pnt& o, const gp_Vec& dirIn, double& tOut) {
    if (dirIn.Magnitude() < 1e-12) return false;
    const gp_Vec d = dirIn.Normalized();
    bool hit = false;
    double best = 1e300;
    for (size_t i = 0; i + 17 < tris.size(); i += 18) {
        const gp_Vec a(tris[i], tris[i + 1], tris[i + 2]);
        const gp_Vec b(tris[i + 6], tris[i + 7], tris[i + 8]);
        const gp_Vec c(tris[i + 12], tris[i + 13], tris[i + 14]);
        // Moller-Trumbore, both sides.
        const gp_Vec e1 = b - a, e2 = c - a;
        const gp_Vec pv = d.Crossed(e2);
        const double det = e1.Dot(pv);
        if (std::abs(det) < 1e-14) continue;
        const gp_Vec tv = toVec(o) - a;
        const double u = tv.Dot(pv) / det;
        if (u < 0 || u > 1) continue;
        const gp_Vec qv = tv.Crossed(e1);
        const double v = d.Dot(qv) / det;
        if (v < 0 || u + v > 1) continue;
        const double t = e2.Dot(qv) / det;
        if (t > 0 && t < best) { best = t; hit = true; }
    }
    if (hit) tOut = best;
    return hit;
}

// ── NURBS ───────────────────────────────────────────────────────────────────

TopoDS_Shape toNurbs(const Cage& c, int gridLevels) {
    if (!c.valid()) return TopoDS_Shape();
    gridLevels = std::max(1, std::min(gridLevels, 5));
    // Level 1, then gridLevels more, keeping every parent -> children map.
    std::vector<Subdivided> lv;
    lv.push_back(subdivide(c));                       // lv[0].mesh = level 1
    for (int k = 0; k < gridLevels; ++k) lv.push_back(subdivide(lv.back().mesh));
    const Cage& finest = lv.back().mesh;
    const std::vector<gp_Pnt> lim = limitPositions(finest);
    const int N = 1 << gridLevels;

    // A child quad's local (a, b) in its parent's (u, v): child j sits at
    // the parent's corner j, its u running toward corner j+1.
    struct Affine { double ox, oy, ux, uy, vx, vy; };
    auto apply = [](const Affine& T, double a, double b) {
        return std::make_pair(T.ox + a * T.ux + b * T.vx, T.oy + a * T.uy + b * T.vy);
    };
    auto childMap = [](int j, double a, double b) {
        switch (j) {
        case 0: return std::make_pair(a / 2, b / 2);
        case 1: return std::make_pair(1 - b / 2, a / 2);
        case 2: return std::make_pair(1 - a / 2, 1 - b / 2);
        default: return std::make_pair(b / 2, 1 - a / 2);
        }
    };

    std::vector<TopoDS_Face> patches;
    const Cage& level1 = lv[0].mesh;
    for (size_t q = 0; q < level1.faces.size(); ++q) {
        TColgp_Array2OfPnt grid(1, N + 1, 1, N + 1);
        std::vector<char> filled(static_cast<size_t>((N + 1) * (N + 1)), 0);
        std::function<void(int, int, const Affine&)> fill = [&](int level, int quad, const Affine& T) {
            if (level == gridLevels) {
                const auto& fv = finest.faces[static_cast<size_t>(quad)];
                const double cu[4] = {0, 1, 1, 0}, cv[4] = {0, 0, 1, 1};
                for (int k = 0; k < 4; ++k) {
                    const auto uv = apply(T, cu[k], cv[k]);
                    const int i = static_cast<int>(std::lround(uv.first));
                    const int jj = static_cast<int>(std::lround(uv.second));
                    if (i < 0 || i > N || jj < 0 || jj > N) continue;
                    grid.SetValue(i + 1, jj + 1, lim[static_cast<size_t>(fv[static_cast<size_t>(k)])]);
                    filled[static_cast<size_t>(jj * (N + 1) + i)] = 1;
                }
                return;
            }
            const auto& kids = lv[static_cast<size_t>(level + 1)].children[static_cast<size_t>(quad)];
            for (int j = 0; j < 4 && j < static_cast<int>(kids.size()); ++j) {
                const auto o = childMap(j, 0, 0), u = childMap(j, 1, 0), v = childMap(j, 0, 1);
                const auto O = apply(T, o.first, o.second);
                const auto U = apply(T, u.first, u.second);
                const auto V = apply(T, v.first, v.second);
                const Affine C{O.first, O.second, U.first - O.first, U.second - O.second,
                               V.first - O.first, V.second - O.second};
                fill(level + 1, kids[static_cast<size_t>(j)], C);
            }
        };
        fill(0, static_cast<int>(q), Affine{0, 0, double(N), 0, 0, double(N)});
        if (std::find(filled.begin(), filled.end(), 0) != filled.end()) return TopoDS_Shape();
        try {
            GeomAPI_PointsToBSplineSurface interp;
            interp.Interpolate(grid, Approx_IsoParametric, Standard_False);
            if (!interp.IsDone()) return TopoDS_Shape();
            BRepBuilderAPI_MakeFace mf(interp.Surface(), 1e-7);
            if (!mf.IsDone()) return TopoDS_Shape();
            patches.push_back(mf.Face());
        } catch (...) {
            return TopoDS_Shape();
        }
    }
    if (patches.empty()) return TopoDS_Shape();
    try {
        Bnd_Box bb;
        for (const gp_Pnt& p : c.verts) bb.Add(p);
        const double tol = std::max(1e-5, std::sqrt(bb.SquareExtent()) * 1e-5);
        BRepBuilderAPI_Sewing sew(tol);
        for (const TopoDS_Face& f : patches) sew.Add(f);
        sew.Perform();
        TopoDS_Shape shape = sew.SewedShape();
        if (shape.IsNull()) return TopoDS_Shape();
        if (c.closed()) {
            TopExp_Explorer sx(shape, TopAbs_SHELL);
            if (sx.More()) {
                BRepBuilderAPI_MakeSolid ms(TopoDS::Shell(sx.Current()));
                if (ms.IsDone()) {
                    TopoDS_Solid solid = ms.Solid();
                    GProp_GProps g;
                    BRepGProp::VolumeProperties(solid, g);
                    if (g.Mass() < 0) solid.Reverse();
                    return solid;
                }
            }
        }
        return shape;
    } catch (...) {
        return TopoDS_Shape();
    }
}

} // namespace form
} // namespace materializr
