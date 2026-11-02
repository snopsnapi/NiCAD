#include "StudioSplineOp.h"
#include "FitSpline.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <imgui.h>
#include "../i18n.h"

TopoDS_Shape StudioSplineOp::buildCurve(const std::vector<gp_Pnt>& pts, bool closed,
                                        const gp_Vec& startTan, const gp_Vec& endTan,
                                        const std::vector<PointCond>& conds) {
    if (pts.size() < 2) return TopoDS_Shape();
    std::vector<glm::dvec3> p3;
    std::vector<PointCond> c3;   // conditions of the points kept
    p3.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        const gp_Pnt& p = pts[i];
        // Coincident neighbours make the chord parameters degenerate.
        if (!p3.empty() && glm::length(glm::dvec3(p.X(), p.Y(), p.Z()) - p3.back()) < 1e-7)
            continue;
        p3.emplace_back(p.X(), p.Y(), p.Z());
        c3.push_back(i < conds.size() ? conds[i] : PointCond());
    }
    if (p3.size() < 2) return TopoDS_Shape();
    try {
        Handle(Geom_BSplineCurve) curve;
        if (closed && p3.size() >= 3) {
            curve = materializr::fitspline::interpolateClosed(p3);
        } else {
            // fitspline's parameter runs 0..1 by chord length, so a unit
            // tangent scales by the total chord (same as the sketch splines).
            double chord = 0.0;
            for (size_t i = 1; i < p3.size(); ++i) chord += glm::length(p3[i] - p3[i - 1]);
            std::vector<materializr::fitspline::Constraint> cons;
            const int n = static_cast<int>(p3.size());
            // Point conditions. A direction is aligned with the curve's run
            // through that point (neighbour to neighbour), so G1 never folds
            // the curve back on itself.
            for (int i = 0; i < n; ++i) {
                const PointCond& pc = c3[static_cast<size_t>(i)];
                if (pc.type == Cond::Free || pc.dir.Magnitude() < 1e-9) continue;
                if ((i == 0 && startTan.Magnitude() > 1e-9) ||
                    (i == n - 1 && endTan.Magnitude() > 1e-9)) continue;
                const glm::dvec3 run = p3[static_cast<size_t>(std::min(i + 1, n - 1))] -
                                       p3[static_cast<size_t>(std::max(i - 1, 0))];
                gp_Vec t = pc.dir.Normalized();
                if (t.X() * run.x + t.Y() * run.y + t.Z() * run.z < 0.0) t.Reverse();
                materializr::fitspline::Constraint c;
                c.index = i;
                c.d1 = glm::dvec3(t.X(), t.Y(), t.Z()) * chord;
                if (pc.type == Cond::G2)
                    c.d2 = glm::dvec3(pc.curv.X(), pc.curv.Y(), pc.curv.Z()) * (chord * chord);
                cons.push_back(c);
            }
            if (startTan.Magnitude() > 1e-9) {
                materializr::fitspline::Constraint c;
                c.index = 0;
                const gp_Vec t = startTan.Normalized() * chord;
                c.d1 = glm::dvec3(t.X(), t.Y(), t.Z());
                cons.push_back(c);
            }
            if (endTan.Magnitude() > 1e-9) {
                materializr::fitspline::Constraint c;
                c.index = static_cast<int>(p3.size()) - 1;
                const gp_Vec t = endTan.Normalized() * chord;
                c.d1 = glm::dvec3(t.X(), t.Y(), t.Z());
                cons.push_back(c);
            }
            materializr::fitspline::Curve fc;
            if (!materializr::fitspline::interpolate(p3, cons, 3, fc)) return TopoDS_Shape();
            curve = materializr::fitspline::toOcct(fc);
        }
        if (curve.IsNull()) return TopoDS_Shape();
        BRepBuilderAPI_MakeEdge mk(curve);
        if (!mk.IsDone()) return TopoDS_Shape();
        return BRepBuilderAPI_MakeWire(mk.Edge()).Wire();
    } catch (...) {
        return TopoDS_Shape();
    }
}

bool StudioSplineOp::execute(Document& doc) {
    const TopoDS_Shape w = buildCurve(m_points, m_closed, m_startTan, m_endTan, m_conds);
    if (w.IsNull()) return false;
    try {
        doc.addOrPutBody(m_createdBodyId, w, "Spline");
        return true;
    } catch (...) {
        return false;
    }
}

bool StudioSplineOp::undo(Document& doc) {
    try {
        if (m_createdBodyId >= 0) doc.removeBody(m_createdBodyId);   // id kept for redo
        return true;
    } catch (...) {
        return false;
    }
}

std::string StudioSplineOp::description() const {
    return "Studio spline, " + std::to_string(m_points.size()) + " points";
}

void StudioSplineOp::renderProperties() {
    ImGui::Text(materializr::tr("Points: %d"), static_cast<int>(m_points.size()));
    ImGui::Checkbox(materializr::tr("Closed"), &m_closed);
}

OperationDiff StudioSplineOp::captureDiff() const {
    OperationDiff d;
    if (m_createdBodyId >= 0) d.created.push_back(m_createdBodyId);
    return d;
}

std::string StudioSplineOp::serializeParams() const {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "closed=%d;t0=%.9g:%.9g:%.9g;t1=%.9g:%.9g:%.9g",
                  m_closed ? 1 : 0, m_startTan.X(), m_startTan.Y(), m_startTan.Z(),
                  m_endTan.X(), m_endTan.Y(), m_endTan.Z());
    std::string blob = buf;
    for (size_t i = 0; i < m_conds.size() && i < m_points.size(); ++i) {
        const PointCond& c = m_conds[i];
        if (c.type == Cond::Free) continue;
        std::snprintf(buf, sizeof(buf), ";c%zu=%d:%.9g:%.9g:%.9g:%.9g:%.9g:%.9g", i,
                      static_cast<int>(c.type), c.dir.X(), c.dir.Y(), c.dir.Z(),
                      c.curv.X(), c.curv.Y(), c.curv.Z());
        blob += buf;
    }
    blob += ";pts=";
    for (size_t i = 0; i < m_points.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s%.9g:%.9g:%.9g", i ? "," : "",
                      m_points[i].X(), m_points[i].Y(), m_points[i].Z());
        blob += buf;
    }
    return blob;
}

bool StudioSplineOp::deserializeParams(const std::string& blob) {
    bool any = false;
    m_conds.clear();   // only the listed points carry conditions
    size_t pos = 0;
    auto vec = [](const std::string& v, gp_Vec& out) {
        double x = 0, y = 0, z = 0;
        if (std::sscanf(v.c_str(), "%lf:%lf:%lf", &x, &y, &z) == 3) out = gp_Vec(x, y, z);
    };
    while (pos < blob.size()) {
        const size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        const std::string key = blob.substr(pos, eq - pos);
        const std::string val = blob.substr(eq + 1, end - eq - 1);
        if (key == "closed") { m_closed = (val == "1"); }
        else if (key.size() > 1 && key[0] == 'c' && key != "closed") {
            const long idx = std::strtol(key.c_str() + 1, nullptr, 10);
            int type = 0;
            double v[6] = {0, 0, 0, 0, 0, 0};
            if (idx >= 0 && idx < 4096 &&
                std::sscanf(val.c_str(), "%d:%lf:%lf:%lf:%lf:%lf:%lf", &type, &v[0], &v[1],
                            &v[2], &v[3], &v[4], &v[5]) == 7 && type >= 1 && type <= 2) {
                if (m_conds.size() <= static_cast<size_t>(idx)) m_conds.resize(idx + 1);
                m_conds[idx].type = static_cast<Cond>(type);
                m_conds[idx].dir = gp_Vec(v[0], v[1], v[2]);
                m_conds[idx].curv = gp_Vec(v[3], v[4], v[5]);
            }
        }
        else if (key == "t0") { vec(val, m_startTan); }
        else if (key == "t1") { vec(val, m_endTan); }
        else if (key == "pts") {
            m_points.clear();
            size_t p = 0;
            while (p < val.size() && m_points.size() < 4096) {
                size_t comma = val.find(',', p);
                if (comma == std::string::npos) comma = val.size();
                double x = 0, y = 0, z = 0;
                if (std::sscanf(val.substr(p, comma - p).c_str(), "%lf:%lf:%lf", &x, &y, &z) == 3)
                    m_points.emplace_back(x, y, z);
                p = comma + 1;
            }
            any = !m_points.empty();
        }
        pos = end + 1;
    }
    return any;
}

bool StudioSplineOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_points.size() < 2 || state.created.empty()) return false;
    m_createdBodyId = state.created.front();
    return true;
}
