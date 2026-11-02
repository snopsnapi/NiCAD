#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <string>
#include <vector>

// Studio Spline (fork): a 3D curve through points in space - the NX Studio
// Spline / Fusion 3D-sketch spline. The points ARE the parameters: the curve
// is the C2 interpolant through them (fitspline, chord-length parameters;
// periodic when closed) with optional end tangents (G1). It creates a
// curve-only body (a wire), so it renders, picks, saves and replays like any
// other body; downstream steps (a sweep along it) follow its edits.
//
// Points are moved by the regular Move gizmo: the app writes new positions
// with setPoint() and replays history from this step.
class StudioSplineOp : public Operation {
public:
    // Condition at a point (NX Studio Spline): the curve's direction there
    // (G1) and, for G2, also its curvature vector (kappa * unit normal toward
    // the centre). Directions are captured from the face/edge when set; the
    // curve passes them through as fitspline derivative rows. Open curves.
    enum class Cond { Free = 0, G1 = 1, G2 = 2 };
    struct PointCond {
        Cond type = Cond::Free;
        gp_Vec dir;     // unit tangent
        gp_Vec curv;    // kappa * normal (G2 only)
    };
    void setCondition(size_t i, const PointCond& c) {
        if (m_conds.size() < m_points.size()) m_conds.resize(m_points.size());
        if (i < m_conds.size()) m_conds[i] = c;
    }
    PointCond condition(size_t i) const {
        return i < m_conds.size() ? m_conds[i] : PointCond();
    }

    void setPoints(const std::vector<gp_Pnt>& pts) { m_points = pts; }
    const std::vector<gp_Pnt>& points() const { return m_points; }
    void setPoint(size_t i, const gp_Pnt& p) { if (i < m_points.size()) m_points[i] = p; }
    void setClosed(bool c) { m_closed = c; }
    bool closed() const { return m_closed; }
    // Optional end tangents (unit directions; zero = free). Open curves only.
    void setStartTangent(const gp_Vec& v) { m_startTan = v; }
    void setEndTangent(const gp_Vec& v) { m_endTan = v; }
    int createdBodyId() const { return m_createdBodyId; }

    // The curve through `pts` alone (no Document) - used by execute, the
    // creation tool's preview and the tests. Null for < 2 points / failure.
    static TopoDS_Shape buildCurve(const std::vector<gp_Pnt>& pts, bool closed,
                                   const gp_Vec& startTan = gp_Vec(),
                                   const gp_Vec& endTan = gp_Vec(),
                                   const std::vector<PointCond>& conds = {});

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Studio Spline"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "studio_spline"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    std::vector<gp_Pnt> m_points;
    bool m_closed = false;
    gp_Vec m_startTan;
    gp_Vec m_endTan;
    std::vector<PointCond> m_conds;   // parallel to m_points (may be shorter)
    int m_createdBodyId = -1;
};
