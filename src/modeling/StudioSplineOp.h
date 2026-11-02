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
                                   const gp_Vec& endTan = gp_Vec());

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
    int m_createdBodyId = -1;
};
