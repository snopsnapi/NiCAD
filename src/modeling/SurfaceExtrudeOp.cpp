#include "SurfaceExtrudeOp.h"
#include "Sketch.h"
#include "SurfaceOps.h"
#include "../ui/LengthField.h"
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <imgui.h>
#include "../i18n.h"

bool SurfaceExtrudeOp::execute(Document& doc) {
    if (m_sketchId < 0 || !(std::abs(m_distance) >= 1e-4)) return false;
    auto sk = doc.getSketch(m_sketchId);
    if (!sk) return false;
    try {
        const std::vector<TopoDS_Wire> wires = sk->buildCurveWires();
        if (wires.empty()) return false;
        // The profile is the curve set itself (the interactive tool reads it
        // for its arrow); rebuilt every run so a sketch edit flows through.
        BRep_Builder b;
        TopoDS_Compound c;
        b.MakeCompound(c);
        for (const TopoDS_Wire& w : wires) b.Add(c, w);
        m_profile = c;
        const gp_Dir n = sk->getPlane().Position().Direction();
        const TopoDS_Shape surf =
            materializr::surf::extrudeCurves(wires, gp_Vec(n) * m_distance);
        if (surf.IsNull()) return false;
        doc.addOrPutBody(m_createdBodyId, surf, "Surface");
        return true;
    } catch (...) {
        return false;
    }
}

bool SurfaceExtrudeOp::undo(Document& doc) {
    try {
        // Keep the id: a redo / edit re-creates the body under it.
        if (m_createdBodyId >= 0) doc.removeBody(m_createdBodyId);
        return true;
    } catch (...) {
        return false;
    }
}

std::string SurfaceExtrudeOp::description() const {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Surface extrude %.2f mm", m_distance);
    return buf;
}

void SurfaceExtrudeOp::renderProperties() {
    materializr::lengthField(materializr::tr("Distance"), &m_distance);
}

OperationDiff SurfaceExtrudeOp::captureDiff() const {
    OperationDiff d;
    if (m_createdBodyId >= 0) d.created.push_back(m_createdBodyId);
    return d;
}

std::string SurfaceExtrudeOp::serializeParams() const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "sketch=%d;dist=%.6f", m_sketchId, m_distance);
    return buf;
}

bool SurfaceExtrudeOp::deserializeParams(const std::string& blob) {
    bool any = false;
    size_t pos = 0;
    while (pos < blob.size()) {
        const size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        const std::string key = blob.substr(pos, eq - pos);
        const std::string val = blob.substr(eq + 1, end - eq - 1);
        if (key == "sketch")    { m_sketchId = std::atoi(val.c_str()); any = true; }
        else if (key == "dist") { m_distance = std::atof(val.c_str()); any = true; }
        pos = end + 1;
    }
    return any;
}

bool SurfaceExtrudeOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_sketchId < 0 || state.created.empty()) return false;
    m_createdBodyId = state.created.front();
    return true;
}
