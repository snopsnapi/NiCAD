#include "TrimSurfaceOp.h"
#include "Sketch.h"
#include "SurfaceOps.h"
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <imgui.h>
#include "../i18n.h"

TopoDS_Shape TrimSurfaceOp::toolShape(const Document& doc,
                                      const TopoDS_Shape& target) const {
    try {
        if (m_toolBodyId >= 0) return doc.getBody(m_toolBodyId);
        if (m_toolSketchId < 0) return TopoDS_Shape();
        auto sk = doc.getSketch(m_toolSketchId);
        if (!sk) return TopoDS_Shape();
        const std::vector<TopoDS_Wire> wires = sk->buildCurveWires();
        if (wires.empty()) return TopoDS_Shape();
        // Reach past the target whichever side of the sketch plane it is on.
        Bnd_Box bb;
        BRepBndLib::Add(target, bb);
        double extent = 100.0;
        if (!bb.IsVoid()) {
            const gp_Pnt o = sk->getPlane().Position().Location();
            bb.Add(o);
            extent = std::sqrt(bb.SquareExtent()) + 1.0;
        }
        const gp_Dir n = sk->getPlane().Position().Direction();
        return materializr::surf::curveCutter(wires, gp_Vec(n), extent);
    } catch (...) {
        return TopoDS_Shape();
    }
}

bool TrimSurfaceOp::execute(Document& doc) {
    // No pieces listed = just split the body along the tool.
    if (m_targetId < 0) return false;
    try {
        m_previous = doc.getBody(m_targetId);
        if (m_previous.IsNull()) return false;
        const TopoDS_Shape tool = toolShape(doc, m_previous);
        if (tool.IsNull()) return false;
        const TopoDS_Shape out = materializr::surf::trim(m_previous, tool, m_removeAt);
        if (out.IsNull()) return false;
        doc.updateBody(m_targetId, out);
        return true;
    } catch (...) {
        return false;
    }
}

bool TrimSurfaceOp::undo(Document& doc) {
    if (m_targetId < 0 || m_previous.IsNull()) return false;
    try {
        doc.updateBody(m_targetId, m_previous);
        return true;
    } catch (...) {
        return false;
    }
}

std::string TrimSurfaceOp::description() const {
    return "Trim " + std::to_string(m_removeAt.size()) + " piece(s)";
}

void TrimSurfaceOp::renderProperties() {
    ImGui::Text(materializr::tr("Pieces removed: %d"), static_cast<int>(m_removeAt.size()));
}

OperationDiff TrimSurfaceOp::captureDiff() const {
    OperationDiff d;
    if (m_targetId >= 0 && !m_previous.IsNull())
        d.modifiedBefore.push_back({m_targetId, m_previous});
    return d;
}

std::string TrimSurfaceOp::serializeParams() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "target=%d;toolbody=%d;toolsketch=%d",
                  m_targetId, m_toolBodyId, m_toolSketchId);
    std::string blob = buf;
    if (!m_removeAt.empty()) {
        blob += ";pts=";
        for (size_t i = 0; i < m_removeAt.size(); ++i) {
            std::snprintf(buf, sizeof(buf), "%s%.6f:%.6f:%.6f", i ? "," : "",
                          m_removeAt[i].X(), m_removeAt[i].Y(), m_removeAt[i].Z());
            blob += buf;
        }
    }
    return blob;
}

bool TrimSurfaceOp::deserializeParams(const std::string& blob) {
    bool any = false;
    size_t pos = 0;
    while (pos < blob.size()) {
        const size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        const std::string key = blob.substr(pos, eq - pos);
        const std::string val = blob.substr(eq + 1, end - eq - 1);
        if (key == "target")          { m_targetId = std::atoi(val.c_str()); any = true; }
        else if (key == "toolbody")   { m_toolBodyId = std::atoi(val.c_str()); }
        else if (key == "toolsketch") { m_toolSketchId = std::atoi(val.c_str()); }
        else if (key == "pts") {
            m_removeAt.clear();
            size_t p = 0;
            while (p < val.size() && m_removeAt.size() < 4096) {
                size_t comma = val.find(',', p);
                if (comma == std::string::npos) comma = val.size();
                double x = 0, y = 0, z = 0;
                if (std::sscanf(val.substr(p, comma - p).c_str(), "%lf:%lf:%lf", &x, &y, &z) == 3)
                    m_removeAt.emplace_back(x, y, z);
                p = comma + 1;
            }
        }
        pos = end + 1;
    }
    return any;
}

bool TrimSurfaceOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_targetId < 0) return false;
    for (const auto& [id, shape] : state.modifiedBefore)
        if (id == m_targetId) m_previous = shape;
    return true;
}
