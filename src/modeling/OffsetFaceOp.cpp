#include "OffsetFaceOp.h"
#include "OffsetFace.h"
#include "SubShapeIndex.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <TopoDS.hxx>
#include <imgui.h>
#include "../i18n.h"

OffsetFaceOp::OffsetFaceOp() = default;

void OffsetFaceOp::setBody(int id) { m_bodyId = id; }
void OffsetFaceOp::addFace(const TopoDS_Face& face) { m_faces.Append(face); }
void OffsetFaceOp::clearFaces() { m_faces.Clear(); }
void OffsetFaceOp::setDistance(double distance) { m_distance = distance; }
void OffsetFaceOp::setRemoveBlends(bool remove) { m_removeBlends = remove; }

bool OffsetFaceOp::execute(Document& doc) {
    if (m_bodyId < 0 || m_faces.IsEmpty()) return false;

    try {
        m_previousShape = doc.getBody(m_bodyId);
        if (m_previousShape.IsNull()) return false;

        std::vector<TopoDS_Face> faces;
        for (const TopoDS_Shape& f : m_faces) faces.push_back(TopoDS::Face(f));

        materializr::offsetface::Options opt;
        opt.removeAdjacentBlends = m_removeBlends;
        materializr::offsetface::Result r;
        if (!materializr::offsetface::offsetFaces(m_previousShape, faces, m_distance, opt, r)) {
            std::fprintf(stderr, "[OffsetFace] %s\n", r.error.c_str());
            return false;
        }

        doc.updateBody(m_bodyId, r.shape);
        return true;
    } catch (...) {
        return false;
    }
}

bool OffsetFaceOp::undo(Document& doc) {
    if (m_bodyId < 0 || m_previousShape.IsNull()) return false;
    try {
        doc.updateBody(m_bodyId, m_previousShape);
        return true;
    } catch (...) {
        return false;
    }
}

std::string OffsetFaceOp::description() const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "Offset %d face(s) by %.1f mm",
                  static_cast<int>(m_faces.Size()), m_distance);
    return buf;
}

void OffsetFaceOp::renderProperties() {
    ImGui::Text("%s", materializr::tr("Offset Face"));
    ImGui::Separator();
    ImGui::Text(materializr::tr("Faces offset: %d"), static_cast<int>(m_faces.Size()));
    ImGui::Text(materializr::tr("Distance: %.1f mm"), m_distance);
    ImGui::Text("%s", materializr::tr(m_removeBlends
                                          ? "Remove adjacent blends: yes"
                                          : "Remove adjacent blends: no"));
}

std::string OffsetFaceOp::serializeParams() const {
    // Offset faces persist as ordinal indices into the INPUT shape's canonical
    // face map (see SubShapeIndex.h) - same scheme as Shell/Fillet/Chamfer.
    std::string blob;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "body=%d;dist=%.6f;blends=%d", m_bodyId,
                  m_distance, m_removeBlends ? 1 : 0);
    blob += buf;
    if (!m_previousShape.IsNull() && !m_faces.IsEmpty()) {
        std::vector<TopoDS_Shape> faces;
        for (const TopoDS_Shape& f : m_faces) faces.push_back(f);
        std::string idx = SubShapeIndex::serialize(m_previousShape, faces,
                                                   TopAbs_FACE);
        if (!idx.empty()) blob += ";faces=" + idx;
    }
    return blob;
}

bool OffsetFaceOp::deserializeParams(const std::string& blob) {
    bool any = false;
    size_t pos = 0;
    while (pos < blob.size()) {
        size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        std::string key = blob.substr(pos, eq - pos);
        std::string val = blob.substr(eq + 1, end - eq - 1);
        if      (key == "body")   { m_bodyId = std::atoi(val.c_str()); any = true; }
        else if (key == "faces")  { m_faceIndices = SubShapeIndex::parse(val); any = true; }
        else if (key == "dist")   { m_distance = std::atof(val.c_str()); any = true; }
        else if (key == "blends") { m_removeBlends = std::atoi(val.c_str()) != 0; any = true; }
        pos = end + 1;
    }
    return any;
}

bool OffsetFaceOp::rehydrateFromReload(const ReloadState& state, Document& /*doc*/) {
    if (m_bodyId < 0) return false;

    m_previousShape.Nullify();
    for (const auto& [id, shp] : state.modifiedBefore)
        if (id == m_bodyId) { m_previousShape = shp; break; }
    if (m_previousShape.IsNull()) return false;

    // Re-resolve the offset faces against the reloaded input shape. No saved
    // indices means nothing to offset - decline so it falls back to a baked op
    // rather than silently doing nothing.
    m_faces.Clear();
    if (m_faceIndices.empty()) return false;
    std::vector<TopoDS_Shape> resolved;
    if (!SubShapeIndex::resolveAll(m_previousShape, m_faceIndices,
                                   TopAbs_FACE, resolved)) {
        return false;
    }
    for (const auto& f : resolved) m_faces.Append(f);
    return true;
}

OperationDiff OffsetFaceOp::captureDiff() const {
    OperationDiff d;
    if (m_bodyId >= 0 && !m_previousShape.IsNull())
        d.modifiedBefore.push_back({m_bodyId, m_previousShape});
    return d;
}
