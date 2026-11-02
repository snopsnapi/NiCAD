#include "SurfaceEditOps.h"
#include "SubShapeIndex.h"
#include "SurfaceOps.h"
#include "../ui/LengthField.h"
#include <TopoDS.hxx>
#include <cstdio>
#include <cstdlib>
#include <imgui.h>
#include "../i18n.h"

namespace {

// key=value;key=value walker shared by both ops.
template <typename F>
bool forEachParam(const std::string& blob, F&& f) {
    bool any = false;
    size_t pos = 0;
    while (pos < blob.size()) {
        const size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        any |= f(blob.substr(pos, eq - pos), blob.substr(eq + 1, end - eq - 1));
        pos = end + 1;
    }
    return any;
}

} // namespace

// ── Extend ──────────────────────────────────────────────────────────────────

bool ExtendSurfaceOp::execute(Document& doc) {
    if (m_bodyId < 0 || m_edges.empty() || !(m_length > 1e-6)) return false;
    try {
        m_previous = doc.getBody(m_bodyId);
        if (m_previous.IsNull()) return false;
        if (m_edgeIndices.empty()) {
            std::vector<TopoDS_Shape> e(m_edges.begin(), m_edges.end());
            m_edgeIndices = SubShapeIndex::parse(
                SubShapeIndex::serialize(m_previous, e, TopAbs_EDGE));
        }
        const TopoDS_Shape out =
            materializr::surf::extendEdges(m_previous, m_edges, m_length);
        if (out.IsNull()) return false;
        doc.updateBody(m_bodyId, out);
        return true;
    } catch (...) {
        return false;
    }
}

bool ExtendSurfaceOp::undo(Document& doc) {
    if (m_bodyId < 0 || m_previous.IsNull()) return false;
    try { doc.updateBody(m_bodyId, m_previous); return true; } catch (...) { return false; }
}

std::string ExtendSurfaceOp::description() const {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Extend %zu edge(s) %.2f mm", m_edges.size(), m_length);
    return buf;
}

void ExtendSurfaceOp::renderProperties() {
    materializr::lengthField(materializr::tr("Distance"), &m_length);
}

OperationDiff ExtendSurfaceOp::captureDiff() const {
    OperationDiff d;
    if (m_bodyId >= 0 && !m_previous.IsNull())
        d.modifiedBefore.push_back({m_bodyId, m_previous});
    return d;
}

std::string ExtendSurfaceOp::serializeParams() const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "body=%d;len=%.6f", m_bodyId, m_length);
    std::string blob = buf;
    if (!m_previous.IsNull() && !m_edges.empty()) {
        std::vector<TopoDS_Shape> e(m_edges.begin(), m_edges.end());
        const std::string idx = SubShapeIndex::serialize(m_previous, e, TopAbs_EDGE);
        if (!idx.empty()) blob += ";edges=" + idx;
    }
    return blob;
}

bool ExtendSurfaceOp::deserializeParams(const std::string& blob) {
    return forEachParam(blob, [&](const std::string& k, const std::string& v) {
        if (k == "body")  { m_bodyId = std::atoi(v.c_str()); return true; }
        if (k == "len")   { m_length = std::atof(v.c_str()); return true; }
        if (k == "edges") { m_edgeIndices = SubShapeIndex::parse(v); return true; }
        return false;
    });
}

bool ExtendSurfaceOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_bodyId < 0 || m_edgeIndices.empty()) return false;
    m_previous.Nullify();
    for (const auto& [id, shp] : state.modifiedBefore)
        if (id == m_bodyId) { m_previous = shp; break; }
    if (m_previous.IsNull()) return false;
    std::vector<TopoDS_Shape> resolved;
    if (!SubShapeIndex::resolveAll(m_previous, m_edgeIndices, TopAbs_EDGE, resolved))
        return false;
    m_edges.clear();
    for (const auto& e : resolved) m_edges.push_back(TopoDS::Edge(e));
    return true;
}

// ── Offset ──────────────────────────────────────────────────────────────────

bool OffsetSurfaceOp::execute(Document& doc) {
    if (m_srcBodyId < 0 || std::abs(m_distance) < 1e-6) return false;
    try {
        const TopoDS_Shape src = doc.getBody(m_srcBodyId);
        if (src.IsNull()) return false;
        if (m_faces.empty() && !m_faceIndices.empty()) {
            // Reloaded: re-resolve against the source as it is at this step.
            std::vector<TopoDS_Shape> resolved;
            if (!SubShapeIndex::resolveAll(src, m_faceIndices, TopAbs_FACE, resolved))
                return false;
            for (const auto& f : resolved) m_faces.push_back(TopoDS::Face(f));
        }
        if (m_faces.empty()) return false;
        if (m_faceIndices.empty()) {
            std::vector<TopoDS_Shape> f(m_faces.begin(), m_faces.end());
            m_faceIndices = SubShapeIndex::parse(
                SubShapeIndex::serialize(src, f, TopAbs_FACE));
        }
        const TopoDS_Shape out = materializr::surf::offsetFaces(m_faces, m_distance);
        if (out.IsNull()) return false;
        doc.addOrPutBody(m_createdBodyId, out, "Offset");
        return true;
    } catch (...) {
        return false;
    }
}

bool OffsetSurfaceOp::undo(Document& doc) {
    try {
        if (m_createdBodyId >= 0) doc.removeBody(m_createdBodyId);   // id kept for redo
        return true;
    } catch (...) {
        return false;
    }
}

std::string OffsetSurfaceOp::description() const {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Offset %zu face(s) %.2f mm", m_faces.size(), m_distance);
    return buf;
}

void OffsetSurfaceOp::renderProperties() {
    materializr::lengthField(materializr::tr("Distance"), &m_distance);
}

OperationDiff OffsetSurfaceOp::captureDiff() const {
    OperationDiff d;
    if (m_createdBodyId >= 0) d.created.push_back(m_createdBodyId);
    return d;
}

std::string OffsetSurfaceOp::serializeParams() const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "src=%d;dist=%.6f", m_srcBodyId, m_distance);
    std::string blob = buf;
    if (!m_faceIndices.empty()) {
        blob += ";faces=";
        for (size_t i = 0; i < m_faceIndices.size(); ++i)
            blob += (i ? "," : "") + std::to_string(m_faceIndices[i]);
    }
    return blob;
}

bool OffsetSurfaceOp::deserializeParams(const std::string& blob) {
    return forEachParam(blob, [&](const std::string& k, const std::string& v) {
        if (k == "src")   { m_srcBodyId = std::atoi(v.c_str()); return true; }
        if (k == "dist")  { m_distance = std::atof(v.c_str()); return true; }
        if (k == "faces") { m_faceIndices = SubShapeIndex::parse(v); return true; }
        return false;
    });
}

bool OffsetSurfaceOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_srcBodyId < 0 || m_faceIndices.empty() || state.created.empty()) return false;
    m_createdBodyId = state.created.front();
    m_faces.clear();   // re-resolved on the next execute
    return true;
}
