#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

// Surface workspace edits (fork). Both persist their picked sub-shapes as
// ordinal indices into the input body's canonical map (SubShapeIndex), like
// Remove Feature, so they reload as real editable steps.

// Extend: push free boundary edges of a surface body outward by a length,
// continuing the faces' own surfaces (surf::extendEdges). Modifies the body.
class ExtendSurfaceOp : public Operation {
public:
    void setBody(int id) { m_bodyId = id; }
    void setEdges(const std::vector<TopoDS_Edge>& e) { m_edges = e; }
    void setLength(double mm) { m_length = mm; }
    double length() const { return m_length; }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Extend"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "extend_surface"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {m_bodyId}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    int m_bodyId = -1;
    std::vector<TopoDS_Edge> m_edges;
    std::vector<int> m_edgeIndices;
    double m_length = 5.0;
    TopoDS_Shape m_previous;
};

// Offset: faces of a body moved along their normals by a distance into a NEW
// surface body (surf::offsetFaces). The source body is left as it is.
class OffsetSurfaceOp : public Operation {
public:
    void setSource(int bodyId) { m_srcBodyId = bodyId; }
    void setFaces(const std::vector<TopoDS_Face>& f) { m_faces = f; }
    void setDistance(double mm) { m_distance = mm; }
    double distance() const { return m_distance; }
    int createdBodyId() const { return m_createdBodyId; }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Offset Surface"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "offset_surface"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    int m_srcBodyId = -1;
    std::vector<TopoDS_Face> m_faces;
    std::vector<int> m_faceIndices;   // into the source body at execute time
    double m_distance = 2.0;
    int m_createdBodyId = -1;
};
