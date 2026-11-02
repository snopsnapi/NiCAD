#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <string>
#include <vector>

// Trim (fork, Surface workspace "scissors"): split a body by a tool - another
// body, or a sketch's curves pushed through along the sketch normal - and
// discard the pieces the user tapped. A piece is remembered by a world point
// on it, so the step replays and reloads (surf::trim picks the face nearest
// each point after the split). The tool is left untouched. With no pieces
// listed the step is a plain split along the tool.
class TrimSurfaceOp : public Operation {
public:
    void setTarget(int bodyId) { m_targetId = bodyId; }
    void setToolBody(int bodyId) { m_toolBodyId = bodyId; m_toolSketchId = -1; }
    void setToolSketch(int sketchId) { m_toolSketchId = sketchId; m_toolBodyId = -1; }
    void setRemovePoints(const std::vector<gp_Pnt>& pts) { m_removeAt = pts; }
    int targetId() const { return m_targetId; }

    // The cutting tool as a shape for `target` (a sketch's curves become a
    // cutter reaching past the target's extent). Null when unresolved.
    TopoDS_Shape toolShape(const Document& doc, const TopoDS_Shape& target) const;

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Trim"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "trim_surface"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {m_targetId}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    int m_targetId = -1;
    int m_toolBodyId = -1;
    int m_toolSketchId = -1;
    std::vector<gp_Pnt> m_removeAt;
    TopoDS_Shape m_previous;
};
