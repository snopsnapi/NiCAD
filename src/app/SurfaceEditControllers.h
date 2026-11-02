#pragma once
#include "InteractiveOpController.h"
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

namespace materializr {

// Extend (Surface workspace): the picked free edges of one body pushed
// outward by a length; live preview on the body (snapshot model).
class ExtendSurfaceController : public InteractiveOpController {
protected:
    const char* title() const override { return "Extend"; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext& ctx) override;
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override { m_edges.clear(); }

private:
    std::vector<TopoDS_Edge> m_edges;
    float m_length = 5.0f;   // remembered between uses
};

// Offset (Surface workspace): the picked faces of one body copied along
// their normals into a new surface body; live preview (live-op model).
class OffsetSurfaceController : public InteractiveOpController {
protected:
    const char* title() const override { return "Offset"; }
    PreviewModel previewModel() const override { return PreviewModel::LiveOp; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext& ctx) override;
    bool syncLiveOp(Operation& op) override;
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override { m_faces.clear(); m_srcBody = -1; }

private:
    std::vector<TopoDS_Face> m_faces;
    int m_srcBody = -1;
    float m_distance = 2.0f;   // remembered between uses; sign = side
};

} // namespace materializr
