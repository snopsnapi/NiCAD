#pragma once
#include "InteractiveOpController.h"
#include "HistoryEditPreview.h"
#include "../modeling/Form.h"
#include <functional>
#include <set>
#include <string>

class FormOp;

namespace materializr {

// Form primitives (Form workspace): Box / Plane / Cylinder / Sphere cage at
// the origin, sized in the panel, previewed live; Confirm records a FormOp.
class FormPrimitiveController : public InteractiveOpController {
public:
    enum class Kind { Box, Plane, Cylinder, Sphere };
    bool beginPrimitive(const IopContext& ctx, Kind kind);

protected:
    const char* title() const override;
    PreviewModel previewModel() const override { return PreviewModel::LiveOp; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext& ctx) override;
    bool syncLiveOp(Operation& op) override;
    void panelBody(const IopContext& ctx, bool& changed) override;

private:
    std::string cageText() const;
    Kind m_kind = Kind::Box;
    float m_size = 40.0f;     // box/plane edge, sphere/cylinder diameter
    float m_height = 40.0f;   // cylinder
    int m_segs = 2;
};

// Edit Form (Fusion's Edit Form): re-opens a FormOp step. Tap vertices,
// edges or faces of the cage (taps add/remove; a tap on nothing clears);
// drag the manipulator - arrows move along an axis, squares in a plane, rings
// rotate about an axis, the centre disc scales - in World axes or Local ones
// (Z along the selection's normal, i.e. at 90 degrees to the surface there).
// Symmetry X/Y/Z mirrors every move and edit across a plane through the
// form's centre. Panel: World/Local, Symmetry, Crease, Uncrease, Subdivide,
// Insert Edge, Extrude, Delete. The preview is
// the history replay from the step; Cancel puts the cage back.
class EditFormController : public InteractiveOpController {
public:
    bool beginEdit(const IopContext& ctx, int historyIndex);
    std::function<void(int step, const std::string& before)> onCommitted;

    void update(const IopContext& ctx) override;
    void commit(const IopContext& ctx) override;
    void cancel(const IopContext& ctx) override;

protected:
    const char* title() const override { return "Edit Form"; }
    PreviewModel previewModel() const override { return PreviewModel::HistoryEdit; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext&) override { return nullptr; }
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override;
    float panelWidth() const override { return 280.0f; }
    bool wantsViewportInput() const override { return true; }
    void onViewportInput(const IopViewport& vp, const IopContext& ctx) override;
    void drawOverlay(const IopOverlay& ov) const override;

private:
    ::FormOp* op(const IopContext& ctx) const;
    void writeLive(const IopContext& ctx);   // cage -> op -> form table (no replay)
    std::vector<int> selectedVerts() const;
    // The manipulator frame for the current selection.
    bool frame(glm::vec3& o, glm::vec3 ax[3]) const;
    float handleLength() const;
    int pickHandle(const IopViewport& vp) const;
    void pickElement(const IopViewport& vp);
    void clearSelection() { m_selV.clear(); m_selE.clear(); m_selF.clear(); }

    int m_step = -1;
    std::string m_before;
    form::Cage m_cage, m_dragStart;
    std::set<int> m_selV, m_selF;
    std::set<form::Edge> m_selE;
    bool m_local = false;
    HistoryEditPreview m_preview;
    glm::vec3 m_camPos{0.0f};

    // gesture
    bool m_pressed = false, m_moved = false;
    int m_handle = -1;         // 0..2 axis, 3..5 plane (normal axis), 6 scale, 7..9 ring
    float m_ringStart = 0.0f;  // ring angle at press (radians)
    glm::vec2 m_pressAt{0.0f};
    glm::vec3 m_frameO{0.0f}, m_frameAx[3];
    float m_axisTravel = 0.0f;
    glm::vec3 m_planeStart{0.0f};
};

} // namespace materializr
