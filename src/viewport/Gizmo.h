#pragma once
#include "gl_common.h"
#include <glm/glm.hpp>

namespace materializr {

class Camera;

enum class GizmoMode { Translate, Rotate, Scale };
// XY / YZ / XZ: the plane handles of the combined Move gizmo (translate
// within that world plane).
enum class GizmoAxis { None, X, Y, Z, XY, YZ, XZ };

struct GizmoResult {
    bool changed = false;
    GizmoMode mode = GizmoMode::Translate;
    GizmoAxis activeAxis = GizmoAxis::None;
    glm::vec3 delta{0};
};

class Gizmo {
public:
    Gizmo();
    ~Gizmo();

    bool initialize();

    // setMode also leaves the combined Move gizmo (see setCombinedMove).
    void setMode(GizmoMode mode);
    GizmoMode getMode() const;
    // Fork: Fusion-style Move - arrows, rotation rings and plane squares all
    // at once. Grabbing a handle switches the effective mode (getMode) to
    // Translate or Rotate; it stays so until the next grab, so the app's
    // commit (which reads getMode after release) applies the right transform.
    void setCombinedMove();
    bool isCombined() const { return m_combined; }
    // Combined mode only: set the effective mode without leaving combined
    // (typed values in the Move panel).
    void setEffectiveMode(GizmoMode mode) { m_mode = mode; }

    void setPosition(glm::vec3 pos);
    void setVisible(bool vis);
    bool isVisible() const;

    void render(const glm::mat4& view, const glm::mat4& projection);

    // Draw ONE translate-arrow (the same cone-headed mesh the move gizmo uses)
    // pointing along an arbitrary world direction, in a custom colour, at a
    // given position. Used by Move Face to show two yellow in-plane arrows that
    // look like the familiar gizmo (minus the out-of-plane axis). Independent of
    // m_position / m_visible / m_mode.
    void renderArrowAlong(const glm::mat4& view, const glm::mat4& projection,
                          glm::vec3 position, glm::vec3 dir, glm::vec3 color);
    // A rotation ring at `position` whose axis is `axisDir` (the ring lies in
    // the plane perpendicular to it). Used for the face-tilt gizmo.
    void renderRingAbout(const glm::mat4& view, const glm::mat4& projection,
                         glm::vec3 position, glm::vec3 axisDir, glm::vec3 color);
    // A scale handle (shaft + cube) at `position` pointing along `dir` - the
    // regular scale-gizmo look, for the face scale gizmo.
    void renderCubeAlong(const glm::mat4& view, const glm::mat4& projection,
                         glm::vec3 position, glm::vec3 dir, glm::vec3 color);

    GizmoResult handleInput(float mouseX, float mouseY,
                            float vpWidth, float vpHeight,
                            bool mouseDown, bool mouseJustPressed,
                            const Camera& camera);

    // Forcibly drop any in-flight drag (called when Escape cancels a move so the
    // gizmo doesn't continue dragging once the mouse keeps moving).
    void cancelDrag();

private:
    GizmoMode m_mode = GizmoMode::Translate;
    // Fork: the Fusion-style combined gizmo is the default (a body picked by
    // double tap gets arrows, rotation arcs and plane squares too).
    bool m_combined = true;
    glm::vec3 m_position{0};
    bool m_visible = false;

    // What the user is currently dragging
    GizmoMode m_draggingMode = GizmoMode::Translate;
    GizmoAxis m_draggingAxis = GizmoAxis::None;
    GizmoAxis m_hoveredAxis = GizmoAxis::None;
    GizmoMode m_hoveredMode = GizmoMode::Translate;
    glm::vec3 m_lastDragPos{0};
    float m_lastDragAngle = 0.0f; // last ring angle (deg), for rotate drags

    // GL resources
    unsigned int m_program = 0;
    int m_locMVP = -1;   // (m_locAlpha below)
    int m_locColor = -1;

    // Meshes: arrows (translate), rings (rotate), cubes (scale)
    unsigned int m_arrowVao = 0, m_arrowVbo = 0;
    int m_arrowVertCount = 0;

    unsigned int m_ringVao = 0, m_ringVbo = 0;
    int m_ringVertCount = 0;

    unsigned int m_cubeVao = 0, m_cubeVbo = 0;
    int m_cubeVertCount = 0;

    unsigned int m_planeVao = 0, m_planeVbo = 0;
    int m_planeVertCount = 0;

    // Combined Move: quarter-arc rotation handles (Fusion style) instead of
    // the 270-degree rings.
    unsigned int m_arcVao = 0, m_arcVbo = 0;
    int m_arcVertCount = 0;
    int m_locAlpha = -1;
    // Viewport height in input units (from handleInput), so the drawn and
    // the picked gizmo share one screen size.
    float m_viewH = 0.0f;
    // World units per local gizmo unit at m_position: a constant size in dp
    // on screen, whatever the zoom.
    float handleScale(const glm::mat4& view, const glm::mat4& proj, float vpH) const;
    void buildArcMesh();

    void buildArrowMesh();
    void buildRingMesh();
    void buildCubeMesh();
    void buildPlaneMesh();
    // Combined mode: move within the handle's plane (XY/YZ/XZ). Ray-plane hit
    // through the gizmo origin; false when the view is edge-on to the plane.
    bool projectOnPlane(float mx, float my, float vpW, float vpH,
                        const Camera& camera, GizmoAxis plane, glm::vec3& out);
    bool compileShader(unsigned int& shader, unsigned int type, const char* source);

    struct PickResult { GizmoAxis axis; GizmoMode mode; float dist; };
    PickResult pickNearest(float mx, float my, float vpW, float vpH, const Camera& camera);
    glm::vec3 projectOnAxis(float mx, float my, float vpW, float vpH,
                            const Camera& camera, GizmoAxis axis);
    // Angle (degrees) of the mouse around the rotation ring for `axis`, measured
    // in the plane through the gizmo perpendicular to that axis. Returns false if
    // the view ray is parallel to the plane.
    bool ringAngle(float mx, float my, float vpW, float vpH,
                   const Camera& camera, GizmoAxis axis, float& outDeg);
    float screenDistToSegment(glm::vec2 mouse, glm::vec2 a, glm::vec2 b);
    glm::vec2 worldToNDC(glm::vec3 pos, const glm::mat4& vp);
};

} // namespace materializr
