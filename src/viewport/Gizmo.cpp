#include "gl_common.h"
#include "Gizmo.h"
#include "Camera.h"
#include "../touch_mode.h"
#include "../ui_scale.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstdio>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace materializr {

static const char* s_vertSource = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
uniform mat4 u_mvp;
void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
}
)";

static const char* s_fragSource = R"(
#version 330 core
uniform vec3 u_color;
uniform float u_alpha;
out vec4 fragColor;
void main() {
    fragColor = vec4(u_color, u_alpha);
}
)";

static constexpr float kAxisLength = 1.0f;
static constexpr float kConeBaseRadius = 0.08f;
static constexpr float kConeHeight = 0.2f;
static constexpr float kShaftRadius = 0.025f;
static constexpr float kRingRadius = 0.75f;
static constexpr float kRingTubeRadius = 0.025f;
static constexpr float kCubeSize = 0.08f;
static constexpr float kCubeOffset = 1.05f;
static constexpr int kSegments = 16;
static constexpr int kRingSegments = 48;
static constexpr float kPickThreshold = 0.04f;
// Combined Move: plane squares between two axes, [kPlaneLo, kPlaneHi] along
// each (inside the rotation rings, clear of the arrow shafts).
static constexpr float kPlaneLo = 0.18f;
static constexpr float kPlaneHi = 0.36f;
// Combined Move: rotation arcs - a quarter circle between the two other
// axes, trimmed clear of the arrow shafts at both ends.
static constexpr float kArcRadius = 0.62f;
static constexpr float kArcTube = 0.028f;
static constexpr float kArcFrom = 0.20f;   // radians
static constexpr float kArcTo = 1.37f;
// On-screen size of one local gizmo unit (the arrow is 1.2 units long).
static float gizmoUnitDp() { return materializr::touchMode() ? 58.0f : 48.0f; }
// The combined gizmo's colours (Fusion's axis hues) and the grabbed handle's.
static const glm::vec3 kAxisCol[3] = {{0.88f, 0.22f, 0.22f}, {0.30f, 0.76f, 0.22f},
                                      {0.22f, 0.46f, 0.96f}};
static const glm::vec3 kActiveCol{1.0f, 0.80f, 0.10f};
// Arc frame for the rotation about `axis`: columns u, v, n with u x v = n
// (the same u / v ringAngle measures the angle in).
static glm::mat4 arcFrame(GizmoAxis axis) {
    glm::vec3 u, v, n;
    if (axis == GizmoAxis::X)      { n = {1,0,0}; u = {0,1,0}; v = {0,0,1}; }
    else if (axis == GizmoAxis::Y) { n = {0,1,0}; u = {0,0,1}; v = {1,0,0}; }
    else                           { n = {0,0,1}; u = {1,0,0}; v = {0,1,0}; }
    glm::mat4 m(1.0f);
    m[0] = glm::vec4(u, 0.0f);
    m[1] = glm::vec4(v, 0.0f);
    m[2] = glm::vec4(n, 0.0f);
    return m;
}

// Local XY quad -> world plane: XY (normal Z), YZ (normal X), XZ (normal Y).
static glm::mat4 planeRotation(GizmoAxis a) {
    glm::mat4 m(1.0f);
    if (a == GizmoAxis::YZ) {
        m[0] = glm::vec4(0, 1, 0, 0); m[1] = glm::vec4(0, 0, 1, 0); m[2] = glm::vec4(1, 0, 0, 0);
    } else if (a == GizmoAxis::XZ) {
        m[0] = glm::vec4(0, 0, 1, 0); m[1] = glm::vec4(1, 0, 0, 0); m[2] = glm::vec4(0, 1, 0, 0);
    }
    return m;
}
static glm::vec3 planeNormal(GizmoAxis a) {
    if (a == GizmoAxis::YZ) return glm::vec3(1, 0, 0);
    if (a == GizmoAxis::XZ) return glm::vec3(0, 1, 0);
    return glm::vec3(0, 0, 1);
}
static bool isPlaneAxis(GizmoAxis a) {
    return a == GizmoAxis::XY || a == GizmoAxis::YZ || a == GizmoAxis::XZ;
}

// World-size factor the gizmo scales by (× camera distance). A fingertip is far
// less precise than a cursor, so grow the whole widget on touch - the visual
// arrows/rings AND their hit-tests both derive from this, so they enlarge
// together (same approach as the ViewCube).
static float gizmoScale() {
    return materializr::touchMode() ? 0.225f : 0.15f;
}

Gizmo::Gizmo() {}

Gizmo::~Gizmo() {
    if (m_program) glDeleteProgram(m_program);
    if (m_arrowVao) glDeleteVertexArrays(1, &m_arrowVao);
    if (m_arrowVbo) glDeleteBuffers(1, &m_arrowVbo);
    if (m_ringVao) glDeleteVertexArrays(1, &m_ringVao);
    if (m_ringVbo) glDeleteBuffers(1, &m_ringVbo);
    if (m_cubeVao) glDeleteVertexArrays(1, &m_cubeVao);
    if (m_cubeVbo) glDeleteBuffers(1, &m_cubeVbo);
    if (m_planeVao) glDeleteVertexArrays(1, &m_planeVao);
    if (m_planeVbo) glDeleteBuffers(1, &m_planeVbo);
    if (m_arcVao) glDeleteVertexArrays(1, &m_arcVao);
    if (m_arcVbo) glDeleteBuffers(1, &m_arcVbo);
}

bool Gizmo::initialize() {
    unsigned int vert = 0, frag = 0;
    if (!compileShader(vert, GL_VERTEX_SHADER, s_vertSource)) return false;
    if (!compileShader(frag, GL_FRAGMENT_SHADER, s_fragSource)) {
        glDeleteShader(vert);
        return false;
    }
    m_program = glCreateProgram();
    glAttachShader(m_program, vert);
    glAttachShader(m_program, frag);
    glLinkProgram(m_program);
    glDeleteShader(vert);
    glDeleteShader(frag);

    int success = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(m_program, 512, nullptr, log);
        std::fprintf(stderr, "Gizmo link error: %s\n", log);
        return false;
    }
    m_locMVP = glGetUniformLocation(m_program, "u_mvp");
    m_locColor = glGetUniformLocation(m_program, "u_color");
    m_locAlpha = glGetUniformLocation(m_program, "u_alpha");

    glGenVertexArrays(1, &m_arrowVao);
    glGenBuffers(1, &m_arrowVbo);
    glGenVertexArrays(1, &m_ringVao);
    glGenBuffers(1, &m_ringVbo);
    glGenVertexArrays(1, &m_cubeVao);
    glGenBuffers(1, &m_cubeVbo);
    glGenVertexArrays(1, &m_planeVao);
    glGenBuffers(1, &m_planeVbo);
    // Fork: the combined gizmo's rotation arcs. Never generated before, so
    // the arcs drew from VAO 0 (client arrays, null pointer) and crashed in
    // the GL driver the moment the combined gizmo was shown.
    glGenVertexArrays(1, &m_arcVao);
    glGenBuffers(1, &m_arcVbo);

    buildArrowMesh();
    buildRingMesh();
    buildCubeMesh();
    buildPlaneMesh();
    buildArcMesh();
    return true;
}

static void uploadMesh(unsigned int vao, unsigned int vbo,
                       const std::vector<float>& verts, int& vertCount) {
    vertCount = static_cast<int>(verts.size() / 3);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Gizmo::buildArrowMesh() {
    std::vector<float> verts;
    auto addTri = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        verts.insert(verts.end(), {a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z});
    };

    // Shaft cylinder along +Z
    for (int i = 0; i < kSegments; ++i) {
        float a1 = 2.0f * (float)M_PI * i / kSegments;
        float a2 = 2.0f * (float)M_PI * (i + 1) / kSegments;
        float c1 = std::cos(a1), s1 = std::sin(a1);
        float c2 = std::cos(a2), s2 = std::sin(a2);
        glm::vec3 p0(kShaftRadius*c1, kShaftRadius*s1, 0.0f);
        glm::vec3 p1(kShaftRadius*c2, kShaftRadius*s2, 0.0f);
        glm::vec3 p2(kShaftRadius*c2, kShaftRadius*s2, kAxisLength);
        glm::vec3 p3(kShaftRadius*c1, kShaftRadius*s1, kAxisLength);
        addTri(p0, p1, p2);
        addTri(p0, p2, p3);
    }

    // Cone tip
    glm::vec3 tip(0, 0, kAxisLength + kConeHeight);
    for (int i = 0; i < kSegments; ++i) {
        float a1 = 2.0f * (float)M_PI * i / kSegments;
        float a2 = 2.0f * (float)M_PI * (i + 1) / kSegments;
        glm::vec3 b1(kConeBaseRadius*std::cos(a1), kConeBaseRadius*std::sin(a1), kAxisLength);
        glm::vec3 b2(kConeBaseRadius*std::cos(a2), kConeBaseRadius*std::sin(a2), kAxisLength);
        addTri(b1, b2, tip);
        addTri(glm::vec3(0,0,kAxisLength), b2, b1);
    }

    uploadMesh(m_arrowVao, m_arrowVbo, verts, m_arrowVertCount);
}

void Gizmo::buildRingMesh() {
    // A torus arc (270 degrees) in the XY plane centered at origin
    std::vector<float> verts;
    auto addTri = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        verts.insert(verts.end(), {a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z});
    };

    float arcAngle = 2.0f * (float)M_PI * 0.75f; // 270 degrees
    int tubeSegs = 8;

    for (int i = 0; i < kRingSegments; ++i) {
        float theta1 = arcAngle * i / kRingSegments;
        float theta2 = arcAngle * (i + 1) / kRingSegments;

        for (int j = 0; j < tubeSegs; ++j) {
            float phi1 = 2.0f * (float)M_PI * j / tubeSegs;
            float phi2 = 2.0f * (float)M_PI * (j + 1) / tubeSegs;

            auto torusPoint = [&](float theta, float phi) -> glm::vec3 {
                float r = kRingRadius + kRingTubeRadius * std::cos(phi);
                return glm::vec3(r * std::cos(theta), r * std::sin(theta),
                                 kRingTubeRadius * std::sin(phi));
            };

            glm::vec3 p00 = torusPoint(theta1, phi1);
            glm::vec3 p10 = torusPoint(theta2, phi1);
            glm::vec3 p11 = torusPoint(theta2, phi2);
            glm::vec3 p01 = torusPoint(theta1, phi2);

            addTri(p00, p10, p11);
            addTri(p00, p11, p01);
        }
    }

    uploadMesh(m_ringVao, m_ringVbo, verts, m_ringVertCount);
}

void Gizmo::buildCubeMesh() {
    // A shaft along +Z with a small cube at the end (mirrors the move arrow's
    // shaft+cone, so scale handles read as bars with a knob).
    std::vector<float> verts;
    auto addTri = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        verts.insert(verts.end(), {a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z});
    };

    // Shaft cylinder from origin to the cube.
    for (int i = 0; i < kSegments; ++i) {
        float a1 = 2.0f * (float)M_PI * i / kSegments;
        float a2 = 2.0f * (float)M_PI * (i + 1) / kSegments;
        float c1 = std::cos(a1), s1 = std::sin(a1);
        float c2 = std::cos(a2), s2 = std::sin(a2);
        glm::vec3 p0(kShaftRadius*c1, kShaftRadius*s1, 0.0f);
        glm::vec3 p1(kShaftRadius*c2, kShaftRadius*s2, 0.0f);
        glm::vec3 p2(kShaftRadius*c2, kShaftRadius*s2, kCubeOffset);
        glm::vec3 p3(kShaftRadius*c1, kShaftRadius*s1, kCubeOffset);
        addTri(p0, p1, p2);
        addTri(p0, p2, p3);
    }

    float s = kCubeSize;
    glm::vec3 o(0, 0, kCubeOffset);
    glm::vec3 corners[8] = {
        o + glm::vec3(-s,-s,-s), o + glm::vec3(+s,-s,-s),
        o + glm::vec3(+s,+s,-s), o + glm::vec3(-s,+s,-s),
        o + glm::vec3(-s,-s,+s), o + glm::vec3(+s,-s,+s),
        o + glm::vec3(+s,+s,+s), o + glm::vec3(-s,+s,+s),
    };

    // 6 faces, 2 triangles each
    int faces[6][4] = {
        {0,1,2,3}, {5,4,7,6}, // front, back
        {4,0,3,7}, {1,5,6,2}, // left, right
        {3,2,6,7}, {4,5,1,0}, // top, bottom
    };
    for (auto& f : faces) {
        addTri(corners[f[0]], corners[f[1]], corners[f[2]]);
        addTri(corners[f[0]], corners[f[2]], corners[f[3]]);
    }

    uploadMesh(m_cubeVao, m_cubeVbo, verts, m_cubeVertCount);
}

void Gizmo::buildPlaneMesh() {
    // A flat square in the local XY plane between kPlaneLo and kPlaneHi on
    // both axes; planeRotation() turns it into the XY / YZ / XZ handle.
    const float a = kPlaneLo, b = kPlaneHi;
    std::vector<float> verts = {
        a, a, 0.0f,  b, a, 0.0f,  b, b, 0.0f,
        a, a, 0.0f,  b, b, 0.0f,  a, b, 0.0f,
    };
    uploadMesh(m_planeVao, m_planeVbo, verts, m_planeVertCount);
}

void Gizmo::buildArcMesh() {
    // Quarter torus arc in the local XY plane (kArcFrom..kArcTo); arcFrame()
    // turns it into the arc about each axis.
    std::vector<float> verts;
    auto addTri = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        verts.insert(verts.end(), {a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z});
    };
    const int segs = 24, tubeSegs = 8;
    auto pt = [&](float theta, float phi) {
        const float r = kArcRadius + kArcTube * std::cos(phi);
        return glm::vec3(r * std::cos(theta), r * std::sin(theta), kArcTube * std::sin(phi));
    };
    for (int i = 0; i < segs; ++i) {
        const float t1 = kArcFrom + (kArcTo - kArcFrom) * i / segs;
        const float t2 = kArcFrom + (kArcTo - kArcFrom) * (i + 1) / segs;
        for (int j = 0; j < tubeSegs; ++j) {
            const float p1 = 2.0f * (float)M_PI * j / tubeSegs;
            const float p2 = 2.0f * (float)M_PI * (j + 1) / tubeSegs;
            addTri(pt(t1, p1), pt(t2, p1), pt(t2, p2));
            addTri(pt(t1, p1), pt(t2, p2), pt(t1, p2));
        }
    }
    uploadMesh(m_arcVao, m_arcVbo, verts, m_arcVertCount);
}

float Gizmo::handleScale(const glm::mat4& view, const glm::mat4& proj, float vpH) const {
    if (vpH <= 1.0f) vpH = 1.0f;
    const float frac = gizmoUnitDp() * materializr::uiScale() / vpH;
    // Visible world height at the gizmo: ortho has none of the depth term.
    const bool ortho = std::abs(proj[2][3]) < 1e-6f;
    const float depth = std::max(1e-3f, -(view * glm::vec4(m_position, 1.0f)).z);
    const float visH = (ortho ? 2.0f : 2.0f * depth) / std::max(1e-6f, proj[1][1]);
    return std::max(1e-4f, frac * visH);
}

void Gizmo::setMode(GizmoMode mode) { m_mode = mode; m_combined = false; }
void Gizmo::setCombinedMove() { m_mode = GizmoMode::Translate; m_combined = true; }
GizmoMode Gizmo::getMode() const { return m_mode; }
void Gizmo::setPosition(glm::vec3 pos) { m_position = pos; }
void Gizmo::setVisible(bool vis) { m_visible = vis; }
bool Gizmo::isVisible() const { return m_visible; }

void Gizmo::cancelDrag() {
    m_draggingAxis = GizmoAxis::None;
    m_hoveredAxis = GizmoAxis::None;
}

glm::vec2 Gizmo::worldToNDC(glm::vec3 pos, const glm::mat4& vp) {
    glm::vec4 clip = vp * glm::vec4(pos, 1.0f);
    if (std::abs(clip.w) < 1e-6f) return glm::vec2(1e6f);
    return glm::vec2(clip.x / clip.w, clip.y / clip.w);
}

float Gizmo::screenDistToSegment(glm::vec2 mouse, glm::vec2 a, glm::vec2 b) {
    glm::vec2 ab = b - a;
    float len2 = glm::dot(ab, ab);
    if (len2 < 1e-12f) return glm::length(mouse - a);
    float t = glm::clamp(glm::dot(mouse - a, ab) / len2, 0.0f, 1.0f);
    return glm::length(mouse - (a + t * ab));
}

void Gizmo::renderArrowAlong(const glm::mat4& view, const glm::mat4& projection,
                            glm::vec3 position, glm::vec3 dir, glm::vec3 color) {
    if (!m_program || !m_arrowVao) return;
    glm::vec3 camPos = glm::vec3(glm::inverse(view)[3]);
    float scale = glm::length(camPos - position) * gizmoScale();
    if (scale < 0.01f) scale = 0.01f;

    // Rotation mapping the +Z canonical arrow onto `dir`.
    glm::vec3 d = glm::normalize(dir);
    glm::vec3 z(0.0f, 0.0f, 1.0f);
    glm::vec3 ax = glm::cross(z, d);
    float axl = glm::length(ax);
    float c = glm::clamp(glm::dot(z, d), -1.0f, 1.0f);
    glm::mat4 rot(1.0f);
    if (axl < 1e-6f) {
        if (c < 0.0f) rot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f),
                                        glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        rot = glm::rotate(glm::mat4(1.0f), std::acos(c), ax / axl);
    }
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                    * glm::scale(glm::mat4(1.0f), glm::vec3(scale)) * rot;
    glm::mat4 mvp = projection * view * model;

    glDisable(GL_DEPTH_TEST);
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locMVP, 1, GL_FALSE, glm::value_ptr(mvp));
    glUniform3fv(m_locColor, 1, glm::value_ptr(color));
    glUniform1f(m_locAlpha, 1.0f);
    glBindVertexArray(m_arrowVao);
    glDrawArrays(GL_TRIANGLES, 0, m_arrowVertCount);
    glBindVertexArray(0);
}

void Gizmo::renderRingAbout(const glm::mat4& view, const glm::mat4& projection,
                            glm::vec3 position, glm::vec3 axisDir, glm::vec3 color) {
    if (!m_program || !m_ringVao) return;
    glm::vec3 camPos = glm::vec3(glm::inverse(view)[3]);
    float scale = glm::length(camPos - position) * gizmoScale();
    if (scale < 0.01f) scale = 0.01f;

    // The ring mesh lies in the XY plane (axis +Z); rotate +Z onto axisDir so it
    // sits in the plane perpendicular to the tilt axis.
    glm::vec3 d = glm::normalize(axisDir);
    glm::vec3 z(0.0f, 0.0f, 1.0f);
    glm::vec3 ax = glm::cross(z, d);
    float axl = glm::length(ax);
    float c = glm::clamp(glm::dot(z, d), -1.0f, 1.0f);
    glm::mat4 rot(1.0f);
    if (axl < 1e-6f) {
        if (c < 0.0f) rot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f),
                                        glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        rot = glm::rotate(glm::mat4(1.0f), std::acos(c), ax / axl);
    }
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                    * glm::scale(glm::mat4(1.0f), glm::vec3(scale)) * rot;
    glm::mat4 mvp = projection * view * model;

    glDisable(GL_DEPTH_TEST);
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locMVP, 1, GL_FALSE, glm::value_ptr(mvp));
    glUniform3fv(m_locColor, 1, glm::value_ptr(color));
    glUniform1f(m_locAlpha, 1.0f);
    glBindVertexArray(m_ringVao);
    glDrawArrays(GL_TRIANGLES, 0, m_ringVertCount);
    glBindVertexArray(0);
}

void Gizmo::renderCubeAlong(const glm::mat4& view, const glm::mat4& projection,
                            glm::vec3 position, glm::vec3 dir, glm::vec3 color) {
    if (!m_program || !m_cubeVao) return;
    glm::vec3 camPos = glm::vec3(glm::inverse(view)[3]);
    float scale = glm::length(camPos - position) * gizmoScale();
    if (scale < 0.01f) scale = 0.01f;

    glm::vec3 d = glm::normalize(dir);
    glm::vec3 z(0.0f, 0.0f, 1.0f);
    glm::vec3 ax = glm::cross(z, d);
    float axl = glm::length(ax);
    float c = glm::clamp(glm::dot(z, d), -1.0f, 1.0f);
    glm::mat4 rot(1.0f);
    if (axl < 1e-6f) {
        if (c < 0.0f) rot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f),
                                        glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        rot = glm::rotate(glm::mat4(1.0f), std::acos(c), ax / axl);
    }
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                    * glm::scale(glm::mat4(1.0f), glm::vec3(scale)) * rot;
    glm::mat4 mvp = projection * view * model;

    glDisable(GL_DEPTH_TEST);
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locMVP, 1, GL_FALSE, glm::value_ptr(mvp));
    glUniform3fv(m_locColor, 1, glm::value_ptr(color));
    glUniform1f(m_locAlpha, 1.0f);
    glBindVertexArray(m_cubeVao);
    glDrawArrays(GL_TRIANGLES, 0, m_cubeVertCount);
    glBindVertexArray(0);
}

void Gizmo::render(const glm::mat4& view, const glm::mat4& projection) {
    if (!m_visible || !m_program) return;

    float vpH = m_viewH;
    if (vpH <= 0.0f) {
        GLint vpr[4] = {0, 0, 0, 0};
        glGetIntegerv(GL_VIEWPORT, vpr);
        vpH = static_cast<float>(vpr[3]);
    }
    const float scale = handleScale(view, projection, vpH);
    const glm::mat4 vp = projection * view;

    // Rotation matrices to align +Z mesh with each axis
    const glm::mat4 rotX = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0));
    const glm::mat4 rotY = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1,0,0));
    const glm::mat4 rotZ = glm::mat4(1.0f);
    // Rotate-only mode's rings: ring in XY, turned into each axis's plane.
    const glm::mat4 ringRotX = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0));
    const glm::mat4 ringRotY = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1,0,0));
    const glm::mat4 ringRotZ = glm::mat4(1.0f);

    struct DrawInfo {
        GizmoAxis axis;
        GizmoMode mode;
        glm::vec3 baseColor;
        glm::mat4 rotation;
        unsigned int vao;
        int vertCount;
        float alpha;
    };
    std::vector<DrawInfo> draws;
    const GizmoAxis ax[3] = {GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z};
    const glm::mat4 arrowRot[3] = {rotX, rotY, rotZ};
    const glm::mat4 ringRot[3] = {ringRotX, ringRotY, ringRotZ};
    if (m_combined) {
        // Fusion-style Move: translucent plane squares underneath, then the
        // rotation arcs, then the arrows on top.
        const GizmoAxis planes[3] = {GizmoAxis::YZ, GizmoAxis::XZ, GizmoAxis::XY};
        for (int i = 0; i < 3; ++i)
            draws.push_back({planes[i], GizmoMode::Translate, kAxisCol[i],
                             planeRotation(planes[i]), m_planeVao, m_planeVertCount, 0.45f});
        for (int i = 0; i < 3; ++i)
            draws.push_back({ax[i], GizmoMode::Rotate, kAxisCol[i], arcFrame(ax[i]),
                             m_arcVao, m_arcVertCount, 1.0f});
        for (int i = 0; i < 3; ++i)
            draws.push_back({ax[i], GizmoMode::Translate, kAxisCol[i], arrowRot[i],
                             m_arrowVao, m_arrowVertCount, 1.0f});
    } else {
        for (int i = 0; i < 3; ++i) {
            if (m_mode == GizmoMode::Translate)
                draws.push_back({ax[i], m_mode, kAxisCol[i], arrowRot[i], m_arrowVao, m_arrowVertCount, 1.0f});
            else if (m_mode == GizmoMode::Rotate)
                draws.push_back({ax[i], m_mode, kAxisCol[i], ringRot[i], m_ringVao, m_ringVertCount, 1.0f});
            else
                draws.push_back({ax[i], m_mode, kAxisCol[i], arrowRot[i], m_cubeVao, m_cubeVertCount, 1.0f});
        }
    }

    glDisable(GL_DEPTH_TEST);
    const GLboolean blendWas = glIsEnabled(GL_BLEND);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(m_program);

    for (const auto& d : draws) {
        const glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position)
                              * glm::scale(glm::mat4(1.0f), glm::vec3(scale))
                              * d.rotation;
        const glm::mat4 mvp = vp * model;

        glm::vec3 color = d.baseColor;
        float alpha = d.alpha;
        const bool grabbed = m_draggingAxis == d.axis && m_draggingMode == d.mode;
        const bool hovered = m_hoveredAxis == d.axis && m_hoveredMode == d.mode &&
                             m_draggingAxis == GizmoAxis::None;
        if (grabbed) {
            color = kActiveCol;
            alpha = std::max(alpha, 0.75f);
        } else if (hovered) {
            color = glm::mix(color, glm::vec3(1.0f), 0.35f);
            alpha = std::max(alpha, 0.65f);
        }

        glUniformMatrix4fv(m_locMVP, 1, GL_FALSE, glm::value_ptr(mvp));
        glUniform3fv(m_locColor, 1, glm::value_ptr(color));
        glUniform1f(m_locAlpha, alpha);

        glBindVertexArray(d.vao);
        glDrawArrays(GL_TRIANGLES, 0, d.vertCount);
    }

    glBindVertexArray(0);
    glUseProgram(0);
    if (!blendWas) glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

Gizmo::PickResult Gizmo::pickNearest(float mx, float my, float vpW, float vpH,
                                      const Camera& camera) {
    // Distances in viewport pixels against a catch radius in dp (a fingertip
    // needs more slack than a cursor).
    PickResult best;
    best.axis = GizmoAxis::None;
    best.mode = GizmoMode::Translate;
    best.dist = (materializr::touchMode() ? 16.0f : 7.0f) * materializr::uiScale();

    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 proj = camera.getProjectionMatrix();
    const glm::mat4 vp = proj * view;
    const float scale = handleScale(view, proj, vpH);
    auto toPx = [&](const glm::vec3& w) {
        const glm::vec2 n = worldToNDC(w, vp);
        return glm::vec2((n.x + 1.0f) * 0.5f * vpW, (1.0f - n.y) * 0.5f * vpH);
    };
    const glm::vec2 mouse(mx, my);
    const glm::vec2 originPx = toPx(m_position);

    const glm::vec3 axisDirs[3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    const GizmoAxis axisIds[3] = {GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z};
    auto consider = [&](float d, GizmoAxis a, GizmoMode m) {
        if (d < best.dist) { best.dist = d; best.axis = a; best.mode = m; }
    };

    // Only the active mode's handles are pickable - otherwise the long translate
    // arrow lines would steal clicks meant for the scale cubes / rotate rings.
    const bool pickTranslate = m_combined || m_mode == GizmoMode::Translate;
    const bool pickRotate = m_combined || m_mode == GizmoMode::Rotate;
    if (pickTranslate)
        for (int i = 0; i < 3; i++) {
            // Combined: the shaft starts past the plane squares so a touch
            // there reaches them.
            const float from = m_combined ? kPlaneHi : 0.0f;
            const glm::vec2 a = m_combined ? toPx(m_position + axisDirs[i] * scale * from) : originPx;
            const glm::vec2 b = toPx(m_position + axisDirs[i] * scale * (kAxisLength + kConeHeight));
            consider(screenDistToSegment(mouse, a, b), axisIds[i], GizmoMode::Translate);
        }

    if (!m_combined && m_mode == GizmoMode::Scale)
        for (int i = 0; i < 3; i++)
            consider(glm::length(mouse - toPx(m_position + axisDirs[i] * scale * kCubeOffset)),
                     axisIds[i], GizmoMode::Scale);

    if (pickRotate)
        for (int i = 0; i < 3; i++) {
            // Combined: the quarter arc; Rotate-only: the 270-degree ring.
            const glm::mat4 frame = m_combined ? arcFrame(axisIds[i])
                : (i == 0 ? glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0))
                 : i == 1 ? glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1,0,0))
                          : glm::mat4(1.0f));
            const float r = m_combined ? kArcRadius : kRingRadius;
            const float t0 = m_combined ? kArcFrom : 0.0f;
            const float t1 = m_combined ? kArcTo : 2.0f * (float)M_PI * 0.75f;
            const int segs = m_combined ? 16 : kRingSegments;
            glm::vec2 prev(0.0f);
            for (int s = 0; s <= segs; s++) {
                const float theta = t0 + (t1 - t0) * s / segs;
                const glm::vec4 local(r * std::cos(theta), r * std::sin(theta), 0.0f, 0.0f);
                const glm::vec2 p = toPx(m_position + glm::vec3(frame * local) * scale);
                if (s > 0) consider(screenDistToSegment(mouse, prev, p), axisIds[i], GizmoMode::Rotate);
                prev = p;
            }
        }

    // Plane squares (combined Move): a touch inside the projected square wins
    // outright; near it counts like any other handle.
    if (m_combined) {
        const GizmoAxis planes[3] = {GizmoAxis::XY, GizmoAxis::YZ, GizmoAxis::XZ};
        for (GizmoAxis pa : planes) {
            const glm::mat4 rot = planeRotation(pa);
            const float a = kPlaneLo, b = kPlaneHi;
            const glm::vec3 local[4] = {{a, a, 0}, {b, a, 0}, {b, b, 0}, {a, b, 0}};
            glm::vec2 q[4];
            for (int k = 0; k < 4; ++k)
                q[k] = toPx(m_position + glm::vec3(rot * glm::vec4(local[k], 0.0f)) * scale);
            // Convex quad: inside when every edge sees the point on one side.
            float sgn = 0.0f;
            bool inside = true;
            for (int k = 0; k < 4; ++k) {
                const glm::vec2 e = q[(k + 1) % 4] - q[k];
                const glm::vec2 w = mouse - q[k];
                const float c = e.x * w.y - e.y * w.x;
                if (std::abs(c) < 1e-6f) continue;
                if (sgn == 0.0f) sgn = c;
                else if ((c > 0.0f) != (sgn > 0.0f)) { inside = false; break; }
            }
            float d = 0.0f;
            if (!inside) {
                d = 1e30f;
                for (int k = 0; k < 4; ++k)
                    d = std::min(d, screenDistToSegment(mouse, q[k], q[(k + 1) % 4]));
            }
            consider(d, pa, GizmoMode::Translate);
        }
    }

    return best;
}

glm::vec3 Gizmo::projectOnAxis(float mx, float my, float vpW, float vpH,
                                const Camera& camera, GizmoAxis axis) {
    glm::mat4 invVP = glm::inverse(camera.getProjectionMatrix() * camera.getViewMatrix());

    float ndcX = (mx / vpW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / vpH) * 2.0f;

    glm::vec4 nearClip = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 farClip  = invVP * glm::vec4(ndcX, ndcY,  1.0f, 1.0f);
    glm::vec3 rayOrigin = glm::vec3(nearClip) / nearClip.w;
    glm::vec3 rayDir = glm::normalize(glm::vec3(farClip) / farClip.w - rayOrigin);

    glm::vec3 axisDir(0);
    if (axis == GizmoAxis::X) axisDir = glm::vec3(1,0,0);
    else if (axis == GizmoAxis::Y) axisDir = glm::vec3(0,1,0);
    else axisDir = glm::vec3(0,0,1);

    glm::vec3 w0 = m_position - rayOrigin;
    float a = 1.0f; // dot(axisDir, axisDir)
    float b = glm::dot(axisDir, rayDir);
    float c = 1.0f; // dot(rayDir, rayDir)
    float d = glm::dot(axisDir, w0);
    float e = glm::dot(rayDir, w0);
    float denom = a * c - b * b;
    float t = (std::abs(denom) > 1e-8f) ? (b * e - c * d) / denom : 0.0f;

    return m_position + t * axisDir;
}

bool Gizmo::projectOnPlane(float mx, float my, float vpW, float vpH,
                           const Camera& camera, GizmoAxis plane, glm::vec3& out) {
    glm::mat4 invVP = glm::inverse(camera.getProjectionMatrix() * camera.getViewMatrix());
    float ndcX = (mx / vpW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / vpH) * 2.0f;
    glm::vec4 nearClip = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 farClip  = invVP * glm::vec4(ndcX, ndcY,  1.0f, 1.0f);
    glm::vec3 rayOrigin = glm::vec3(nearClip) / nearClip.w;
    glm::vec3 rayDir = glm::normalize(glm::vec3(farClip) / farClip.w - rayOrigin);
    const glm::vec3 n = planeNormal(plane);
    const float denom = glm::dot(rayDir, n);
    // Edge-on: the hit races off for a pixel of motion (same guard idea as
    // ringAngle, a little looser - the plane handle is hidden there anyway).
    if (std::abs(denom) < 0.08f) return false;
    const float t = glm::dot(m_position - rayOrigin, n) / denom;
    if (t <= 0.0f) return false;
    out = rayOrigin + rayDir * t;
    return true;
}

bool Gizmo::ringAngle(float mx, float my, float vpW, float vpH,
                      const Camera& camera, GizmoAxis axis, float& outDeg) {
    glm::mat4 invVP = glm::inverse(camera.getProjectionMatrix() * camera.getViewMatrix());
    float ndcX = (mx / vpW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / vpH) * 2.0f;
    glm::vec4 nearClip = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 farClip  = invVP * glm::vec4(ndcX, ndcY,  1.0f, 1.0f);
    glm::vec3 rayOrigin = glm::vec3(nearClip) / nearClip.w;
    glm::vec3 rayDir = glm::normalize(glm::vec3(farClip) / farClip.w - rayOrigin);

    // Axis (plane normal) and an in-plane basis (u, v) with u x v = axis.
    glm::vec3 n, u, v;
    if (axis == GizmoAxis::X)      { n = {1,0,0}; u = {0,1,0}; v = {0,0,1}; }
    else if (axis == GizmoAxis::Y) { n = {0,1,0}; u = {0,0,1}; v = {1,0,0}; }
    else                           { n = {0,0,1}; u = {1,0,0}; v = {0,1,0}; }

    float denom = glm::dot(rayDir, n);
    // Reject near-grazing rays: at the true parallel limit t explodes towards
    // +/-infinity, but well before that the intersection point races across
    // the plane for a pixel of mouse motion - a tiny cursor nudge reads as a
    // huge swept angle (the "spins thousands of degrees" bug). 1e-5 (matching
    // the equivalent guard on the revolve-arc and Move-Face ring drags) is
    // too tight to catch this; widen it so the ring keeps its low-sensitivity
    // handoff to the None case well before the hit point runs away.
    if (std::abs(denom) < 0.15f) return false;
    float t = glm::dot(m_position - rayOrigin, n) / denom;
    // A hit behind the camera is not where the cursor visually is either -
    // same failure mode as above, just on the other side of the singularity.
    if (t <= 0.0f) return false;
    glm::vec3 hit = rayOrigin + rayDir * t;
    glm::vec3 vec = hit - m_position;
    outDeg = glm::degrees(std::atan2(glm::dot(vec, v), glm::dot(vec, u)));
    return true;
}

GizmoResult Gizmo::handleInput(float mouseX, float mouseY,
                                float vpWidth, float vpHeight,
                                bool mouseDown, bool mouseJustPressed,
                                const Camera& camera) {
    GizmoResult result;
    if (!m_visible) return result;
    m_viewH = vpHeight;

    if (mouseJustPressed) {
        auto pick = pickNearest(mouseX, mouseY, vpWidth, vpHeight, camera);
        if (pick.axis != GizmoAxis::None) {
            bool started = true;
            if (pick.mode == GizmoMode::Rotate) {
                // If the ring can't be read at press time, don't start the
                // drag at all - starting it here would leave m_lastDragAngle
                // stale, so the next good frame diffs against an unrelated
                // baseline angle and reads as a huge spurious rotation.
                started = ringAngle(mouseX, mouseY, vpWidth, vpHeight, camera,
                                     pick.axis, m_lastDragAngle);
            } else if (isPlaneAxis(pick.axis)) {
                started = projectOnPlane(mouseX, mouseY, vpWidth, vpHeight, camera,
                                         pick.axis, m_lastDragPos);
            } else {
                m_lastDragPos = projectOnAxis(mouseX, mouseY, vpWidth, vpHeight, camera, pick.axis);
            }
            if (started) {
                m_draggingAxis = pick.axis;
                m_draggingMode = pick.mode;
                // Combined Move: the grabbed handle decides the effective
                // mode until the next grab (the app commits after release).
                if (m_combined) m_mode = pick.mode;
            }
        }
    }

    if (mouseDown && m_draggingAxis != GizmoAxis::None) {
        glm::vec3 axisDir = (m_draggingAxis == GizmoAxis::X) ? glm::vec3(1,0,0)
                          : (m_draggingAxis == GizmoAxis::Y) ? glm::vec3(0,1,0)
                                                             : glm::vec3(0,0,1);
        if (m_draggingMode == GizmoMode::Rotate) {
            // Rotation: signed angle change as the mouse goes around the ring,
            // encoded as axisDir * degrees so the app reads it via dot(delta,axis).
            float ang;
            if (ringAngle(mouseX, mouseY, vpWidth, vpHeight, camera, m_draggingAxis, ang)) {
                float dDeg = ang - m_lastDragAngle;
                while (dDeg > 180.0f) dDeg -= 360.0f;   // shortest direction
                while (dDeg < -180.0f) dDeg += 360.0f;
                m_lastDragAngle = ang;
                result.changed = true;
                result.activeAxis = m_draggingAxis;
                result.mode = m_draggingMode;
                result.delta = axisDir * dDeg;
            }
        } else if (isPlaneAxis(m_draggingAxis)) {
            // Combined Move plane handle: free movement within the plane.
            glm::vec3 currentPos;
            if (projectOnPlane(mouseX, mouseY, vpWidth, vpHeight, camera,
                               m_draggingAxis, currentPos)) {
                result.changed = true;
                result.activeAxis = m_draggingAxis;
                result.mode = m_draggingMode;
                result.delta = currentPos - m_lastDragPos;
                m_lastDragPos = currentPos;
            }
        } else {
            // Translate / Scale: movement along the axis line.
            glm::vec3 currentPos = projectOnAxis(mouseX, mouseY, vpWidth, vpHeight,
                                                  camera, m_draggingAxis);
            glm::vec3 delta = currentPos - m_lastDragPos;
            m_lastDragPos = currentPos;
            result.changed = true;
            result.activeAxis = m_draggingAxis;
            result.mode = m_draggingMode;
            result.delta = delta;
        }
    }

    if (!mouseDown) {
        m_draggingAxis = GizmoAxis::None;
        auto pick = pickNearest(mouseX, mouseY, vpWidth, vpHeight, camera);
        m_hoveredAxis = pick.axis;
        m_hoveredMode = pick.mode;
    }

    if (m_draggingAxis != GizmoAxis::None) {
        result.activeAxis = m_draggingAxis;
    }

    return result;
}

bool Gizmo::compileShader(unsigned int& shader, unsigned int type, const char* source) {
    shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        std::fprintf(stderr, "Gizmo shader error: %s\n", log);
        glDeleteShader(shader);
        shader = 0;
        return false;
    }
    return true;
}

} // namespace materializr
