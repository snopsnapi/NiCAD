#pragma once
// Shared screen-space arrow geometry for the touch op handles (Offset
// Face, Extrude, Push/Pull). ONE function computes the arrow's screen
// segment, and both the controller's hit-test and the viewport's overlay
// draw call it - so the visible arrow IS the click target.
#include "../ui_scale.h"
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

namespace materializr {
namespace toucharrow {

// The arrow is never shorter than this on screen - a 5 mm arrow on a big
// part is a few pixels, impossible to grab with a finger.
constexpr float kMinArrowDp = 70.0f;

// Drawing sizes, dp: shaft thickness and head length, idle / grabbed.
constexpr float kLineDp        = 5.0f;
constexpr float kLineGrabbedDp = 7.0f;
constexpr float kHeadDp        = 22.0f;
constexpr float kHeadGrabbedDp = 26.0f;

// Push/Pull starter arrow (distance ~0): double-headed, drawn through the
// face centre at +/- this many dp in SCREEN space.
constexpr float kStarterHalfDp = 40.0f;

// Arrow screen segment: base at `origin`, tip `distance` along `dir`,
// clamped to a minimum on-screen length (kMinArrowDp). False when either
// projection fails or `dir` points at the camera.
template <class ToScreen>
bool arrowScreen(const ToScreen& toScreen, const glm::vec3& origin,
                 const glm::vec3& dir, float distance,
                 glm::vec2& a, glm::vec2& b) {
    if (!toScreen(origin, a)) return false;
    glm::vec2 unit;
    if (!toScreen(origin + dir, unit)) return false;
    glm::vec2 sd = unit - a;
    const float dl = glm::length(sd);
    if (dl < 1e-4f) return false;            // dir points at the camera
    sd /= dl;
    float len = dl * std::max(std::abs(distance), 1.0f);
    len = std::max(len, kMinArrowDp * materializr::uiScale());
    b = a + sd * len;
    return true;
}

// Distance from a point to a screen segment - the shaft hit-test.
inline float distToSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 d = b - a;
    const float len2 = glm::dot(d, d);
    glm::vec2 q;
    if (len2 < 1e-6f) q = p - a;
    else {
        const float t = glm::clamp(glm::dot(p - a, d) / len2, 0.0f, 1.0f);
        q = p - (a + t * d);
    }
    return glm::length(q);
}

// Hit radius in PHYSICAL pixels: ImGui runs at device resolution, and a
// finger needs a much fatter grab zone (48 dp) than a mouse (20 dp).
inline float pickRadiusPx(bool imTouch, float uiScale) {
    return (imTouch ? 48.0f : 20.0f) * uiScale;
}

} // namespace toucharrow
} // namespace materializr
