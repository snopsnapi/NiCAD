#pragma once
#include <glm/glm.hpp>
#include <optional>
#include <vector>
#include <Geom_BSplineCurve.hxx>

namespace materializr {
namespace fitspline {

struct Constraint {
    int index = 0;                       // which input point
    glm::dvec3 d1{0.0};                  // 1st derivative wrt the [0,1] parameter
    std::optional<glm::dvec3> d2;        // optional 2nd derivative
};

struct Curve {
    int degree = 0;
    std::vector<double> knots;           // FULL knot vector (with repeats), length = poles + degree + 1
    std::vector<glm::dvec3> poles;
    std::vector<double> params;          // parameter of each input point, in [0,1]
};

// Port of rt_spline.interpolate(): chord-length parameters, collocation matrix with averaged
// knots, degree = clamp(degree, needed multiplicity, sites-1); 2 points -> single span.
// Returns false for < 2 points or coincident consecutive points.
bool interpolate(const std::vector<glm::dvec3>& points, const std::vector<Constraint>& constraints,
                 int degree, Curve& out);

// Closed curve through the points (>= 3), periodic, via OCCT GeomAPI_Interpolate (as rt_spline does
// with PeriodicFlag). Null handle on failure.
Handle(Geom_BSplineCurve) interpolateClosed(const std::vector<glm::dvec3>& points);

// Curve -> OCCT curve (knots collapsed into distinct values + multiplicities).
Handle(Geom_BSplineCurve) toOcct(const Curve& c);

// Point / derivative on a Curve (own de Boor evaluation, no OCCT) - used by tests and UI previews.
glm::dvec3 evaluate(const Curve& c, double t, int derivative = 0);

} // namespace fitspline
} // namespace materializr
