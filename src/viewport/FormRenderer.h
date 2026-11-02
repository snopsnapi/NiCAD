#pragma once
#include "gl_common.h"
#include <glm/glm.hpp>
#include <vector>

namespace materializr {

// Form (T-spline-style) surfaces: opaque, depth-tested triangle meshes with
// smooth normals, one per form, each in its own colour. The forms are not
// Document bodies (they become one only through Convert), so the regular
// ShapeRenderer never sees them - this draws them in the body pass.
class FormRenderer {
public:
    FormRenderer() = default;
    ~FormRenderer();
    bool initialize();

    struct Mesh {
        std::vector<float> vertices;   // pos3 + normal3 per vertex, triangles
        glm::vec3 color{0.75f};
    };
    void setMeshes(const std::vector<Mesh>& meshes);
    void render(const glm::mat4& view, const glm::mat4& projection,
                const glm::vec3& eye);

private:
    struct Gpu { unsigned int vao = 0, vbo = 0; int count = 0; glm::vec3 color{0.75f}; };
    std::vector<Gpu> m_gpu;
    unsigned int m_program = 0;
    int m_locMVP = -1, m_locColor = -1, m_locEye = -1;
    void freeAll();
};

} // namespace materializr
