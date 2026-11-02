#include "FormRenderer.h"

#include <glm/gtc/type_ptr.hpp>
#include <cstdio>

namespace materializr {

// Two-sided diffuse + a soft specular: a form is often an open sheet, so
// both sides light, and a highlight makes its curvature readable.
static const char* s_vert = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_mvp;
out vec3 v_normal;
out vec3 v_pos;
void main() {
    v_normal = a_normal;
    v_pos = a_position;
    gl_Position = u_mvp * vec4(a_position, 1.0);
}
)";

static const char* s_frag = R"(
#version 330 core
uniform vec3 u_color;
uniform vec3 u_eye;
in vec3 v_normal;
in vec3 v_pos;
out vec4 fragColor;
void main() {
    vec3 n = normalize(v_normal);
    vec3 v = normalize(u_eye - v_pos);
    if (dot(n, v) < 0.0) n = -n;
    vec3 l = normalize(vec3(0.4, 0.7, 0.6));
    float diff = max(dot(n, l), 0.0);
    float spec = pow(max(dot(n, normalize(l + v)), 0.0), 40.0);
    vec3 c = u_color * (0.35 + 0.65 * diff) + vec3(0.25) * spec;
    fragColor = vec4(c, 1.0);
}
)";

static unsigned int compile(unsigned int type, const char* src) {
    unsigned int s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::fprintf(stderr, "FormRenderer shader: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

FormRenderer::~FormRenderer() {
    freeAll();
    if (m_program) glDeleteProgram(m_program);
}

void FormRenderer::freeAll() {
    for (Gpu& g : m_gpu) {
        if (g.vao) glDeleteVertexArrays(1, &g.vao);
        if (g.vbo) glDeleteBuffers(1, &g.vbo);
    }
    m_gpu.clear();
}

bool FormRenderer::initialize() {
    const unsigned int vs = compile(GL_VERTEX_SHADER, s_vert);
    const unsigned int fs = compile(GL_FRAGMENT_SHADER, s_frag);
    if (!vs || !fs) return false;
    m_program = glCreateProgram();
    glAttachShader(m_program, vs);
    glAttachShader(m_program, fs);
    glLinkProgram(m_program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    int ok = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
    if (!ok) {
        glDeleteProgram(m_program);
        m_program = 0;
        return false;
    }
    m_locMVP = glGetUniformLocation(m_program, "u_mvp");
    m_locColor = glGetUniformLocation(m_program, "u_color");
    m_locEye = glGetUniformLocation(m_program, "u_eye");
    return true;
}

void FormRenderer::setMeshes(const std::vector<Mesh>& meshes) {
    freeAll();
    if (!m_program) return;
    for (const Mesh& m : meshes) {
        if (m.vertices.empty()) continue;
        Gpu g;
        g.count = static_cast<int>(m.vertices.size() / 6);
        g.color = m.color;
        glGenVertexArrays(1, &g.vao);
        glGenBuffers(1, &g.vbo);
        glBindVertexArray(g.vao);
        glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
        glBufferData(GL_ARRAY_BUFFER, m.vertices.size() * sizeof(float), m.vertices.data(),
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        m_gpu.push_back(g);
    }
}

void FormRenderer::render(const glm::mat4& view, const glm::mat4& projection,
                          const glm::vec3& eye) {
    if (!m_program || m_gpu.empty()) return;
    const glm::mat4 vp = projection * view;
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locMVP, 1, GL_FALSE, glm::value_ptr(vp));
    glUniform3fv(m_locEye, 1, glm::value_ptr(eye));
    for (const Gpu& g : m_gpu) {
        glUniform3fv(m_locColor, 1, glm::value_ptr(g.color));
        glBindVertexArray(g.vao);
        glDrawArrays(GL_TRIANGLES, 0, g.count);
    }
    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace materializr
