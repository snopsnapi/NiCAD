#include "MoveOp.h"
#include "core/Units.h"
#include "../i18n.h"
#include <gp_Ax1.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string toHex(const std::string& s) {
    static const char* d = "0123456789abcdef";
    std::string o;
    o.reserve(s.size() * 2);
    for (unsigned char c : s) { o.push_back(d[c >> 4]); o.push_back(d[c & 15]); }
    return o;
}

std::string fromHex(const std::string& h) {
    auto v = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return 0;
    };
    std::string o;
    o.reserve(h.size() / 2);
    for (size_t i = 0; i + 1 < h.size(); i += 2)
        o.push_back(static_cast<char>((v(h[i]) << 4) | v(h[i + 1])));
    return o;
}

bool readTriple(const std::string& v, double out[3]) {
    std::istringstream is(v);
    for (int i = 0; i < 3; ++i) {
        std::string tok;
        if (!std::getline(is, tok, ',')) return false;
        out[i] = std::atof(tok.c_str());
        if (!std::isfinite(out[i])) return false;
    }
    return true;
}

} // namespace

void MoveOp::addBody(int bodyId, const std::vector<int>& detachSketches) {
    m_target = Target::Bodies;
    BodyChild c;
    c.bodyId = bodyId;
    c.detach = detachSketches;
    c.op = std::make_unique<TransformOp>();
    c.op->setBodyId(bodyId);
    for (int s : detachSketches) c.op->addDetachSketch(s);
    m_bodies.push_back(std::move(c));
}

void MoveOp::setFace(int bodyId, const TopoDS_Face& face) {
    m_target = Target::Face;
    m_faceBody = bodyId;
    m_faceOp = std::make_unique<FaceTweakOp>();
    m_faceOp->setBody(bodyId);
    m_faceOp->setFace(face);
}

void MoveOp::setParams(const double t[3], const double rDeg[3]) {
    for (int i = 0; i < 3; ++i) { m_t[i] = t[i]; m_r[i] = rDeg[i]; }
}

// T(t) * T(p) * Rz * Ry * Rx * T(-p): rotate about the pivot (X first), then
// move the pivot by t.
gp_Trsf MoveOp::trsf() const {
    const gp_Pnt p(m_pivot[0], m_pivot[1], m_pivot[2]);
    gp_Trsf rx, ry, rz, tr;
    rx.SetRotation(gp_Ax1(p, gp_Dir(1, 0, 0)), m_r[0] * kPi / 180.0);
    ry.SetRotation(gp_Ax1(p, gp_Dir(0, 1, 0)), m_r[1] * kPi / 180.0);
    rz.SetRotation(gp_Ax1(p, gp_Dir(0, 0, 1)), m_r[2] * kPi / 180.0);
    tr.SetTranslation(gp_Vec(m_t[0], m_t[1], m_t[2]));
    return tr * rz * ry * rx;
}

void MoveOp::setFromTrsf(const gp_Trsf& m) {
    const gp_Pnt p(m_pivot[0], m_pivot[1], m_pivot[2]);
    const gp_Pnt q = p.Transformed(m);
    m_t[0] = q.X() - p.X();
    m_t[1] = q.Y() - p.Y();
    m_t[2] = q.Z() - p.Z();
    // R = Rz(c) Ry(b) Rx(a): r31 = -sin b, r32 = cos b sin a, r33 = cos b cos a,
    // r21 = sin c cos b, r11 = cos c cos b (1-based rows/cols).
    const gp_Mat R = m.VectorialPart();
    const double s = std::clamp(-R.Value(3, 1), -1.0, 1.0);
    const double b = std::asin(s);
    double a, c;
    if (std::abs(std::cos(b)) > 1e-9) {
        a = std::atan2(R.Value(3, 2), R.Value(3, 3));
        c = std::atan2(R.Value(2, 1), R.Value(1, 1));
    } else {   // gimbal lock: fold everything into X
        a = std::atan2(-R.Value(2, 3), R.Value(2, 2));
        c = 0.0;
    }
    m_r[0] = a * 180.0 / kPi;
    m_r[1] = b * 180.0 / kPi;
    m_r[2] = c * 180.0 / kPi;
    for (double& v : m_r)
        if (std::abs(v) < 1e-9) v = 0.0;
}

std::vector<int> MoveOp::bodyIds() const {
    std::vector<int> ids;
    if (m_target == Target::Face) {
        if (m_faceBody >= 0) ids.push_back(m_faceBody);
    } else {
        for (const auto& c : m_bodies) ids.push_back(c.bodyId);
    }
    return ids;
}

void MoveOp::pushTrsfToChildren() {
    const gp_Trsf t = trsf();
    for (auto& c : m_bodies)
        if (c.op) c.op->setRawTransform(t);
    if (m_faceOp) m_faceOp->setTransform(t);
}

bool MoveOp::execute(Document& doc) {
    pushTrsfToChildren();
    if (m_target == Target::Face) {
        if (!m_faceOp && !m_faceBlob.empty()) {
            m_faceOp = std::make_unique<FaceTweakOp>();
            m_faceOp->deserializeParams(m_faceBlob);
            m_faceOp->setTransform(trsf());
        }
        return m_faceOp && m_faceOp->execute(doc);
    }
    size_t done = 0;
    for (auto& c : m_bodies) {
        if (!c.op || !c.op->execute(doc)) {
            // All or nothing: put back the bodies already moved.
            for (size_t i = done; i-- > 0;)
                if (m_bodies[i].op) m_bodies[i].op->undo(doc);
            return false;
        }
        ++done;
    }
    return !m_bodies.empty();
}

bool MoveOp::undo(Document& doc) {
    bool ok = true;
    if (m_faceOp) ok = m_faceOp->undo(doc);
    for (size_t i = m_bodies.size(); i-- > 0;)
        if (m_bodies[i].op) ok = m_bodies[i].op->undo(doc) && ok;
    return ok;
}

std::string MoveOp::description() const {
    // User axes (Z up): user X = world X, user Y = world Z, user Z = world Y.
    char buf[200];
    const double ux = m_t[0], uy = m_t[2], uz = m_t[1];
    const double rx = m_r[0], ry = m_r[2], rz = m_r[1];
    std::string what;
    if (m_target == Target::Face) what = materializr::tr("face");
    else if (m_bodies.size() == 1) what = materializr::tr("body");
    else what = std::to_string(m_bodies.size()) + " " + materializr::tr("bodies");
    std::snprintf(buf, sizeof(buf), "%s: X %s  Y %s  Z %s",
                  what.c_str(), materializr::fmtLength(ux).c_str(),
                  materializr::fmtLength(uy).c_str(),
                  materializr::fmtLength(uz).c_str());
    std::string s = buf;
    if (std::abs(rx) > 1e-6 || std::abs(ry) > 1e-6 || std::abs(rz) > 1e-6) {
        std::snprintf(buf, sizeof(buf), "  |  %.1f\xC2\xB0 %.1f\xC2\xB0 %.1f\xC2\xB0", rx, ry, rz);
        s += buf;
    }
    return s;
}

void MoveOp::renderProperties() {
    ImGui::TextUnformatted(materializr::tr("Move"));
    ImGui::Separator();
    ImGui::TextUnformatted(description().c_str());
}

OperationDiff MoveOp::captureDiff() const {
    OperationDiff d;
    if (m_faceOp) {
        OperationDiff c = m_faceOp->captureDiff();
        d.modifiedBefore.insert(d.modifiedBefore.end(), c.modifiedBefore.begin(),
                                c.modifiedBefore.end());
    }
    for (const auto& b : m_bodies) {
        if (!b.op) continue;
        OperationDiff c = b.op->captureDiff();
        d.modifiedBefore.insert(d.modifiedBefore.end(), c.modifiedBefore.begin(),
                                c.modifiedBefore.end());
    }
    return d;
}

std::string MoveOp::serializeParams() const {
    std::ostringstream o;
    o << std::setprecision(std::numeric_limits<double>::max_digits10);
    o << "target=" << (m_target == Target::Face ? 1 : 0);
    o << ";t=" << m_t[0] << "," << m_t[1] << "," << m_t[2];
    o << ";r=" << m_r[0] << "," << m_r[1] << "," << m_r[2];
    o << ";p=" << m_pivot[0] << "," << m_pivot[1] << "," << m_pivot[2];
    if (m_target == Target::Face) {
        o << ";fbody=" << m_faceBody;
        o << ";face=" << toHex(m_faceOp ? m_faceOp->serializeParams() : m_faceBlob);
    } else {
        o << ";bodies=";
        for (size_t i = 0; i < m_bodies.size(); ++i) {
            if (i) o << ",";
            o << m_bodies[i].bodyId;
            for (int s : m_bodies[i].detach) o << "/" << s;
        }
    }
    return o.str();
}

bool MoveOp::deserializeParams(const std::string& blob) {
    bool any = false;
    size_t pos = 0;
    while (pos < blob.size()) {
        const size_t eq = blob.find('=', pos);
        if (eq == std::string::npos) break;
        size_t end = blob.find(';', eq);
        if (end == std::string::npos) end = blob.size();
        const std::string k = blob.substr(pos, eq - pos);
        const std::string v = blob.substr(eq + 1, end - eq - 1);
        if (k == "target") { m_target = std::atoi(v.c_str()) == 1 ? Target::Face : Target::Bodies; any = true; }
        else if (k == "t") any = readTriple(v, m_t) || any;
        else if (k == "r") any = readTriple(v, m_r) || any;
        else if (k == "p") any = readTriple(v, m_pivot) || any;
        else if (k == "fbody") m_faceBody = std::atoi(v.c_str());
        else if (k == "face") m_faceBlob = fromHex(v);
        else if (k == "bodies") {
            m_bodies.clear();
            std::istringstream is(v);
            std::string item;
            while (std::getline(is, item, ',')) {
                std::istringstream it(item);
                std::string tok;
                std::getline(it, tok, '/');
                const int id = std::atoi(tok.c_str());
                std::vector<int> det;
                while (std::getline(it, tok, '/')) det.push_back(std::atoi(tok.c_str()));
                addBody(id, det);
            }
        }
        pos = end + 1;
    }
    if (m_target == Target::Face && !m_faceBlob.empty()) {
        m_faceOp = std::make_unique<FaceTweakOp>();
        m_faceOp->deserializeParams(m_faceBlob);
    }
    return any;
}

bool MoveOp::rehydrateFromReload(const ReloadState& state, Document& doc) {
    pushTrsfToChildren();
    bool ok = true;
    if (m_faceOp) ok = m_faceOp->rehydrateFromReload(state, doc);
    for (auto& b : m_bodies)
        if (b.op) ok = b.op->rehydrateFromReload(state, doc) && ok;
    return ok;
}

void MoveOp::snapshotEditState() {
    if (m_faceOp) m_faceOp->snapshotEditState();
    for (auto& b : m_bodies)
        if (b.op) b.op->snapshotEditState();
}

void MoveOp::restoreEditState() {
    if (m_faceOp) m_faceOp->restoreEditState();
    for (auto& b : m_bodies)
        if (b.op) b.op->restoreEditState();
}
