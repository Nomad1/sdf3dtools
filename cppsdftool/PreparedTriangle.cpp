#include "PreparedTriangle.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <algorithm>

PreparedTriangle::PreparedTriangle(int id, const std::vector<glm::vec3>& vertices, 
                                          int ia, int ib, int ic)
    : id(id), ia(ia), ib(ib), ic(ic),a(vertices[ia]), b(vertices[ib]), c(vertices[ic])  {
    // Calculate normal vector
    glm::vec3 n = glm::cross(b - a, c - a);
    float area = glm::dot(n, n);

    // Calculate area and normalize normal vector
    normalLength = std::sqrt(area);// std::max(std::sqrt(area), 1e-20f);
    this->n = n / normalLength;
    this->area = area / 2.0f;

    // Calculate center and radius
    glm::vec3 center = (a + b + c) / 3.0f;
    radius = std::sqrt(std::max(
        glm::dot(a - center, a - center),
        std::max(
            glm::dot(b - center, b - center),
            glm::dot(c - center, c - center)
        )
    ));

    // Calculate bounds
    lowerBound = glm::min(glm::min(a, b), c);
    upperBound = glm::max(glm::max(a, b), c);
}

void PreparedTriangle::setPseudoNormals(const glm::vec3& van, const glm::vec3& vbn, const glm::vec3& vcn, const glm::vec3& eab, const glm::vec3& ebc, const glm::vec3& eac) {
    this->van = van;
    this->vbn = vbn;
    this->vcn = vcn;
    this->eab = eab;
    this->ebc = ebc;
    this->eac = eac;
}

std::tuple<glm::vec3, glm::vec3, glm::vec3, int> PreparedTriangle::closestPointToTriangle(const glm::vec3& p) const {
    float snom = glm::dot(p - a, b - a);
    float sdenom = glm::dot(p - b, a - b);
    float tnom = glm::dot(p - a, c - a);
    float tdenom = glm::dot(p - c, a - c);
    float unom = glm::dot(p - b, c - b);
    float udenom = glm::dot(p - c, b - c);

    // Check vertices
    if (snom <= 0.0f && tnom <= 0.0f) {
        return {a, glm::vec3(1.0f, 0.0f, 0.0f), van, 1};
    }

    if (sdenom <= 0.0f && unom <= 0.0f) {
        return {b, glm::vec3(0.0f, 1.0f, 0.0f), vbn, 2};
    }

    if (tdenom <= 0.0f && udenom <= 0.0f) {
        return {c, glm::vec3(0.0f, 0.0f, 1.0f), vcn, 3};
    }

    // Check edges
    float coordsPab = glm::dot(n, glm::cross(a - p, b - p));
    if (coordsPab <= 0.0f && snom >= 0.0f && sdenom >= 0.0f) {
        float nab = snom / (snom + sdenom);
        return {a * (1.0f - nab) + b * nab, glm::vec3(1.0f - nab, nab, 0.0f), eab, 4};
    }

    float coordsPbc = glm::dot(n, glm::cross(b - p, c - p));
    if (coordsPbc <= 0.0f && unom >= 0.0f && udenom >= 0.0f) {
        float nbc = unom / (unom + udenom);
        return {b * (1.0f - nbc) + c * nbc, glm::vec3(0.0f, 1.0f - nbc, nbc), ebc, 5};
    }

    float coordsPca = glm::dot(n, glm::cross(c - p, a - p));
    if (coordsPca <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f) {
        float nca = tnom / (tnom + tdenom);
        return {a * nca + c * (1.0f - nca), glm::vec3(nca, 0.0f, 1.0f - nca), eac, 6};
    }

    // Point is inside triangle
    float denom = coordsPab + coordsPbc + coordsPca;
    float u = coordsPbc / denom;
    float v = coordsPca / denom;
    float w = coordsPab / denom;
    glm::vec3 weights(u, v, w);
    return {a * weights.x + b * weights.y + c * weights.z, weights, n, 0};
}

bool PreparedTriangle::intersectsRay(const glm::vec3& p, const glm::vec3& dir) const {
    glm::vec3 ba = b - a;
    glm::vec3 ca = c - a;
    glm::vec3 h = glm::cross(dir, ca);
    float proj = glm::dot(ba, h);

    if (std::abs(proj) < 1e-20f) {
        return false;
    }

    proj = 1.0f / proj;
    glm::vec3 pa = p - a;
    float u = glm::dot(pa, h) * proj;

    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    glm::vec3 q = glm::cross(pa, ba);
    float v = glm::dot(dir, q) * proj;

    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }

    float t = glm::dot(ca, q) * proj;
    return t >= 0.0f;
}

bool PreparedTriangle::intersectsAABB(const glm::vec3& lb, const glm::vec3& ub) const {
    return glm::all(glm::greaterThanEqual(ub, lowerBound)) && 
           glm::all(glm::lessThanEqual(lb, upperBound));
}

bool PreparedTriangle::intersectsSphere(const glm::vec3& point, float radius) const {
    glm::vec3 center = (a + b + c) / 3.0f;
    float distance = glm::length(point - center);
    return distance < radius + this->radius;
}

bool PreparedTriangle::planeIntersectsAABB(const glm::vec3& lb, const glm::vec3& ub) const {
    if (!intersectsAABB(lb, ub)) {
        return false;
    }

    glm::vec3 center = (ub + lb) / 2.0f;
    glm::vec3 e = ub - center;
    float r = glm::dot(e, glm::abs(n));
    float s = glm::dot(n, center) - glm::dot(n, a);
    return std::abs(s) <= r;
}