#include "PreparedTriangle.hpp"
#include <limits>
#include <algorithm>

PreparedTriangle::PreparedTriangle(int id, const std::vector<glm::dvec3> &vertices,
                                   size_t ia, size_t ib, size_t ic)
    : id(id), ia(ia), ib(ib), ic(ic), a(vertices[ia]), b(vertices[ib]), c(vertices[ic])
{

    //glm::dvec3 edge1 = b - a;
    //glm::dvec3 edge2 = c - a;
    //n = glm::normalize(glm::cross(edge1, edge2));
    //area = 0.5 * glm::length(glm::cross(edge1, edge2));


    // Calculate normal vector
    glm::dvec3 ba = b - a;
    glm::dvec3 ca = c - a;
    glm::dvec3 bc = b - c;

    double lba = glm::dot(ba, ba); // Length squared of ba
    double lca = glm::dot(ca, ca); // Length squared of ca
    double lbc = glm::dot(bc, bc); // Length squared of bc

    if (lba >= lca && lbc >= lca)
    {
        // ba and bc are longest
        n = glm::cross(-bc, -ba);
    }
    else if (lba >= lbc && lca >= lbc)
    {
        // ba and ca are longest
        n = glm::cross(ba, ca);
    }
    else
    {
        // bc and ca are longest
        n = glm::cross(-ca, bc);
    }

    // Calculate area and normalize normal vector
    area = std::max(glm::length(n), std::numeric_limits<double>::epsilon());

    n /= area;
    area /= 2.0;

    // Calculate center and radius
    center = (a + b + c) / 3.0;
    radius = std::sqrt(std::max(
        glm::dot(a - center, a - center),
        std::max(
            glm::dot(b - center, b - center),
            glm::dot(c - center, c - center))));

    // Calculate bounds
    lowerBound = glm::min(glm::min(a, b), c);
    upperBound = glm::max(glm::max(a, b), c);
}

void PreparedTriangle::setPseudoNormals(const glm::dvec3 &van, const glm::dvec3 &vbn, const glm::dvec3 &vcn, const glm::dvec3 &eab, const glm::dvec3 &ebc, const glm::dvec3 &eac)
{
    this->van = van;
    this->vbn = vbn;
    this->vcn = vcn;
    this->eab = eab;
    this->ebc = ebc;
    this->eac = eac;
}

std::tuple<glm::dvec3, glm::dvec3, glm::dvec3, int> PreparedTriangle::closestPointToTriangle(const glm::dvec3 &p) const
{
    double snom = glm::dot(p - a, b - a);
    double sdenom = glm::dot(p - b, a - b);
    double tnom = glm::dot(p - a, c - a);
    double tdenom = glm::dot(p - c, a - c);
    double unom = glm::dot(p - b, c - b);
    double udenom = glm::dot(p - c, b - c);

    // Check vertices
    if (snom <= 0.0 && tnom <= 0.0)
    {
        return {a, glm::dvec3(1.0, 0.0, 0.0), van, 1};
    }

    if (sdenom <= 0.0 && unom <= 0.0)
    {
        return {b, glm::dvec3(0.0, 1.0, 0.0), vbn, 2};
    }

    if (tdenom <= 0.0 && udenom <= 0.0)
    {
        return {c, glm::dvec3(0.0, 0.0, 1.0), vcn, 3};
    }

    // Check edges
    double coordsPab = glm::dot(n, glm::cross(a - p, b - p));
    if (coordsPab <= 0.0 && snom >= 0.0 && sdenom >= 0.0)
    {
        double nab = snom / (snom + sdenom);
        return {a * (1.0 - nab) + b * nab, glm::dvec3(1.0 - nab, nab, 0.0), eab, 4};
    }

    double coordsPbc = glm::dot(n, glm::cross(b - p, c - p));
    if (coordsPbc <= 0.0 && unom >= 0.0 && udenom >= 0.0)
    {
        double nbc = unom / (unom + udenom);
        return {b * (1.0 - nbc) + c * nbc, glm::dvec3(0.0, 1.0 - nbc, nbc), ebc, 5};
    }

    double coordsPca = glm::dot(n, glm::cross(c - p, a - p));
    if (coordsPca <= 0.0 && tnom >= 0.0 && tdenom >= 0.0)
    {
        double nca = tnom / (tnom + tdenom);
        return {a * nca + c * (1.0 - nca), glm::dvec3(nca, 0.0, 1.0 - nca), eac, 6};
    }

    // Point is inside triangle
    double denom = coordsPab + coordsPbc + coordsPca;
    double u = coordsPbc / denom;
    double v = coordsPca / denom;
    double w = coordsPab / denom;
    glm::dvec3 weights(u, v, w);
    return {a * weights.x + b * weights.y + c * weights.z, weights, n, 0};
}

bool PreparedTriangle::intersectsRay(const glm::dvec3 &p, const glm::dvec3 &dir) const
{
    glm::dvec3 ba = b - a;
    glm::dvec3 ca = c - a;
    glm::dvec3 h = glm::cross(dir, ca);
    double proj = glm::dot(ba, h);

    if (glm::abs(proj) < std::numeric_limits<double>::epsilon())
        return false;

    proj = 1.0 / proj;
    glm::dvec3 pa = p - a;
    glm::dvec3 q = glm::cross(pa, ba);
    double u = glm::dot(pa, h) * proj;
    double v = glm::dot(dir, q) * proj;
    double t = glm::dot(ca, q) * proj;

    return u >= 0.0 && u <= 1.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0;
}

bool PreparedTriangle::intersectsAABB(const glm::dvec3 &lb, const glm::dvec3 &ub) const
{
    return glm::all(glm::greaterThanEqual(ub, lowerBound)) &&
           glm::all(glm::lessThanEqual(lb, upperBound));
}

bool PreparedTriangle::intersectsSphere(const glm::dvec3 &point, double radius) const
{
    double distance = glm::length(point - center);
    return distance < radius + this->radius;
}

bool PreparedTriangle::planeIntersectsAABB(const glm::dvec3 &lb, const glm::dvec3 &ub) const
{
    if (!intersectsAABB(lb, ub))
    {
        return false;
    }

    glm::dvec3 pcenter = (ub + lb) / 2.0;
    glm::dvec3 e = ub - pcenter;
    double r = glm::dot(e, glm::abs(n));
    double s = glm::dot(n, pcenter) - glm::dot(n, a);
    return std::abs(s) <= r;
}