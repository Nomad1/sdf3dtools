#pragma once

#define GLM_PRECISION_HIGHP_FLOAT

#include <glm/glm.hpp>
#include <vector>
#include <utility>

class PreparedTriangle {
public:
    PreparedTriangle(int id, const std::vector<glm::dvec3>& vertices, 
                                          size_t ia, size_t ib, size_t ic);

    std::tuple<glm::dvec3, glm::dvec3, glm::dvec3, int> closestPointToTriangle(const glm::dvec3& p) const;
    bool intersectsRay(const glm::dvec3& p, const glm::dvec3& dir) const;
    bool intersectsAABB(const glm::dvec3& lb, const glm::dvec3& ub) const;
    bool intersectsSphere(const glm::dvec3& point, double radius) const;
    bool planeIntersectsAABB(const glm::dvec3& lb, const glm::dvec3& ub) const;
    void setPseudoNormals(const glm::dvec3& van, const glm::dvec3& vbn, const glm::dvec3& vcn, const glm::dvec3& eab, const glm::dvec3& ebc, const glm::dvec3& eac);

    // Getters
    int getId() const { return id; }
    const glm::dvec3& getLowerBound() const { return lowerBound; }
    const glm::dvec3& getUpperBound() const { return upperBound; }
    double getArea() const { return area; }
    const glm::dvec3& getNormal() const { return n; }
    const glm::dvec3& getVertexA() const { return a; }
    const glm::dvec3& getVertexB() const { return b; }
    const glm::dvec3& getVertexC() const { return c; }    
    size_t getIndexA() const { return ia; }
    size_t getIndexB() const { return ib; }
    size_t getIndexC() const { return ic; }    
private:
    int id;
    size_t ia, ib, ic;
    glm::dvec3 a, b, c;
    glm::dvec3 n;
    glm::dvec3 van, vbn, vcn, eab, ebc, eac;
    glm::dvec3 lowerBound, upperBound, center;
    //double normalLength;
    double area;
    double radius;
};