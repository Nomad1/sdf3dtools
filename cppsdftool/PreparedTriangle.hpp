#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <utility>

class PreparedTriangle {
public:
    PreparedTriangle(int id, const std::vector<glm::vec3>& vertices, 
                                          int ia, int ib, int ic);

    std::tuple<glm::vec3, glm::vec3, glm::vec3, int> closestPointToTriangle(const glm::vec3& p) const;
    bool intersectsRay(const glm::vec3& p, const glm::vec3& dir) const;
    bool intersectsAABB(const glm::vec3& lb, const glm::vec3& ub) const;
    bool intersectsSphere(const glm::vec3& point, float radius) const;
    bool planeIntersectsAABB(const glm::vec3& lb, const glm::vec3& ub) const;
    void setPseudoNormals(const glm::vec3& van, const glm::vec3& vbn, const glm::vec3& vcn, const glm::vec3& eab, const glm::vec3& ebc, const glm::vec3& eac);

    // Getters
    int getId() const { return id; }
    const glm::vec3& getLowerBound() const { return lowerBound; }
    const glm::vec3& getUpperBound() const { return upperBound; }
    float getArea() const { return area; }
    const glm::vec3& getNormal() const { return n; }
    const glm::vec3& getVertexA() const { return a; }
    const glm::vec3& getVertexB() const { return b; }
    const glm::vec3& getVertexC() const { return c; }    
    int getIndexA() const { return ia; }
    int getIndexB() const { return ib; }
    int getIndexC() const { return ic; }    
private:
    int id;
    int ia, ib, ic;
    glm::vec3 a, b, c;
    glm::vec3 n;
    glm::vec3 van, vbn, vcn, eab, ebc, eac;
    glm::vec3 lowerBound, upperBound;
    float normalLength;
    float area;
    float radius;
};