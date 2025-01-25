#pragma once

#define GLM_PRECISION_HIGHP_FLOAT

#include <glm/glm.hpp>
#include <vector>
#include <utility>

/**
 * @brief A class representing a triangle prepared for geometric calculations
 * 
 * This class encapsulates a triangle in 3D space with additional pre-computed properties
 * for efficient geometric calculations and intersection tests.
 */
class PreparedTriangle {
public:
    /**
     * @brief Constructs a prepared triangle
     * @param id Unique identifier for the triangle
     * @param vertices Vector containing all vertex positions
     * @param ia Index of first vertex in vertices array
     * @param ib Index of second vertex in vertices array
     * @param ic Index of third vertex in vertices array
     */
    PreparedTriangle(int id, const std::vector<glm::dvec3>& vertices, 
                    size_t ia, size_t ib, size_t ic);

    /**
     * @brief Calculates the closest point on the triangle to a given point
     * @param p Point to calculate distance from
     * @return Tuple containing: closest point on triangle, barycentric coordinates, and region code
     */
    std::tuple<glm::dvec3, glm::dvec3, glm::dvec3, int> closestPointToTriangle(const glm::dvec3& p) const;

    /**
     * @brief Tests if a ray intersects this triangle
     * @param p Origin point of the ray
     * @param dir Direction vector of the ray
     * @return True if ray intersects triangle, false otherwise
     */
    bool intersectsRay(const glm::dvec3& p, const glm::dvec3& dir) const;

    /**
     * @brief Tests if this triangle intersects with an axis-aligned bounding box
     * @param lb Lower bound corner of the AABB
     * @param ub Upper bound corner of the AABB
     * @return True if triangle intersects AABB, false otherwise
     */
    bool intersectsAABB(const glm::dvec3& lb, const glm::dvec3& ub) const;

    /**
     * @brief Tests if this triangle intersects with a sphere
     * @param point Center of the sphere
     * @param radius Radius of the sphere
     * @return True if triangle intersects sphere, false otherwise
     */
    bool intersectsSphere(const glm::dvec3& point, double radius) const;

    /**
     * @brief Tests if the plane containing this triangle intersects an AABB
     * @param lb Lower bound corner of the AABB
     * @param ub Upper bound corner of the AABB
     * @return True if plane intersects AABB, false otherwise
     */
    bool planeIntersectsAABB(const glm::dvec3& lb, const glm::dvec3& ub) const;

    /**
     * @brief Sets the pseudo-normals for the triangle
     * @param van Pseudo-normal for vertex A
     * @param vbn Pseudo-normal for vertex B
     * @param vcn Pseudo-normal for vertex C
     * @param eab Edge normal for edge AB
     * @param ebc Edge normal for edge BC
     * @param eac Edge normal for edge AC
     */
    void setPseudoNormals(const glm::dvec3& van, const glm::dvec3& vbn, const glm::dvec3& vcn, 
                         const glm::dvec3& eab, const glm::dvec3& ebc, const glm::dvec3& eac);

    // Getters
    int getId() const { return id; }
    const glm::dvec3& getLowerBound() const { return lowerBound; }
    const glm::dvec3& getUpperBound() const { return upperBound; }
    double getArea() const { return area; }
    const glm::dvec3& getNormal() const { return n; }
    const glm::dvec3& getVertexA() const { return a; }
    const glm::dvec3& getVertexB() const { return b; }
    const glm::dvec3& getVertexC() const { return c; }    
    const glm::dvec3& getCenter() const { return center; }
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
