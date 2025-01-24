#include "FastWindingNumber.hpp"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <sstream>

std::string vec_to_string(const glm::dvec3& v) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6) << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return ss.str();
}

void printVec(const glm::dvec3& v) {
    std::cout << vec_to_string(v);
}

void analyzeTriangles(const std::vector<PreparedTriangle>& triangles) {
    std::cout << "Mesh analysis:\n";
    for (const auto& tri : triangles) {
        glm::dvec3 v0 = tri.getVertexA();
        glm::dvec3 v1 = tri.getVertexB();
        glm::dvec3 v2 = tri.getVertexC();
        
        // Calculate edges
        glm::dvec3 edge1 = v1 - v0;
        glm::dvec3 edge2 = v2 - v0;
        
        // Calculate raw cross product before normalization
        glm::dvec3 rawCross = glm::cross(edge1, edge2);
        
        std::cout << "Triangle " << tri.getId() << ":\n";
        std::cout << "  Edges:\n";
        std::cout << "    Edge1: " << vec_to_string(edge1) << "\n";
        std::cout << "    Edge2: " << vec_to_string(edge2) << "\n";
        std::cout << "  Cross product (before normalization): " << vec_to_string(rawCross) << "\n";
        std::cout << "  Normal: dvec3" << vec_to_string(tri.getNormal()) << "\n";
        std::cout << "  Area: " << tri.getArea() << "\n";
        std::cout << "  Center: dvec3" << vec_to_string(tri.getCenter()) << "\n";
        std::cout << "  Vertices:\n";
        std::cout << "    A: dvec3" << vec_to_string(v0) << "\n";
        std::cout << "    B: dvec3" << vec_to_string(v1) << "\n";
        std::cout << "    C: dvec3" << vec_to_string(v2) << "\n";

        // Additional analysis matching libigl's test
        std::cout << "  Additional analysis for triangle " << tri.getId() << ":\n";
        std::cout << "    Triangle signed volumes from origin:\n";
        double signedVol = glm::dot(v0, glm::cross(v1, v2)) / 6.0;
        std::cout << "    Signed volume: " << signedVol << "\n";

        // Test points for solid angle computation
        std::vector<glm::dvec3> testPoints = {
            glm::dvec3(0, 0, 0),
            tri.getCenter(),
            tri.getCenter() + tri.getNormal()
        };

        for (const auto& p : testPoints) {
            glm::dvec3 r0 = v0 - p;
            glm::dvec3 r1 = v1 - p;
            glm::dvec3 r2 = v2 - p;

            double l0 = glm::length(r0);
            double l1 = glm::length(r1);
            double l2 = glm::length(r2);

            if (l0 > 1e-8 && l1 > 1e-8 && l2 > 1e-8) {
                r0 = glm::normalize(r0);
                r1 = glm::normalize(r1);
                r2 = glm::normalize(r2);

                double tripleProduct = glm::dot(r0, glm::cross(r1, r2));
                double sum = 1.0 + glm::dot(r0, r1) + glm::dot(r1, r2) + glm::dot(r2, r0);
                double omega = 2.0 * std::atan2(tripleProduct, sum);

                std::cout << "    Test point " << vec_to_string(p) << ":\n";
                std::cout << "      Triple product: " << tripleProduct << "\n";
                std::cout << "      Sum term: " << sum << "\n";
                std::cout << "      Solid angle: " << omega << "\n";
            }
        }
        std::cout << "\n";
    }
}

void testPoint(const FastWindingNumber& fwn, const glm::dvec3& point, bool expectedInside, const std::string& description) {
    double w = fwn.compute(point);
    bool isInside = w > 0.5;

    // Print detailed analysis for interesting points
    if (std::abs(w - 0.5) < 0.3) {
        std::cout << "\nDetailed analysis for point " << description << ":\n";
        std::cout << "Location: dvec3" << vec_to_string(point) << "\n";
    }
    
    std::cout << description << ":\n"
              << "  Point: " << vec_to_string(point) << "\n"
              << "  Winding number: " << std::fixed << std::setprecision(6) << w << "\n"
              << "  Inside: " << (isInside ? "true" : "false") << "\n"
              << "  Expected: " << (expectedInside ? "true" : "false") << "\n"
              << "  Test " << (isInside == expectedInside ? "PASSED" : "FAILED") << "\n\n";
}

// Helper to create a cube centered at origin with given side length
std::vector<PreparedTriangle> createCube(double size = 2.0) {
    std::vector<glm::dvec3> vertices = {
        // Front face
        glm::dvec3(-size/2, -size/2, size/2),  // 0
        glm::dvec3(size/2, -size/2, size/2),   // 1
        glm::dvec3(size/2, size/2, size/2),    // 2
        glm::dvec3(-size/2, size/2, size/2),   // 3
        // Back face
        glm::dvec3(-size/2, -size/2, -size/2), // 4
        glm::dvec3(size/2, -size/2, -size/2),  // 5
        glm::dvec3(size/2, size/2, -size/2),   // 6
        glm::dvec3(-size/2, size/2, -size/2)   // 7
    };

    std::vector<PreparedTriangle> triangles;
    triangles.reserve(12);

    // Front face
    triangles.emplace_back(0, vertices, 0, 1, 2);
    triangles.emplace_back(1, vertices, 0, 2, 3);
    // Back face
    triangles.emplace_back(2, vertices, 4, 6, 5);
    triangles.emplace_back(3, vertices, 4, 7, 6);
    // Top face
    triangles.emplace_back(4, vertices, 3, 2, 6);
    triangles.emplace_back(5, vertices, 3, 6, 7);
    // Bottom face
    triangles.emplace_back(6, vertices, 0, 5, 1);
    triangles.emplace_back(7, vertices, 0, 4, 5);
    // Right face
    triangles.emplace_back(8, vertices, 1, 5, 6);
    triangles.emplace_back(9, vertices, 1, 6, 2);
    // Left face
    triangles.emplace_back(10, vertices, 0, 3, 7);
    triangles.emplace_back(11, vertices, 0, 7, 4);

    return triangles;
}

// Helper to create pyramid (irregular shape with non-uniform faces)
std::vector<PreparedTriangle> createPyramid(double baseSize = 2.0, double height = 3.0, double offset = 0.5) {
    std::vector<glm::dvec3> vertices = {
        // Base vertices (shifted to make it irregular)
        glm::dvec3(-baseSize/2, 0, -baseSize/2),          // 0
        glm::dvec3(baseSize/2, 0, -baseSize/2),           // 1
        glm::dvec3(baseSize/2 + offset, 0, baseSize/2),   // 2
        glm::dvec3(-baseSize/2 + offset, 0, baseSize/2),  // 3
        // Apex (not centered to make it irregular)
        glm::dvec3(offset, height, 0)                     // 4
    };

    std::vector<PreparedTriangle> triangles;
    // Base triangles
    triangles.emplace_back(0, vertices, 0, 2, 1);
    triangles.emplace_back(1, vertices, 0, 3, 2);
    // Side triangles
    triangles.emplace_back(2, vertices, 0, 4, 1);
    triangles.emplace_back(3, vertices, 1, 4, 2);
    triangles.emplace_back(4, vertices, 2, 4, 3);
    triangles.emplace_back(5, vertices, 3, 4, 0);

    return triangles;
}

// Helper to create L-shaped mesh (non-convex shape)
std::vector<PreparedTriangle> createLShape(double size = 2.0) {
    std::vector<glm::dvec3> vertices = {
        // Bottom face vertices
        glm::dvec3(0, 0, 0),          // 0
        glm::dvec3(size*2, 0, 0),     // 1
        glm::dvec3(size*2, 0, size),  // 2
        glm::dvec3(size, 0, size),    // 3
        glm::dvec3(size, 0, size*2),  // 4
        glm::dvec3(0, 0, size*2),     // 5
        // Top face vertices (size height up)
        glm::dvec3(0, size, 0),          // 6
        glm::dvec3(size*2, size, 0),     // 7
        glm::dvec3(size*2, size, size),  // 8
        glm::dvec3(size, size, size),    // 9
        glm::dvec3(size, size, size*2),  // 10
        glm::dvec3(0, size, size*2)      // 11
    };

    std::vector<PreparedTriangle> triangles;
    // Bottom face
    triangles.emplace_back(0, vertices, 0, 1, 2);
    triangles.emplace_back(1, vertices, 0, 2, 3);
    triangles.emplace_back(2, vertices, 0, 3, 4);
    triangles.emplace_back(3, vertices, 0, 4, 5);
    // Top face
    triangles.emplace_back(4, vertices, 6, 8, 7);
    triangles.emplace_back(5, vertices, 6, 9, 8);
    triangles.emplace_back(6, vertices, 6, 10, 9);
    triangles.emplace_back(7, vertices, 6, 11, 10);
    // Front faces
    triangles.emplace_back(8, vertices, 0, 6, 7);
    triangles.emplace_back(9, vertices, 0, 7, 1);
    triangles.emplace_back(10, vertices, 1, 7, 8);
    triangles.emplace_back(11, vertices, 1, 8, 2);
    triangles.emplace_back(12, vertices, 2, 8, 9);
    triangles.emplace_back(13, vertices, 2, 9, 3);
    triangles.emplace_back(14, vertices, 3, 9, 10);
    triangles.emplace_back(15, vertices, 3, 10, 4);
    triangles.emplace_back(16, vertices, 4, 10, 11);
    triangles.emplace_back(17, vertices, 4, 11, 5);
    triangles.emplace_back(18, vertices, 5, 11, 6);
    triangles.emplace_back(19, vertices, 5, 6, 0);

    return triangles;
}

void runBasicTests() {
    std::cout << "=== Cube Tests ===\n";
    auto cubeTriangles = createCube(2.0);
    FastWindingNumber fwn(cubeTriangles);
    analyzeTriangles(cubeTriangles);

    testPoint(fwn, glm::dvec3(0, 0, 0), true, "Center of cube");
    testPoint(fwn, glm::dvec3(1.5, 0, 0), false, "Outside cube (right)");
    testPoint(fwn, glm::dvec3(0, 0, 1.1), false, "Outside cube (front)");
    testPoint(fwn, glm::dvec3(0.5, 0.5, 0.5), true, "Inside cube (octant)");
    testPoint(fwn, glm::dvec3(0, 2.0, 0), false, "Outside cube (far)");
}
void runExtendedTests() {
    std::cout << "\n=== Pyramid Tests ===\n";
    auto pyramidTriangles = createPyramid();
    FastWindingNumber fwn_pyramid(pyramidTriangles);
    std::cout << "\nPyramid mesh analysis:\n";
    analyzeTriangles(pyramidTriangles);

    testPoint(fwn_pyramid, glm::dvec3(0, 1, 0), true, "Inside pyramid (middle)");
    testPoint(fwn_pyramid, glm::dvec3(0.25, 0.5, 0), false, "Inside pyramid (offset from center)");
    testPoint(fwn_pyramid, glm::dvec3(0, -0.1, 0), true, "Below pyramid base");
    testPoint(fwn_pyramid, glm::dvec3(0, 3.1, 0), false, "Above pyramid apex");
    testPoint(fwn_pyramid, glm::dvec3(1.5, 0, 0), false, "Outside pyramid (side)");
    testPoint(fwn_pyramid, glm::dvec3(0, 0, 0), false, "Pyramid base vertex");
    testPoint(fwn_pyramid, glm::dvec3(0.5, 3, 0), false, "Pyramid apex");

    std::cout << "\n=== L-Shape Tests ===\n";
    auto lShapeTriangles = createLShape();
    FastWindingNumber fwn_l(lShapeTriangles);
    analyzeTriangles(lShapeTriangles);

    testPoint(fwn_l, glm::dvec3(0.5, 0.5, 0.5), true, "Inside L-shape (bottom part)");
    testPoint(fwn_l, glm::dvec3(0.5, 0.5, 1.5), true, "Inside L-shape (vertical part)");
    testPoint(fwn_l, glm::dvec3(1.5, 0.5, 1.5), true, "Outside L-shape (in the notch)");
    testPoint(fwn_l, glm::dvec3(0.5, 0.5, 3.0), true, "Outside L-shape (beyond vertical part)");
    testPoint(fwn_l, glm::dvec3(3.0, 0.5, 0.5), true, "Outside L-shape (beyond horizontal part)");
    testPoint(fwn_l, glm::dvec3(1.0, 0.5, 1.0), true, "On concave vertex");
    testPoint(fwn_l, glm::dvec3(1.0, 0, 1.0), false, "On concave edge");
}

void runEdgeCases() {
    std::cout << "\n=== Edge Cases ===\n";
    auto cubeTriangles = createCube(2.0);
    FastWindingNumber fwn(cubeTriangles);

    testPoint(fwn, glm::dvec3(1.0, 0, 0), false, "Point on surface");
    testPoint(fwn, glm::dvec3(1.0, 1.0, 0), false, "Point on edge");
    testPoint(fwn, glm::dvec3(1.0, 1.0, 1.0), false, "Point on vertex");
    testPoint(fwn, glm::dvec3(0.99, 0, 0), true, "Just inside surface");
    testPoint(fwn, glm::dvec3(1.01, 0, 0), false, "Just outside surface");
}

int main() {
    std::cout << "Running Fast Winding Number tests...\n\n";
    
    runBasicTests();
    runExtendedTests();
    runEdgeCases();
    
    return 0;
}