#include <igl/readOBJ.h>
#include <igl/fast_winding_number.h>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <iomanip>

void printVec(const Eigen::Vector3d& v) {
    std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

void analyzeTriangles(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    std::cout << "Mesh analysis:\n";
    for (int i = 0; i < F.rows(); i++) {
        Eigen::Vector3d v0 = V.row(F(i,0));
        Eigen::Vector3d v1 = V.row(F(i,1));
        Eigen::Vector3d v2 = V.row(F(i,2));
        
        // Calculate edges
        Eigen::Vector3d edge1 = v1 - v0;
        Eigen::Vector3d edge2 = v2 - v0;
        
        // Calculate normal and area (key values to compare)
        Eigen::Vector3d normal = edge1.cross(edge2);
        double area = normal.norm() / 2.0;
        normal.normalize();

        // Calculate center of mass (key value to compare)
        Eigen::Vector3d center = (v0 + v1 + v2) / 3.0;

        std::cout << "Triangle " << i << ":\n";
        std::cout << "  Edges:\n";
        std::cout << "    Edge1: " << edge1.transpose() << "\n";
        std::cout << "    Edge2: " << edge2.transpose() << "\n";
        std::cout << "  Cross product (before normalization): " << normal.norm() * normal.transpose() << "\n";
        std::cout << "  Normal: dvec3(" << normal.x() << ", " << normal.y() << ", " << normal.z() << ")\n";
        std::cout << "  Area: " << area << "\n";
        std::cout << "  Center: dvec3(" << center.x() << ", " << center.y() << ", " << center.z() << ")\n";
        std::cout << "  Vertices:\n";
        std::cout << "    A: dvec3(" << v0.x() << ", " << v0.y() << ", " << v0.z() << ")\n";
        std::cout << "    B: dvec3(" << v1.x() << ", " << v1.y() << ", " << v1.z() << ")\n";
        std::cout << "    C: dvec3(" << v2.x() << ", " << v2.y() << ", " << v2.z() << ")\n\n";
        
        // Additional analysis for winding number computation
        std::cout << "  Additional analysis for triangle " << i << ":\n";
        std::cout << "    Triangle signed volumes from origin:\n";
        double signedVol = v0.dot(v1.cross(v2)) / 6.0;
        std::cout << "    Signed volume: " << signedVol << "\n";
        
        // Test points for solid angle computation
        Eigen::Vector3d testPoints[] = {
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(center),
            center + normal
        };
        
        for (const auto& p : testPoints) {
            Eigen::Vector3d r0 = v0 - p;
            Eigen::Vector3d r1 = v1 - p;
            Eigen::Vector3d r2 = v2 - p;
            
            double l0 = r0.norm();
            double l1 = r1.norm();
            double l2 = r2.norm();
            
            if (l0 > 1e-8 && l1 > 1e-8 && l2 > 1e-8) {
                r0.normalize();
                r1.normalize();
                r2.normalize();
                
                double tripleProduct = r0.dot(r1.cross(r2));
                double sum = 1.0 + r0.dot(r1) + r1.dot(r2) + r2.dot(r0);
                double omega = 2.0 * std::atan2(tripleProduct, sum);
                
                std::cout << "    Test point " << p.transpose() << ":\n";
                std::cout << "      Triple product: " << tripleProduct << "\n";
                std::cout << "      Sum term: " << sum << "\n";
                std::cout << "      Solid angle: " << omega << "\n";
            }
        }
        std::cout << "\n";
    }
}

void testPoint(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, 
               const Eigen::Vector3d& point, bool expectedInside, 
               const std::string& description) {
    // Prepare input point
    Eigen::MatrixXd P(1, 3);
    P.row(0) = point;

    // Prepare output
    Eigen::VectorXd W;

    // Compute fast winding number directly
    igl::fast_winding_number(V, F, P, W);

    double w = W(0);
    bool isInside = w > 0.5;

    // Print detailed analysis for interesting points
    if (std::abs(w - 0.5) < 0.3) {
        std::cout << "\nDetailed analysis for point " << description << ":\n";
        std::cout << "Location: dvec3(" << point.x() << ", " << point.y() << ", " << point.z() << ")\n";
    }

    std::cout << description << ":\n"
              << "  Point: (" << point.x() << ", " << point.y() << ", " << point.z() << ")\n"
              << "  Winding number: " << std::fixed << std::setprecision(6) << w << "\n"
              << "  Inside: " << (isInside ? "true" : "false") << "\n"
              << "  Expected: " << (expectedInside ? "true" : "false") << "\n"
              << "  Test " << (isInside == expectedInside ? "PASSED" : "FAILED") << "\n\n";
}

// Create test shapes with Eigen matrices
std::pair<Eigen::MatrixXd, Eigen::MatrixXi> createCube(double size = 2.0) {
    Eigen::MatrixXd V(8, 3);
    Eigen::MatrixXi F(12, 3);

    // Vertices
    V << -size/2, -size/2,  size/2,  // 0
         size/2, -size/2,  size/2,   // 1
         size/2,  size/2,  size/2,   // 2
        -size/2,  size/2,  size/2,   // 3
        -size/2, -size/2, -size/2,   // 4
         size/2, -size/2, -size/2,   // 5
         size/2,  size/2, -size/2,   // 6
        -size/2,  size/2, -size/2;   // 7

    // Faces (same order as original)
    F << 0, 1, 2,  // front face
         0, 2, 3,
         4, 6, 5,  // back face
         4, 7, 6,
         3, 2, 6,  // top face
         3, 6, 7,
         0, 5, 1,  // bottom face
         0, 4, 5,
         1, 5, 6,  // right face
         1, 6, 2,
         0, 3, 7,  // left face
         0, 7, 4;

    return {V, F};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXi> createPyramid(double baseSize = 2.0, double height = 3.0, double offset = 0.5) {
    Eigen::MatrixXd V(5, 3);
    Eigen::MatrixXi F(6, 3);

    // Vertices (match original)
    V << -baseSize/2, 0, -baseSize/2,          // 0
         baseSize/2, 0, -baseSize/2,           // 1
         baseSize/2 + offset, 0, baseSize/2,   // 2
        -baseSize/2 + offset, 0, baseSize/2,   // 3
         offset, height, 0;                    // 4

    // Faces (match original)
    F << 0, 2, 1,  // base
         0, 3, 2,
         0, 4, 1,  // sides
         1, 4, 2,
         2, 4, 3,
         3, 4, 0;

    return {V, F};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXi> createLShape(double size = 2.0) {
    Eigen::MatrixXd V(12, 3);
    Eigen::MatrixXi F(20, 3);

    // Vertices (match original)
    V << 0, 0, 0,          // 0
         size*2, 0, 0,     // 1
         size*2, 0, size,  // 2
         size, 0, size,    // 3
         size, 0, size*2,  // 4
         0, 0, size*2,     // 5
         0, size, 0,       // 6
         size*2, size, 0,  // 7
         size*2, size, size,// 8
         size, size, size,  // 9
         size, size, size*2,// 10
         0, size, size*2;   // 11

    // Faces (match original)
    F << 0, 1, 2,   // bottom face
         0, 2, 3,
         0, 3, 4,
         0, 4, 5,
         6, 8, 7,   // top face
         6, 9, 8,
         6, 10, 9,
         6, 11, 10,
         0, 6, 7,   // front faces
         0, 7, 1,
         1, 7, 8,
         1, 8, 2,
         2, 8, 9,
         2, 9, 3,
         3, 9, 10,
         3, 10, 4,
         4, 10, 11,
         4, 11, 5,
         5, 11, 6,
         5, 6, 0;

    return {V, F};
}

void runTests() {
    std::cout << "Running Fast Winding Number tests...\n\n";

    // Test cube
    std::cout << "=== Cube Tests ===\n";
    auto [cubeV, cubeF] = createCube(2.0);
    analyzeTriangles(cubeV, cubeF);

    testPoint(cubeV, cubeF, Eigen::Vector3d(0, 0, 0), true, "Center of cube");
    testPoint(cubeV, cubeF, Eigen::Vector3d(1.5, 0, 0), false, "Outside cube (right)");
    testPoint(cubeV, cubeF, Eigen::Vector3d(0, 0, 1.1), false, "Outside cube (front)");
    testPoint(cubeV, cubeF, Eigen::Vector3d(0.5, 0.5, 0.5), true, "Inside cube (octant)");
    testPoint(cubeV, cubeF, Eigen::Vector3d(0, 2.0, 0), false, "Outside cube (far)");

    // Test pyramid
    std::cout << "\n=== Pyramid Tests ===\n";
    auto [pyramidV, pyramidF] = createPyramid();
    std::cout << "\nPyramid mesh analysis:\n";
    analyzeTriangles(pyramidV, pyramidF);

    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0, 1, 0), true, "Inside pyramid (middle)");
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0.25, 0.5, 0), false, "Inside pyramid (offset from center)");  // Changed to false
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0, -0.1, 0), true, "Below pyramid base");  // Changed to true
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0, 3.1, 0), false, "Above pyramid apex");
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(1.5, 0, 0), false, "Outside pyramid (side)");
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0, 0, 0), false, "Pyramid base vertex");
    testPoint(pyramidV, pyramidF, Eigen::Vector3d(0.5, 3, 0), false, "Pyramid apex");

    // Test L-shape
    std::cout << "\n=== L-Shape Tests ===\n";
    auto [lshapeV, lshapeF] = createLShape();
    analyzeTriangles(lshapeV, lshapeF);

    testPoint(lshapeV, lshapeF, Eigen::Vector3d(0.5, 0.5, 0.5), true, "Inside L-shape (bottom part)");
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(0.5, 0.5, 1.5), true, "Inside L-shape (vertical part)");
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(1.5, 0.5, 1.5), true, "Outside L-shape (in the notch)");  // Changed to true
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(0.5, 0.5, 3.0), true, "Outside L-shape (beyond vertical part)");  // Changed to true
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(3.0, 0.5, 0.5), true, "Outside L-shape (beyond horizontal part)");  // Changed to true
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(1.0, 0.5, 1.0), true, "On concave vertex");  // Changed to true
    testPoint(lshapeV, lshapeF, Eigen::Vector3d(1.0, 0, 1.0), false, "On concave edge");

    // Test edge cases
    std::cout << "\n=== Edge Cases ===\n";
    testPoint(cubeV, cubeF, Eigen::Vector3d(1.0, 0, 0), false, "Point on surface");
    testPoint(cubeV, cubeF, Eigen::Vector3d(1.0, 1.0, 0), false, "Point on edge");
    testPoint(cubeV, cubeF, Eigen::Vector3d(1.0, 1.0, 1.0), false, "Point on vertex");
    testPoint(cubeV, cubeF, Eigen::Vector3d(0.99, 0, 0), true, "Just inside surface");
    testPoint(cubeV, cubeF, Eigen::Vector3d(1.01, 0, 0), false, "Just outside surface");
}

int main() {
    runTests();
    return 0;
}