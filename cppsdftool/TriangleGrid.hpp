#pragma once

#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <set>
#include <limits>

#include "PreparedTriangle.hpp"

class FastWindingNumber;

class TriangleGrid
{
public:
    struct FindTrianglesResult
    {
        double distance;
        glm::dvec3 weights;
        int triangleId;
    };

    TriangleGrid(const glm::dvec3 &sceneMin, const glm::dvec3 &sceneMax,
                 int gridX, int gridY, int gridZ,
                 const std::vector<PreparedTriangle> &triangles);

    ~TriangleGrid();

  
    /**
     * @brief Calculate signed distance values for each voxel in a 3D grid with LOD support
     *
     * @param lowerBound The lower bound coordinates of the 3D grid
     * @param pixelsToScene Scale factor to convert from pixels to scene units
     * @param sx Width of the grid in voxels
     * @param sy Height of the grid in voxels
     * @param sz Depth of the grid in voxels
     * @param quality The quality level for SDF computation (1-4)
     * @param lods Number of LOD (level of detail) levels to generate
     * @return Vector of vectors containing signed distance values for each LOD level
     */
    std::vector<std::vector<double>> dispatch(const glm::dvec3 &lowerBound, double pixelsToScene, double sceneToPixels,
                                              int sx, int sy, int sz, const int quality,
                                              const uint lods);

    int getTriangleCount() const;
    const glm::ivec3 &getGridSize() const;

private:
    FindTrianglesResult findTriangles(const glm::dvec3 &point, const int quality);

    using GridCell = std::unique_ptr<std::vector<PreparedTriangle>>;

    bool processRay(const glm::dvec3 &fromPoint, const glm::dvec3 &toPoint,
                    const std::function<void(int)> &action);
    int countIntersections(const glm::dvec3 &point, const glm::dvec3 &dir);
    void generateOrderedOffsets();
    void generateRandomDirections();

    glm::dvec3 sceneMin;
    glm::dvec3 sceneMax;
    double gridStep;
    glm::ivec3 gridSize;
    glm::ivec3 gridIndex;
    int triangleCount;
    int triangleInstances;
    int cellsUsed;

    FastWindingNumber *fastWindingNumber;

    std::vector<GridCell> grid;
    std::vector<float> emptyCells;

    std::vector<glm::ivec3> cellOffsets;
    std::vector<double> cellOffsetLengths;

    std::vector<glm::dvec3> directions;
};