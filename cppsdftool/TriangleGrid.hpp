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

class TriangleGrid {
public:
    struct FindTrianglesResult {
        double distance;
        glm::dvec3 weights;
        int triangleId;
    };

    TriangleGrid(const glm::dvec3& sceneMin, const glm::dvec3& sceneMax,
                int gridX, int gridY, int gridZ,
                const std::vector<PreparedTriangle>& triangles);

    FindTrianglesResult findTriangles(const glm::dvec3& point, const int quality);
    std::vector<double> dispatch(const glm::dvec3& lowerBound, double pixelsToScene, 
                              double sceneToPixels, int sx, int sy, int sz, const int quality);
    int getTriangleCount() const;
    const glm::ivec3 & getGridSize() const;
private:
    using GridCell = std::unique_ptr<std::vector<PreparedTriangle>>;

    bool processRay(const glm::dvec3& fromPoint, const glm::dvec3& toPoint, 
                   const std::function<void(int)>& action);
    int countIntersections(const glm::dvec3& point, const glm::dvec3& dir);
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

    std::vector<GridCell> grid;
    std::vector<float> emptyCells;

    std::vector<glm::ivec3> cellOffsets;
    std::vector<double> cellOffsetLengths;

    std::vector<glm::dvec3> directions;
};