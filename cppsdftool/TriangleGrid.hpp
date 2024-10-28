#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
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
        float distance;
        glm::vec3 weights;
        int triangleId;
    };

    TriangleGrid(const glm::vec3& sceneMin, const glm::vec3& sceneMax,
                 int gridX, int gridY, int gridZ,
                 const std::vector<PreparedTriangle>& triangles);

    FindTrianglesResult findTriangles(const glm::vec3& point);
    std::vector<float> dispatch(const glm::vec3& lowerBound, float pixelsToScene, 
                              float sceneToPixels, int sx, int sy, int sz);
    int getTriangleCount() const;
private:
    using GridCell = std::unique_ptr<std::vector<PreparedTriangle>>;

    bool processRay(const glm::vec3& fromPoint, const glm::vec3& toPoint, 
                   const std::function<void(size_t)>& action);
    int countIntersections(const glm::vec3& point, const glm::vec3& dir, int exceptTriangle = -1);
    void generateOrderedOffsets();

    glm::vec3 sceneMin;
    float gridStep;
    int gridX, gridY, gridZ;
    int triangleCount;
    int triangleInstances;
    int cellsUsed;

    std::vector<GridCell> grid;
    std::vector<glm::ivec3> cellOffsets;
    std::vector<float> cellOffsetLengths;
};