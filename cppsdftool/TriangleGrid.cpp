#include "TriangleGrid.hpp"

namespace {
    // Pre-computed immediate neighbor offsets (26 adjacent cells)
    const std::vector<glm::ivec3> NEIGHBOR_OFFSETS = {
        // Same layer (z=0)
        {-1, -1, 0}, {0, -1, 0}, {1, -1, 0},
        {-1,  0, 0}, {0,  0, 0}, {1,  0, 0},
        {-1,  1, 0}, {0,  1, 0}, {1,  1, 0},
        // Layer below (z=-1)
        {-1, -1, -1}, {0, -1, -1}, {1, -1, -1},
        {-1,  0, -1}, {0,  0, -1}, {1,  0, -1},
        {-1,  1, -1}, {0,  1, -1}, {1,  1, -1},
        // Layer above (z=1)
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
        {-1,  0, 1}, {0,  0, 1}, {1,  0, 1},
        {-1,  1, 1}, {0,  1, 1}, {1,  1, 1}
    };

    bool rayBoundIntersection(const glm::vec3& lb, const glm::vec3& ub,
                            const glm::vec3& point, const glm::vec3& idir) {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        if (idir.x >= 0) {
            tmin = (lb.x - point.x) * idir.x;
            tmax = (ub.x - point.x) * idir.x;
        } else {
            tmin = (ub.x - point.x) * idir.x;
            tmax = (lb.x - point.x) * idir.x;
        }

        if (idir.y >= 0) {
            tymin = (lb.y - point.y) * idir.y;
            tymax = (ub.y - point.y) * idir.y;
        } else {
            tymin = (ub.y - point.y) * idir.y;
            tymax = (lb.y - point.y) * idir.y;
        }

        if (tmin > tymax || tmax < tymin) return false;

        tmin = std::max(tmin, tymin);
        tmax = std::min(tmax, tymax);

        if (idir.z >= 0) {
            tzmin = (lb.z - point.z) * idir.z;
            tzmax = (ub.z - point.z) * idir.z;
        } else {
            tzmin = (ub.z - point.z) * idir.z;
            tzmax = (lb.z - point.z) * idir.z;
        }

        return tmin <= tzmax && tmax >= tzmin;
    }

    std::tuple<bool, float, float> segmentBoundIntersection(
        const glm::vec3& lb, const glm::vec3& ub,
        const glm::vec3& point, const glm::vec3& idir, float length) {
        
        float resultEnter = 0.0f;
        float resultExit = length;
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        if (idir.x >= 0) {
            tmin = (lb.x - point.x) * idir.x;
            tmax = (ub.x - point.x) * idir.x;
        } else {
            tmin = (ub.x - point.x) * idir.x;
            tmax = (lb.x - point.x) * idir.x;
        }

        if (idir.y >= 0) {
            tymin = (lb.y - point.y) * idir.y;
            tymax = (ub.y - point.y) * idir.y;
        } else {
            tymin = (ub.y - point.y) * idir.y;
            tymax = (lb.y - point.y) * idir.y;
        }

        if (tmin > tymax || tmax < tymin) 
            return {false, resultEnter, resultExit};

        tmin = std::max(tmin, tymin);
        tmax = std::min(tmax, tymax);

        if (idir.z >= 0) {
            tzmin = (lb.z - point.z) * idir.z;
            tzmax = (ub.z - point.z) * idir.z;
        } else {
            tzmin = (ub.z - point.z) * idir.z;
            tzmax = (lb.z - point.z) * idir.z;
        }

        if (tmin > tzmax || tmax < tzmin)
            return {false, resultEnter, resultExit};

        tmin = std::max(tmin, tzmin);
        tmax = std::min(tmax, tzmax);

        if (tmin <= 0 && tmax >= length)
            return {true, resultEnter, resultExit};

        if (tmax < 0 || tmin > length)
            return {false, resultEnter, resultExit};

        if (tmin > 0) resultEnter = tmin;
        if (tmax < length) resultExit = tmax;

        return {true, resultEnter, resultExit};
    }
}

TriangleGrid::TriangleGrid(const glm::vec3& sceneMin, const glm::vec3& sceneMax,
                          int gridX, int gridY, int gridZ,
                          const std::vector<PreparedTriangle>& triangles)
    : sceneMin(sceneMin), gridX(gridX), gridY(gridY), gridZ(gridZ) {
    
    gridStep = std::max(
        std::max((sceneMax.x - sceneMin.x) / gridX,
                (sceneMax.y - sceneMin.y) / gridY),
        (sceneMax.z - sceneMin.z) / gridZ
    );

    triangleCount = triangles.size();
    triangleInstances = 0;
    cellsUsed = 0;

    // Initialize grid
    grid.resize(gridX * gridY * gridZ);

    // Generate ordered cell offsets
    generateOrderedOffsets();

    // Distribute triangles to grid cells
    for (const auto& triangle : triangles) {
        glm::vec3 lb = (triangle.getLowerBound() - sceneMin) / gridStep;
        int fromX = std::max(static_cast<int>(std::floor(lb.x)), 0);
        int fromY = std::max(static_cast<int>(std::floor(lb.y)), 0);
        int fromZ = std::max(static_cast<int>(std::floor(lb.z)), 0);

        glm::vec3 ub = (triangle.getUpperBound() - sceneMin) / gridStep;
        int toX = std::min(static_cast<int>(std::ceil(ub.x)), gridX - 1);
        int toY = std::min(static_cast<int>(std::ceil(ub.y)), gridY - 1);
        int toZ = std::min(static_cast<int>(std::ceil(ub.z)), gridZ - 1);

        int instances = 0;

        // Check each potential grid cell
        for (int z = fromZ; z <= toZ; ++z) {
            for (int y = fromY; y <= toY; ++y) {
                for (int x = fromX; x <= toX; ++x) {
                    glm::vec3 tileStart = glm::vec3(x, y, z) * gridStep + sceneMin;
                    glm::vec3 tileEnd = tileStart + glm::vec3(gridStep);

                    if (!triangle.planeIntersectsAABB(tileStart, tileEnd)) {
                        continue;
                    }

                    size_t index = x + y * gridX + z * gridX * gridY;

                    if (!grid[index]) {
                        grid[index] = std::make_unique<std::vector<PreparedTriangle>>();
                        cellsUsed++;
                    }

                    grid[index]->push_back(triangle);
                    instances++;
                }
            }
        }

        triangleInstances += instances;
    }

    // Sort triangles by area in descending order
    for (auto& cell : grid) {
        if (cell) {
            std::sort(cell->begin(), cell->end(),
                     [](const PreparedTriangle& a, const PreparedTriangle& b) {
                         return a.getArea() > b.getArea();
                     });
        }
    }
}

void TriangleGrid::generateOrderedOffsets() {
    // Start with immediate neighbors
    cellOffsets = NEIGHBOR_OFFSETS;
    
    // Generate additional offsets for larger grids
    if (std::max({gridX, gridY, gridZ}) > 1) {
        std::vector<glm::ivec3> additionalOffsets;
        for (int z = -gridZ; z < gridZ; ++z) {
            for (int y = -gridY; y < gridY; ++y) {
                for (int x = -gridX; x < gridX; ++x) {
                    if (std::max({std::abs(x), std::abs(y), std::abs(z)}) > 1) {
                        additionalOffsets.emplace_back(x, y, z);
                    }
                }
            }
        }

        if (!additionalOffsets.empty()) {
            // Sort by distance from center
            std::sort(additionalOffsets.begin(), additionalOffsets.end(),
                     [](const glm::ivec3& a, const glm::ivec3& b) {
                         return glm::length(glm::vec3(a)) < glm::length(glm::vec3(b));
                     });

            // Combine with immediate neighbors
            cellOffsets.insert(cellOffsets.end(), 
                             additionalOffsets.begin(), additionalOffsets.end());
        }
    }

    // Calculate offset lengths
    offsetLengths.resize(cellOffsets.size());
    std::transform(cellOffsets.begin(), cellOffsets.end(), offsetLengths.begin(),
                  [](const glm::ivec3& offset) {
                      return glm::length(glm::vec3(offset));
                  });
}

std::vector<float> TriangleGrid::dispatch(const glm::vec3& lowerBound, float pixelsToScene,
                                        float sceneToPixels, int sx, int sy, int sz) {
    int maxCount = sx * sy * sz;
    std::vector<float> result(maxCount * 4);

    for (int i = 0; i < maxCount; ++i) {
        int iz = i / (sx * sy);
        int iy = (i % (sx * sy)) / sx;
        int ix = (i % (sx * sy)) % sx;

        glm::vec3 point = lowerBound + glm::vec3(ix, iy, iz) * pixelsToScene;
        
        auto [distance, weights, triangleId] = findTriangles(point);
        float pixelDistance = distance * sceneToPixels;

        result[i * 4] = pixelDistance;
        result[i * 4 + 1] = weights.x;
        result[i * 4 + 2] = weights.y;
        result[i * 4 + 3] = static_cast<float>(triangleId);
    }

    return result;
}

int TriangleGrid::countIntersections(const glm::vec3& point, const glm::vec3& dir) {
    int count = 0;
    glm::vec3 idir = glm::vec3(
        dir.x != 0 ? 1.0f / dir.x : std::numeric_limits<float>::infinity(),
        dir.y != 0 ? 1.0f / dir.y : std::numeric_limits<float>::infinity(),
        dir.z != 0 ? 1.0f / dir.z : std::numeric_limits<float>::infinity()
    );

    // Convert to grid coordinates
    glm::vec3 localPoint = (point - sceneMin) / gridStep;
    glm::vec3 gridMax(gridX, gridY, gridZ);
    float lengthMax = glm::length(gridMax);

    auto [intersects, boundEnter, boundExit] = segmentBoundIntersection(
        glm::vec3(0), gridMax, localPoint, idir, lengthMax);

    if (!intersects) return 0;

    glm::vec3 localEndPoint = localPoint + dir * boundExit;

    std::set<const PreparedTriangle*> triangles;

    auto processCell = [&](size_t index) {
        if (!grid[index]) return;

        for (const auto& triangle : *grid[index]) {
            if (triangles.insert(&triangle).second) {
                if (!rayBoundIntersection(triangle.getLowerBound(), 
                                        triangle.getUpperBound(), point, idir)) {
                    continue;
                }

                if (triangle.intersectsRay(point, dir)) {
                    count++;
                }
            }
        }
    };

    processRay(localPoint, localEndPoint, processCell);
    return count;
}

bool TriangleGrid::processRay(const glm::vec3& fromPoint, const glm::vec3& toPoint,
                            const std::function<void(size_t)>& action) {
    glm::ivec3 fromTile = glm::floor(glm::max(fromPoint, glm::vec3(0.0f)));
    glm::ivec3 toTile = glm::floor(glm::min(toPoint, 
        glm::vec3(gridX - 1, gridY - 1, gridZ - 1)));

    glm::ivec3 step = glm::sign(glm::vec3(toTile - fromTile));

    // Handle case when ray stays in one cell
    if (glm::all(glm::equal(step, glm::ivec3(0)))) {
        if (glm::all(glm::greaterThanEqual(fromTile, glm::ivec3(0))) &&
            glm::all(glm::lessThan(fromTile, glm::ivec3(gridX, gridY, gridZ)))) {
            size_t index = fromTile.x + fromTile.y * gridX + fromTile.z * gridX * gridY;
            action(index);
            return true;
        }
        return false;
    }

    // Calculate normalized direction
    glm::vec3 dir = glm::vec3(toTile - fromTile);
    float dirLength = glm::length(dir);
    if (dirLength > 0) {
        dir /= dirLength;
    }

    // Calculate deltas and initial intersection points
    glm::vec3 delta;
    for (int i = 0; i < 3; ++i) {
        delta[i] = std::abs(dir[i]) > std::numeric_limits<float>::epsilon() ?
            1.0f / std::abs(dir[i]) : std::numeric_limits<float>::infinity();
    }

    glm::vec3 point(std::numeric_limits<float>::infinity());
    for (int i = 0; i < 3; ++i) {
        if (delta[i] != std::numeric_limits<float>::infinity()) {
            point[i] = (fromTile[i] - fromPoint[i]) / dir[i];
            if (dir[i] > 0) point[i] += delta[i];
        }
    }

    // Initialize current position
    glm::ivec3 current = fromTile;
    size_t index = current.x + current.y * gridX + current.z * gridX * gridY;

    // Process cells along ray
    int maxSteps = std::max({gridX, gridY, gridZ}) + 2;

    while (maxSteps > 0) {
        maxSteps--;

        if (glm::all(glm::greaterThanEqual(current, glm::ivec3(0))) &&
            glm::all(glm::lessThan(current, glm::ivec3(gridX, gridY, gridZ)))) {
            action(index);
        } else {
            break;
        }

        // Find next cell crossing
        int nextCross = 0;
        float minPoint = point.x;
        if (point.y < minPoint) {
            minPoint = point.y;
            nextCross = 1;
        }
        if (point.z < minPoint) {
            nextCross = 2;
        }

        if (current[nextCross] == toTile[nextCross]) {
            break;
        }

        point[nextCross] += delta[nextCross];
        current[nextCross] += step[nextCross];

        if (nextCross == 0) index += step.x;
        else if (nextCross == 1) index += step.y * gridX;
        else index += step.z * gridX * gridY;
    }

    return false;
}

TriangleGrid::FindTrianglesResult TriangleGrid::findTriangles(const glm::vec3& point) {
    FindTrianglesResult result;
    result.distance = std::numeric_limits<float>::infinity();
    result.triangleId = -1;
    result.weights = glm::vec3(0.0f);

    float minDistanceSqrd = std::numeric_limits<float>::infinity();
    const PreparedTriangle* closestTriangle = nullptr;

    // Convert point to grid coordinates
    glm::vec3 localPoint = (point - sceneMin) / gridStep;
    int pointX = static_cast<int>(std::floor(localPoint.x));
    int pointY = static_cast<int>(std::floor(localPoint.y));
    int pointZ = static_cast<int>(std::floor(localPoint.z));

    std::set<const PreparedTriangle*> triangles;
    float localDist = std::numeric_limits<float>::infinity();
    glm::vec3 lb(0.0f), ub(0.0f);
    bool earlyExit = false;

    // Check cells in order of increasing distance
    for (size_t i = 0; i < cellOffsets.size(); ++i) {
        if (offsetLengths[i] > localDist) {
            break;
        }

        const auto& offset = cellOffsets[i];
        int x = pointX + offset.x;
        int y = pointY + offset.y;
        int z = pointZ + offset.z;

        if (x < 0 || x >= gridX || y < 0 || y >= gridY || z < 0 || z >= gridZ) {
            continue;
        }

        size_t index = x + y * gridX + z * gridX * gridY;

        // Early exit for isolated points
        if (i >= 26 && localDist == std::numeric_limits<float>::infinity()) {
            earlyExit = true;
            result.distance = offsetLengths[i] * gridStep;
            break;
        }

        if (!grid[index]) {
            continue;
        }

        for (const auto& triangle : *grid[index]) {
            if (triangles.insert(&triangle).second) {
                if (result.distance != std::numeric_limits<float>::infinity()) {
                    if (!triangle.intersectsAABB(lb, ub)) continue;
                    if (!triangle.intersectsSphere(point, result.distance)) continue;
                }

                auto [closestPoint, weights] = triangle.closestPointToTriangle(point);
                glm::vec3 dir = point - closestPoint;
                float dist = glm::dot(dir, dir);

                if (dist < minDistanceSqrd) {
                    minDistanceSqrd = dist;
                    result.distance = std::sqrt(dist);
                    
                    lb = point - glm::vec3(result.distance);
                    ub = point + glm::vec3(result.distance);
                    
                    result.weights = weights;
                    closestTriangle = &triangle;
                }
            }
        }

        if (result.distance != std::numeric_limits<float>::infinity()) {
            localDist = result.distance / gridStep + std::sqrt(3.0f) / 2.0f;
        }

        if (earlyExit) break;
    }

    // Determine sign using ray casting
    if (!earlyExit && closestTriangle) {
        glm::vec3 direction;
        if (std::isinf(result.distance)) {
            direction = point - glm::vec3(0.5f);
        } else {
            auto [closestPoint, _] = closestTriangle->closestPointToTriangle(point);
            direction = point - closestPoint;
        }
        direction = glm::normalize(direction);

        std::vector<glm::vec3> testDirections = {
            direction,
            {0, 0, 1}, {1, 0, 0}, {0, 1, 0},
            {0, 0, -1}, {-1, 0, 0}, {0, -1, 0}
        };

        int sign = 0;
        for (const auto& dir : testDirections) {
            sign += (countIntersections(point, dir) % 2 == 0) ? 1 : -1;
            if (std::abs(sign) >= static_cast<int>(testDirections.size()) / 2 + 1) {
                break;
            }
        }

        result.distance *= (sign >= 0) ? 1.0f : -1.0f;
    }

    result.triangleId = closestTriangle ? closestTriangle->getId() : -1;
    return result;
}