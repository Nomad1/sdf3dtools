#include "TriangleGrid.hpp"
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/string_cast.hpp"

namespace
{
    // Pre-computed immediate neighbor offsets (26 adjacent cells)

    bool rayBoundIntersection(const glm::dvec3 &lb, const glm::dvec3 &ub,
                              const glm::dvec3 &point, const glm::dvec3 &idir)
    {
        double tmin, tmax, tymin, tymax, tzmin, tzmax;

        if (idir.x >= 0)
        {
            tmin = (lb.x - point.x) * idir.x;
            tmax = (ub.x - point.x) * idir.x;
        }
        else
        {
            tmin = (ub.x - point.x) * idir.x;
            tmax = (lb.x - point.x) * idir.x;
        }

        if (idir.y >= 0)
        {
            tymin = (lb.y - point.y) * idir.y;
            tymax = (ub.y - point.y) * idir.y;
        }
        else
        {
            tymin = (ub.y - point.y) * idir.y;
            tymax = (lb.y - point.y) * idir.y;
        }

        if (tmin > tymax || tmax < tymin)
            return false;

        tmin = std::max(tmin, tymin);
        tmax = std::min(tmax, tymax);

        if (idir.z >= 0)
        {
            tzmin = (lb.z - point.z) * idir.z;
            tzmax = (ub.z - point.z) * idir.z;
        }
        else
        {
            tzmin = (ub.z - point.z) * idir.z;
            tzmax = (lb.z - point.z) * idir.z;
        }

        return tmin <= tzmax && tmax >= tzmin;
    }

    int idot(const glm::ivec3 &a, const glm::ivec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
}

TriangleGrid::TriangleGrid(const glm::dvec3 &sceneMin, const glm::dvec3 &sceneMax,
                           int gridX, int gridY, int gridZ,
                           const std::vector<PreparedTriangle> &triangles)
    : sceneMin(sceneMin), sceneMax(sceneMax), gridSize(gridX, gridY, gridZ), gridIndex(1, gridX, gridX * gridY)
{

    gridStep = std::max(
        std::max((sceneMax.x - sceneMin.x) / gridX,
                 (sceneMax.y - sceneMin.y) / gridY),
        (sceneMax.z - sceneMin.z) / gridZ);

    triangleCount = triangles.size();
    triangleInstances = 0;
    cellsUsed = 0;

    // Initialize grid
    grid.resize(gridX * gridY * gridZ);
    emptyCells.resize(gridX * gridY * gridZ, std::numeric_limits<double>::infinity());

    // Generate ordered cell offsets
    generateOrderedOffsets();

    // Generate directions for inside/outside raycasting check
    generateRandomDirections();

    // Distribute triangles to grid cells
    for (const auto &triangle : triangles)
    {
        //if (triangle.getArea() < 1e-20)
          //  continue;

        glm::dvec3 lb = (triangle.getLowerBound() - sceneMin) / gridStep;
        int fromX = std::max(static_cast<int>(std::floor(lb.x)), 0);
        int fromY = std::max(static_cast<int>(std::floor(lb.y)), 0);
        int fromZ = std::max(static_cast<int>(std::floor(lb.z)), 0);

        glm::dvec3 ub = (triangle.getUpperBound() - sceneMin) / gridStep;
        int toX = std::min(static_cast<int>(std::ceil(ub.x)), gridX - 1);
        int toY = std::min(static_cast<int>(std::ceil(ub.y)), gridY - 1);
        int toZ = std::min(static_cast<int>(std::ceil(ub.z)), gridZ - 1);

        int instances = 0;

        // Check each potential grid cell
        for (int z = fromZ; z <= toZ; ++z)
        {
            for (int y = fromY; y <= toY; ++y)
            {
                for (int x = fromX; x <= toX; ++x)
                {
                    glm::dvec3 tileStart = glm::dvec3(x, y, z) * gridStep + sceneMin;
                    glm::dvec3 tileEnd = tileStart + glm::dvec3(gridStep);

                    if (!triangle.planeIntersectsAABB(tileStart, tileEnd))
                    {
                        continue;
                    }

                    int index = idot(glm::ivec3(x, y, z), gridIndex);

                    if (!grid[index])
                    {
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
    for (auto &cell : grid)
    {
        if (cell)
        {
            std::sort(cell->begin(), cell->end(),
                      [](const PreparedTriangle &a, const PreparedTriangle &b)
                      {
                          return a.getArea() > b.getArea();
                      });
        }
    }
}

void TriangleGrid::generateOrderedOffsets()
{
    std::vector<glm::ivec4> offsets;
    for (int z = -gridSize.z; z < gridSize.z; ++z)
    {
        for (int y = -gridSize.y; y < gridSize.y; ++y)
        {
            for (int x = -gridSize.x; x < gridSize.x; ++x)
            {
                offsets.emplace_back(x, y, z, x * x + y * y + z * z);
            }
        }
    }

    // Sort by distance (fourth component)
    std::sort(offsets.begin(), offsets.end(),
              [](const glm::ivec4 &a, const glm::ivec4 &b)
              {
                  return a.w < b.w;
              });

    cellOffsets.reserve(offsets.size());
    cellOffsetLengths.reserve(offsets.size());

    for (const auto &offset : offsets)
    {
        cellOffsets.emplace_back(offset.x, offset.y, offset.z);
        cellOffsetLengths.push_back(std::sqrt(offset.w));
    }
}

void TriangleGrid::generateRandomDirections()
{
    // Fixed seed
    srand(42); // Use any fixed number as seed

    for (int i = 0; i < 5; i++)
    {
        // Generate random coords between -1 and 1
        double x = (rand() / double(RAND_MAX)) * 2.0 - 1.0;
        double y = (rand() / double(RAND_MAX)) * 2.0 - 1.0;
        double z = (rand() / double(RAND_MAX)) * 2.0 - 1.0;

        glm::dvec3 dir(x, y, z);
        directions.push_back(glm::normalize(dir));
    }
}

int TriangleGrid::getTriangleCount() const
{
    return triangleCount;
}

std::vector<double> TriangleGrid::dispatch(const glm::dvec3 &lowerBound, double pixelsToScene,
                                           double sceneToPixels, int sx, int sy, int sz)
{
    int maxCount = sx * sy * sz;
    std::vector<double> result;
    result.resize(maxCount * 4);

// Use OpenMP to parallelize the loop
#pragma omp parallel for schedule(dynamic) collapse(3)
    for (int iz = 0; iz < sz; ++iz)
    {
        for (int iy = 0; iy < sy; ++iy)
        {
            for (int ix = 0; ix < sx; ++ix)
            {
                int index = (iz * sy * sx + iy * sx + ix) * 4;

                glm::dvec3 point = lowerBound + glm::dvec3(ix, iy, iz) * pixelsToScene;
                auto [distance, weights, triangleId] = findTriangles(point);
                double pixelDistance = distance * sceneToPixels;

                // Store results in a thread-safe manner
                result[index] = pixelDistance;
                result[index + 1] = weights.x;
                result[index + 2] = weights.y;
                result[index + 3] = static_cast<double>(triangleId);
            }
        }
    }

    return result;
}

int TriangleGrid::countIntersections(const glm::dvec3 &point, const glm::dvec3 &dir)
{
    int count = 0;
    glm::dvec3 idir = 1.0 / dir;

    // Convert to grid coordinates
    glm::dvec3 localPoint = (point - sceneMin) / gridStep;
    // glm::dvec3 gridMax(gridSize);
    //    double lengthMax = glm::length(gridMax);

    // auto [intersects, boundEnter, boundExit] = segmentBoundIntersection(
    // glm::dvec3(0), gridMax, localPoint, idir, lengthMax);

    // if (!intersects)
    // return 0;

    // glm::dvec3 localEndPoint = localPoint + dir * boundExit;

    // glm::dvec3 localEndPoint = localPoint + dir * lengthMax;

    std::set<int> triangles;

    auto processCell = [&](int index)
    {
        if (!grid[index])
            return;

        for (const auto &triangle : *grid[index])
        {
            if (triangles.insert(triangle.getId()).second)
            {
                if (!rayBoundIntersection(triangle.getLowerBound(),
                                          triangle.getUpperBound(), point, idir))
                    continue;

                if (triangle.intersectsRay(point, dir))
                    count++;
            }
        }
    };

    processRay(localPoint, dir, processCell);

    return count;
}

bool TriangleGrid::processRay(const glm::dvec3 &fromPoint, const glm::dvec3 &dir,
                              const std::function<void(int)> &action)
{
    glm::ivec3 fromTile = glm::max((glm::ivec3)glm::floor(fromPoint), glm::ivec3(0));

    // Calculate normalized direction
    const glm::ivec3 step = glm::sign(dir);

    // Calculate deltas and initial intersection points
    glm::dvec3 delta;
    for (int i = 0; i < 3; ++i)
        delta[i] = std::abs(dir[i]) > std::numeric_limits<double>::epsilon() ? 1.0 / std::abs(dir[i]) : std::numeric_limits<double>::infinity();

    glm::dvec3 point(std::numeric_limits<double>::infinity());
    for (int i = 0; i < 3; ++i)
        if (!std::isinf(delta[i]))
        {
            point[i] = (fromTile[i] - fromPoint[i]) / dir[i];
            if (dir[i] > 0)
                point[i] += delta[i];
        }

    // Initialize current position
    glm::ivec3 current = fromTile;
    int index = idot(current, gridIndex);

    // Process cells along ray
    int maxSteps = gridSize.x + gridSize.y + gridSize.z;

    const int indexOffset[] = {step.x, step.y * gridSize.x, step.z * gridSize.x * gridSize.y};

    while (maxSteps > 0)
    {
        maxSteps--;

        if (glm::all(glm::greaterThanEqual(current, glm::ivec3(0))) && glm::all(glm::lessThan(current, gridSize)))
        {
            action(index);
        }
        else
        {
            break;
        }

        // Find next cell crossing
        int nextCross = 0;
        double minPoint = point.x;
        if (point.y < minPoint)
        {
            minPoint = point.y;
            nextCross = 1;
        }
        if (point.z < minPoint)
        {
            nextCross = 2;
        }

        // if (current[nextCross] == toTile[nextCross])
        // {
        //     break;
        // }

        point[nextCross] += delta[nextCross];
        current[nextCross] += step[nextCross];
        index += indexOffset[nextCross];
    }

    return false;
}

TriangleGrid::FindTrianglesResult TriangleGrid::findTriangles(const glm::dvec3 &point)
{
    FindTrianglesResult result;
    result.distance = std::numeric_limits<double>::infinity();
    result.triangleId = -1;
    result.weights = glm::dvec3(0.0);

    glm::dvec3 resultPoint;
    glm::dvec3 resultNormal;

    double minDistanceSqrd = std::numeric_limits<double>::infinity();
    const PreparedTriangle *resultTriangle = nullptr;
    int resultCode = -1;

    // Convert point to grid coordinates
    glm::dvec3 localPoint = (point - sceneMin) / gridStep;

    glm::ivec3 ipoint(glm::floor(localPoint));
    ipoint = glm::max(ipoint, glm::ivec3(0.0));
    ipoint = glm::min(ipoint, glm::ivec3(gridSize - 1));

    auto emptyValue = emptyCells[idot(ipoint, gridIndex)];

    if (!std::isinf(emptyValue))
    {
        result.distance = emptyValue;
        return result;
    }

    std::set<int> triangles;
    double localDist = std::numeric_limits<double>::infinity();
    glm::dvec3 lb(0.0), ub(0.0);
    bool earlyExit = false;

    // Check cells in order of increasing distance
    for (size_t i = 0; i < cellOffsets.size(); ++i)
    {
        if (cellOffsetLengths[i] > localDist)
            break;

        const auto &offset = cellOffsets[i];
        glm::ivec3 p = ipoint + offset;

        if (!(glm::all(glm::greaterThanEqual(p, glm::ivec3(0))) &&
              glm::all(glm::lessThan(p, gridSize))))
            continue;

        size_t index = idot(p, gridIndex);

        // Early exit for isolated points
        if (i >= 27 && std::isinf(localDist))
        {
            earlyExit = true;
            result.distance = cellOffsetLengths[i] * gridStep;
            resultPoint = sceneMin + (glm::dvec3(ipoint) + 0.5) * gridStep;
            break;
        }

        if (!grid[index])
            continue;

        for (const auto &triangle : *grid[index])
        {
            if (triangles.insert(triangle.getId()).second)
            {
                if (!std::isinf(result.distance))
                {
                    if (!triangle.intersectsAABB(lb, ub))
                        continue;
                    if (!triangle.intersectsSphere(point, result.distance))
                        continue;
                }

                auto [closestPoint, weights, normal, code] = triangle.closestPointToTriangle(point);
                glm::dvec3 dir = point - closestPoint;
                double dist = glm::dot(dir, dir);

                if (dist < minDistanceSqrd)
                {
                    minDistanceSqrd = dist;
                    result.distance = std::sqrt(dist);

                    lb = point - glm::dvec3(result.distance);
                    ub = point + glm::dvec3(result.distance);

                    resultTriangle = &triangle;

                    resultPoint = closestPoint;
                    resultNormal = normal;
                    resultCode = code;

                    result.weights = weights;
                    result.triangleId = triangle.getId();
                }
            }
        }

        if (!std::isinf(result.distance))
        {
            localDist = result.distance / gridStep + 
                                                    //std::sqrt(2.0) / 2.0;
                                                    std::sqrt(3.0) / 2.0;
        }

        if (earlyExit)
            break;
    }

    // Determine sign using ray casting
    int sign = 0;

    // if (resultCode == 0) // only when point is over the triangle
    // {
    //     double projection = glm::dot(resultNormal, point - resultPoint);
    //     sign = projection >= 0 ? 1 : -1;
    // } else
    {
        // glm::dvec3 toCenter = glm::normalize(point - ((sceneMax + sceneMin) * 0.5));

        for (const auto &dir : directions)
        {
            int count = countIntersections(point, dir);
            sign += (count % 2 == 0) ? 1 : -1;

            if (/*earlyExit || */std::abs(sign) >= static_cast<int>(directions.size()) / 2 + 1)
            {
                // if (earlyExit) {
                // std::cout << "Early exit for point [" << glm::to_string(point) << "], intersection count " << count << ", sign " << sign << std::endl;
                // }
                break;
            }

            if (count != 0)
                break;
        }
    }

    result.distance *= (sign >= 0) ? 1.0 : -1.0;

    if (earlyExit)
        emptyCells[idot(ipoint, gridIndex)] = result.distance;

    return result;
}