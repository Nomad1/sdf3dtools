#include "TriangleGrid.hpp"
#include "FastWindingNumber.hpp"
#include "utils.hpp"

#include <iostream>

//#define CALC_ALL_LODS

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
    : sceneMin(sceneMin), sceneMax(sceneMax), gridSize(gridX, gridY, gridZ), gridIndex(1, gridX, gridX * gridY),
      fastWindingNumber(new FastWindingNumber(triangles))
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

    // Sort triangles by area in ascending order
    for (auto &cell : grid)
    {
        if (cell)
        {
            std::sort(cell->begin(), cell->end(),
                      [](const PreparedTriangle &a, const PreparedTriangle &b)
                      {
                          return a.getArea() < b.getArea();
                      });
        }
    }
}

TriangleGrid::~TriangleGrid()
{
    delete fastWindingNumber;
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

const glm::ivec3 &TriangleGrid::getGridSize() const
{
    return gridSize;
}

std::string formatBytes(size_t bytes) {
    std::string result = std::to_string(bytes);
    for (int i = result.length() - 3; i > 0; i -= 3) {
        result.insert(i, " ");
    }
    return result;
}

std::vector<std::vector<float>> TriangleGrid::dispatch(const glm::dvec3 &lowerBound,
                                                        const double pixelsToScene, const int lowerLodPixels, const int cellSize,
                                                        const int sx, const int sy, const int sz, const int quality,
                                                        const uint lods)
{
    std::vector<std::vector<float>> results;
    results.resize(lods);

    int curSX = sx;
    int curSY = sy;
    int curSZ = sz;

    // allocate all the vectors to creash early
    for (size_t l = 0; l < lods; ++l)
    {
        int scale = 1 << l;
        curSX = sx * scale - (scale - 1);
        curSY = sy * scale - (scale - 1);
        curSZ = sz * scale - (scale - 1);

        std::vector<float> &result = results[l];
        size_t size = (size_t)curSX * (size_t)curSY * (size_t)curSZ * 3;

        std::cout << timestamp()
                  << "Allocating " << formatBytes(size * 4) << " bytes for LOD "<< l + 1
                  << std::endl;

        result.resize(size);
    }

#ifdef CALC_ALL_LODS
    curSX = sx;
    curSY = sy;
    curSZ = sz;

    for (size_t l = 0; l < lods; ++l)
    {
        double currentPixelsToScene = pixelsToScene / (lowerLodPixels * (1 << l));
        double currentSearchWindow = pixelsToScene / ((std::max({curSX,curSY,curSZ}) / cellSize));

        size_t maxIndex = getMaxSearchIndex(currentSearchWindow);

        std::cout << timestamp()
                  << "Processing LOD " << (l + 1)
                  << ", size: [" << curSX << ", " << curSY << ", " << curSZ << "]"
                  << ", search window is " << currentSearchWindow << ", maxIndex " << maxIndex
                  << std::endl;

        std::vector<float> &result = results[l];

        #pragma omp parallel for schedule(dynamic) collapse(3)
        for (int iz = 0; iz < curSZ; ++iz)
        {
            for (int iy = 0; iy < curSY; ++iy)
            {
                for (int ix = 0; ix < curSX; ++ix)
                {
                    int index = (iz * curSY * curSX + iy * curSX + ix) * 3;

                    glm::dvec3 point = lowerBound + glm::dvec3(ix, iy, iz) * currentPixelsToScene;
                    auto [distance, weights, triangleId] = findTriangles(point, quality, maxIndex);
                    float pixelDistance = distance / currentPixelsToScene;

                    // Store results in a thread-safe manner
                    result[index] = pixelDistance;
                    result[index + 1] = static_cast<float>(weights.x);
                    result[index + 2] = static_cast<float>(weights.y);
                    //result[index + 3] = static_cast<float>(triangleId);
                }
            }
        }

        curSX = curSX * 2 - 1;
        curSY = curSY * 2 - 1;
        curSZ = curSZ * 2 - 1;
    }
#else

    // Process only the last LOD first
    int scale = 1 << (lods - 1);
    curSX = sx * scale - (scale - 1);
    curSY = sy * scale - (scale - 1);
    curSZ = sz * scale - (scale - 1);
    double currentPixelsToScene = pixelsToScene / (lowerLodPixels * scale);
    double currentSearchWindow = pixelsToScene / ((std::max({curSX,curSY,curSZ}) / cellSize));

    // Process highest resolution first (last LOD)
    std::vector<float> &lastLOD = results[lods - 1];

    size_t maxIndex = getMaxSearchIndex(currentSearchWindow);

    std::cout << timestamp()
              << "Processing LOD " << (lods)
              << ", size: [" << curSX << ", " << curSY << ", " << curSZ << "]"
              << ", search window is " << currentSearchWindow << ", maxIndex " << maxIndex
              << std::endl;

    #pragma omp parallel for schedule(dynamic) collapse(3)
    for (int iz = 0; iz < curSZ; ++iz)
    {
        for (int iy = 0; iy < curSY; ++iy)
        {
            for (int ix = 0; ix < curSX; ++ix)
            {
                int index = (iz * curSX * curSY + iy * curSX + ix) * 3;
                glm::dvec3 point = lowerBound + glm::dvec3(ix, iy, iz) * currentPixelsToScene;
                auto [distance, weights, triangleId] = findTriangles(point, quality, maxIndex);
                float pixelDistance = distance / currentPixelsToScene;// / (pixelsToScene * (double)cellSize);

                lastLOD[index + 0] = pixelDistance;
                lastLOD[index + 1] = weights.x;
                lastLOD[index + 2] = weights.y;
                //lastLOD[index + 3] = static_cast<float>(triangleId);
            }
        }
    }

    int prevSX = curSX;
    int prevSY = curSY;
    int prevSZ = curSZ;
    // Calculate lower LODs by downsampling
    for (int l = lods - 2; l >= 0; --l)
    {
        int scale = 1 << l;
        curSX = sx * scale - (scale - 1);
        curSY = sy * scale - (scale - 1);
        curSZ = sz * scale - (scale - 1);

        float nextPixelsToScene = pixelsToScene / (lowerLodPixels * scale);
        float recalcDistance = currentPixelsToScene / nextPixelsToScene;

        std::cout << timestamp()
                  << "Processing LOD " << (l + 1)
                  << ", size: [" << curSX << ", " << curSY << ", " << curSZ << "]"
                  << std::endl;

        #pragma omp parallel for collapse(3)
        for (int iz = 0; iz < curSZ; ++iz)
        {
            for (int iy = 0; iy < curSY; ++iy)
            {
                for (int ix = 0; ix < curSX; ++ix)
                {
                    int curIndex = (iz * curSY * curSX + iy * curSX + ix) * 3;

                     // Variables to store the sums of the 8 pixels
                    float sumValue0 = 0.0f;
                    float sumValue1 = 0.0f;
                    float sumValue2 = 0.0f;
                    int validPixelCount = 0;

                    // Base coordinate in the previous level
                    int fromZ = iz * 2;
                    int fromY = iy * 2;
                    int fromX = ix * 2;

                    int toZ = std::min(fromZ + 2, prevSZ);
                    int toY = std::min(fromY + 2, prevSY);
                    int toX = std::min(fromX + 2, prevSX);

                    // Accumulate values from all 8 neighboring pixels
                    for(int pz = fromZ; pz < toZ; ++pz)
                    for(int py = fromY; py < toY; ++py)
                    for(int px = fromX; px < toX; ++px)
                    {
                        int index = (pz * prevSY * prevSX + py * prevSX + px) * 3;

                        sumValue0 += results[l + 1][index + 0];
                        sumValue1 += results[l + 1][index + 1];
                        sumValue2 += results[l + 1][index + 2];
                        validPixelCount++;
                    }

                    float pixelCoef = validPixelCount == 0 ? 0.0f : 1.0f / validPixelCount;

                    results[l][curIndex + 0] = sumValue0 * pixelCoef * recalcDistance;
                    results[l][curIndex + 1] = sumValue1 * pixelCoef;
                    results[l][curIndex + 2] = sumValue2 * pixelCoef;
                }
            }
        }

        currentPixelsToScene = nextPixelsToScene;
        prevSX = curSX;
        prevSY = curSY;
        prevSZ = curSZ;
    }

#endif

    return results;
}

int TriangleGrid::countIntersections(const glm::dvec3 &point, const glm::dvec3 &dir)
{
    int count = 0;
    glm::dvec3 idir = 1.0 / dir;

    // Convert to grid coordinates
    glm::dvec3 localPoint = (point - sceneMin) / gridStep;

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

size_t TriangleGrid::getMaxSearchIndex(const double searchWindow)
{
    size_t maxIndex = cellOffsetLengths.size() - 1;

    double maxSearchWindow = searchWindow / gridStep + std::sqrt(3.0) / 2.0;

    // std::cout << timestamp()
    //           << "Max search window in grid units: "
    //           << maxSearchWindow
    //           << std::endl;

    for (size_t i = 0; i < maxIndex; ++i)
        if (cellOffsetLengths[i] > maxSearchWindow)
        {
            maxIndex = i;
            break;
        }

    return maxIndex;
}

TriangleGrid::FindTrianglesResult TriangleGrid::findTriangles(const glm::dvec3 &point, const int quality, const size_t maxIndex)
{
    FindTrianglesResult result;
    result.distance = std::numeric_limits<double>::infinity();
    result.triangleId = -1;
    result.weights = glm::dvec3(0.0);

    glm::dvec3 resultPoint;
    glm::dvec3 resultNormal;

    double minDistanceSqrd = std::numeric_limits<double>::infinity();
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

        if (p.x < 0 || p.y < 0 || p.z < 0 || p.x >= gridSize.x || p.y >= gridSize.y || p.z >= gridSize.z)
            continue;

        size_t index = idot(p, gridIndex);

        // TODO: when the grid cell is small enough, the next triangle could be in many cells beyond the current
        // we need to determine how many cells we can read from cellOffsetLengths before stopping early
        // it's 27 for 3x3x3 cube, meaning that element 28 is guaranteed to be at least 1 cell length away 
        #ifndef EARLY_EXIT
        
        // Early exit for isolated points
        if (i >= maxIndex && std::isinf(localDist))
        {
            earlyExit = true;
            result.distance = cellOffsetLengths[i] * gridStep;
            resultPoint = sceneMin + (glm::dvec3(ipoint) + 0.5) * gridStep;
            break;
        }
        #endif

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

                double diff = dist - minDistanceSqrd;

                if (diff <= 0)
                {
                    if (std::abs(diff) < std::numeric_limits<double>::epsilon())
                    {
                        if (resultCode == 0 && code != 0)
                            continue;
                    }

                    minDistanceSqrd = dist;
                    result.distance = std::sqrt(dist);

                    lb = point - glm::dvec3(result.distance);
                    ub = point + glm::dvec3(result.distance);

                    resultPoint = closestPoint;
                    resultNormal = normal;
                    resultCode = code;

                    result.weights = weights;
                    result.triangleId = triangle.getId();

                    localDist = result.distance / gridStep+ std::sqrt(3.0) / 2.0;
                }
            }
        }

        // if (!std::isinf(result.distance))
        // {
        //     localDist = result.distance / gridStep + std::sqrt(3.0) / 2.0;
        // }
    }

    // Determine sign using ray casting
    int sign = 0;

    if (quality == 4)
    {
        sign = fastWindingNumber->is_inside(point) ? -1 : 1;
    }
    else if (resultCode != -1 && (quality == 0 || (quality == 1 && resultCode == 0)))
    {
        double projection = glm::dot(resultNormal, point - resultPoint);
        sign = projection >= 0 ? 1 : -1;
    }
    else
    {
        for (const auto &dir : directions)
        {
            int count = countIntersections(point, dir);
            sign += (count % 2 == 0) ? 1 : -1;

            if (/*earlyExit || */ std::abs(sign) >= static_cast<int>(directions.size()) / 2 + 1)
                break;

            if (count != 0 && quality == 2)
                break;
        }

        // if (resultCode == 3)
        // {
        //     double projection = glm::dot(resultNormal, point - resultPoint);
        //     if ((projection < 0 && sign >= 0) || (projection >= 0 && sign < 0))
        //     {
        //         std::cout << "Faulty point type " << resultCode << ": [" << glm::to_string(point) << "], triangle " << resultTriangle->getId()
        //                   << ", result point [" << glm::to_string(resultPoint) << "], sign " << sign
        //                   << ", triangle normal [" << glm::to_string(resultTriangle->getNormal()) << "]"
        //                   << ", pseudo normal [" << glm::to_string(resultNormal) << "]"
        //                   << std::endl;
        //     }
        // }
    }

    result.distance *= (sign >= 0) ? 1.0 : -1.0;

    if (earlyExit)
        emptyCells[idot(ipoint, gridIndex)] = result.distance;

    return result;
}