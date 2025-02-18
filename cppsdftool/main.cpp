#include <iostream>
#include <filesystem>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "PreparedTriangle.hpp"
#include "TriangleGrid.hpp"
#include "utils.hpp"

struct ProcessingMetadata
{
    glm::ivec3 gridDimensions;
    glm::dvec3 sceneMin;
    glm::dvec3 sceneMax;
    double sceneToPixels;
    double pixelsToScene;
    size_t triangleCount;
};

struct DVec3Hash
{
    size_t operator()(const glm::dvec3 &v) const
    {
        size_t hash = 0;
        hash_combine(hash, v.x);
        hash_combine(hash, v.y);
        hash_combine(hash, v.z);
        return hash;
    }

    static void hash_combine(size_t &seed, double v)
    {
        seed ^= std::hash<double>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
};

struct DVec3Equal
{
    bool operator()(const glm::dvec3 &a, const glm::dvec3 &b) const
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

struct PairDVec3Hash
{
    size_t operator()(const std::pair<glm::dvec3, glm::dvec3> &v) const
    {
        size_t hash = 0;
        hash_combine(hash, v.first.x);
        hash_combine(hash, v.first.y);
        hash_combine(hash, v.first.z);
        hash_combine(hash, v.second.x);
        hash_combine(hash, v.second.y);
        hash_combine(hash, v.second.z);
        return hash;
    }

    static void hash_combine(size_t &seed, double v)
    {
        seed ^= std::hash<double>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
};

struct PairDVec3Equal
{
    bool operator()(const std::pair<glm::dvec3, glm::dvec3> &a, const std::pair<glm::dvec3, glm::dvec3> &b) const
    {
        return a.first.x == b.first.x && a.first.y == b.first.y && a.first.z == b.first.z &&
               a.second.x == b.second.x && a.second.y == b.second.y && a.second.z == b.second.z;
    }
};

std::tuple<std::vector<PreparedTriangle>, glm::dvec3, glm::dvec3>
prepareScene(const std::string &filename, double scale, const std::vector<int> *selectedIndices)
{
    std::vector<PreparedTriangle> triangles;
    glm::dvec3 sceneMin(std::numeric_limits<double>::infinity());
    glm::dvec3 sceneMax(-std::numeric_limits<double>::infinity());

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(filename,
                                             aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

    if (!scene)
    {
        throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
    }

    int triangleIdx = 0;
    for (unsigned int meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
    {
        // Skip if not in selected indices
        if (selectedIndices &&
            std::find(selectedIndices->begin(), selectedIndices->end(), meshIdx) == selectedIndices->end())
        {
            continue;
        }

        const aiMesh *mesh = scene->mMeshes[meshIdx];
        std::vector<glm::dvec3> vertices;
        vertices.reserve(mesh->mNumVertices);

        std::unordered_map<glm::dvec3, glm::dvec3, DVec3Hash, DVec3Equal> vertexNormals;
        vertexNormals.reserve(mesh->mNumVertices);

        if (mesh->HasNormals())
        {
            std::cout << timestamp()
                      << "Processing vertex normals" << std::endl;
        }

        // Convert vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
        {
            const auto &v = mesh->mVertices[i];
            glm::dvec3 vertex(v.x * scale, v.y * scale, v.z * scale);
            vertices.push_back(vertex);

            // Update scene bounds
            sceneMin = glm::min(sceneMin, vertex);
            sceneMax = glm::max(sceneMax, vertex);
        }

        triangles.reserve(mesh->mNumFaces);

        std::unordered_map<std::pair<glm::dvec3, glm::dvec3>, glm::dvec3, PairDVec3Hash, PairDVec3Equal> edgeNormals;
        edgeNormals.reserve(mesh->mNumFaces * 3);

        // // helper method
        // auto addEdgeNormal = [&](const int i, const int j, const glm::dvec3 &normal)
        // {
        //     const uint64_t key = (((uint64_t)std::min(i, j)) << 32l) + (uint64_t)std::max(i, j);

        //     auto it = edgeNormals.find(key);

        //     glm::dvec4 normal4(normal.x, normal.y, normal.z, 1);

        //     if (it != edgeNormals.end())
        //         normal4 += it->second;
        //     edgeNormals[key] = normal4;
        // };

        // auto getEdgeNormal = [&](const int i, const int j)
        // {
        //     const uint64_t key = (((uint64_t)std::min(i, j)) << 32l) + (uint64_t)std::max(i, j);
        //     auto normal = edgeNormals.find(key)->second;
        //     return glm::dvec3(normal.x, normal.y, normal.z) / normal.w;
        // };

        auto addNormal = [](std::unordered_map<glm::dvec3, glm::dvec3, DVec3Hash, DVec3Equal> &collection, const glm::dvec3 &key, const glm::dvec3 &normal)
        {
            auto [it, inserted] = collection.try_emplace(key, normal);

            if (!inserted)
                it->second += normal;
        };

        auto addEdgeNormal = [](std::unordered_map<std::pair<glm::dvec3, glm::dvec3>, glm::dvec3, PairDVec3Hash, PairDVec3Equal> &collection, const glm::dvec3 &a, const glm::dvec3 &b, const glm::dvec3 &normal)
        {
            glm::dvec3 center = (a + b) * 0.5;
            glm::dvec3 ca = a - center;
            glm::dvec3 cb = b - center;

            std::pair<glm::dvec3, glm::dvec3> key = ca.x + ca.y + ca.z < cb.x + cb.y + cb.z ? std::pair<glm::dvec3, glm::dvec3>(a, b) : std::pair<glm::dvec3, glm::dvec3>(b, a);

            auto [it, inserted] = collection.try_emplace(key, normal);

            if (!inserted)
                it->second += normal;
        };

        auto getEdgeNormal = [](std::unordered_map<std::pair<glm::dvec3, glm::dvec3>, glm::dvec3, PairDVec3Hash, PairDVec3Equal> &collection, const glm::dvec3 &a, const glm::dvec3 &b)
        {
            glm::dvec3 center = (a + b) * 0.5;
            glm::dvec3 ca = a - center;
            glm::dvec3 cb = b - center;

            std::pair<glm::dvec3, glm::dvec3> key = ca.x + ca.y + ca.z < cb.x + cb.y + cb.z ? std::pair<glm::dvec3, glm::dvec3>(a, b) : std::pair<glm::dvec3, glm::dvec3>(b, a);

            return collection.find(key)->second;
        };

        auto corner = [](
                          const glm::dvec3 &x,
                          const glm::dvec3 &y,
                          const glm::dvec3 &z)
        {
            auto v1 = x - y;
            auto v2 = z - y;
            return 2.0 * glm::atan(glm::length(v1 / glm::length(v1) - v2 / glm::length(v2)), glm::length((v1 / glm::length(v1) + v2 / glm::length(v2))));
        };

        // Create triangles
        for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
        {
            const aiFace &face = mesh->mFaces[i];
            if (face.mNumIndices == 3)
            {
                int a = face.mIndices[0];
                int b = face.mIndices[1];
                int c = face.mIndices[2];

                auto &triangle = triangles.emplace_back(
                    triangleIdx++,
                    vertices,
                    a, b, c);

                if (!mesh->HasNormals())
                {
                    // Vertex
                    const double alpha_0 = corner(triangle.getVertexB(), triangle.getVertexA(), triangle.getVertexC());
                    const double alpha_1 = corner(triangle.getVertexA(), triangle.getVertexB(), triangle.getVertexC());
                    const double alpha_2 = corner(triangle.getVertexB(), triangle.getVertexC(), triangle.getVertexA());

                    addNormal(vertexNormals, triangle.getVertexA(), alpha_0 * triangle.getNormal());
                    addNormal(vertexNormals, triangle.getVertexB(), alpha_1 * triangle.getNormal());
                    addNormal(vertexNormals, triangle.getVertexC(), alpha_2 * triangle.getNormal());
                }

                // Edge
                addEdgeNormal(edgeNormals, triangle.getVertexA(), triangle.getVertexB(), triangle.getNormal());
                addEdgeNormal(edgeNormals, triangle.getVertexB(), triangle.getVertexC(), triangle.getNormal());
                addEdgeNormal(edgeNormals, triangle.getVertexA(), triangle.getVertexC(), triangle.getNormal());
            }
        }

        for (auto &normal : vertexNormals)
            normal.second = glm::normalize(normal.second);

        for (auto &normal : edgeNormals)
            normal.second = glm::normalize(normal.second);

        for (auto &triangle : triangles)
        {
            const auto &a = triangle.getVertexA();
            const auto &b = triangle.getVertexB();
            const auto &c = triangle.getVertexC();

            if (mesh->HasNormals())
            {
                int ia = triangle.getIndexA();
                int ib = triangle.getIndexB();
                int ic = triangle.getIndexC();

                glm::dvec3 va(mesh->mNormals[ia].x, mesh->mNormals[ia].y, mesh->mNormals[ia].z);
                glm::dvec3 vb(mesh->mNormals[ib].x, mesh->mNormals[ib].y, mesh->mNormals[ib].z);
                glm::dvec3 vc(mesh->mNormals[ic].x, mesh->mNormals[ic].y, mesh->mNormals[ic].z);
                triangle.setPseudoNormals(
                    va, vb, vc,
                    getEdgeNormal(edgeNormals, a, b), getEdgeNormal(edgeNormals, b, c), getEdgeNormal(edgeNormals, a, c));
            }
            else
            {
                triangle.setPseudoNormals(
                    vertexNormals[a], vertexNormals[b], vertexNormals[c],
                    getEdgeNormal(edgeNormals, a, b), getEdgeNormal(edgeNormals, b, c), getEdgeNormal(edgeNormals, a, c));
            }
        }

        // int openEdges = 0;

        // for (auto &edge : edgeNormals)
        // {
        //     if (edge.second.w == 1)
        //     {
        //         // std::cout << "Edge " << edge.first
        //         //           << " has " << edge.second.w
        //         //           << " triangles affecting its normals"
        //         //           << std::endl;
        //         openEdges++;
        //     }
        // }

        // if (openEdges > 0)
        // {
        //     std::cout << "Total " << openEdges
        //               << " edges of " << edgeNormals.size()
        //               << " has less than 2 triangles"
        //               << std::endl;
        // }
    }

    return {triangles, sceneMin, sceneMax};
}

ProcessingMetadata processModel(const std::string &filename, const std::string &outputFile,
                                int pixels = 256, int quality = 0, int lod = 1, int topLodCellSize = 8, double scale = 1.0,
                                const std::vector<int> *selectedIndices = nullptr, bool saveToPoints = false)
{
    std::cout << timestamp()
              << "Processing file "
              << filename
              << std::endl;

    // Load and prepare scene
    auto [triangles, sceneMin, sceneMax] = prepareScene(filename, scale, selectedIndices);

    // Calculate scene dimensions
    double maxSide = std::max({sceneMax.z - sceneMin.z,
                               sceneMax.y - sceneMin.y,
                               sceneMax.x - sceneMin.x});

    // Calculate conversion factors and dimensions
    int paddedTopLodCellSize = topLodCellSize;
    topLodCellSize -= 1;

    double sceneToPixels = pixels / maxSide;
    double pixelsToScene = 1.0 / sceneToPixels;

    // Calculate grid dimensions
    int sx = static_cast<int>(std::ceil((sceneMax.x - sceneMin.x) * sceneToPixels));
    int sy = static_cast<int>(std::ceil((sceneMax.y - sceneMin.y) * sceneToPixels));
    int sz = static_cast<int>(std::ceil((sceneMax.z - sceneMin.z) * sceneToPixels));

    // Calculate padding
    int padx = sx % topLodCellSize != 0 ? topLodCellSize - (sx % topLodCellSize) : topLodCellSize;
    int pady = sy % topLodCellSize != 0 ? topLodCellSize - (sy % topLodCellSize) : topLodCellSize;
    int padz = sz % topLodCellSize != 0 ? topLodCellSize - (sz % topLodCellSize) : topLodCellSize;

    sx += padx + 1;
    sy += pady + 1;
    sz += padz + 1;

    // Calculate bounds with padding
    glm::dvec3 padding(padx, pady, padz);
    padding *= pixelsToScene * 0.5;
    glm::dvec3 lowerBound = sceneMin - padding;
    glm::dvec3 upperBound = sceneMax + padding;

    std::cout << timestamp()
              << "File preprocessed. "
              << "X: " << sx << ", Y: " << sy << ", Z: " << sz
              << ", maximum distance: " << maxSide
              << std::endl;

    // Create triangle grid
    TriangleGrid triangleGrid(lowerBound, upperBound,
                              (sx / topLodCellSize + (sx % topLodCellSize != 0)) << (lod - 1),
                              (sy / topLodCellSize + (sy % topLodCellSize != 0)) << (lod - 1),
                              (sz / topLodCellSize + (sz % topLodCellSize != 0)) << (lod - 1),
                              triangles);

    std::cout << timestamp()
              << "Triangle grid ready: "
              << glm::to_string(triangleGrid.getGridSize())
              << std::endl;

    // Generate distance field
    std::vector<std::vector<double>> distanceData = triangleGrid.dispatch(
        lowerBound,
        pixelsToScene,
        topLodCellSize,
        sx, sy, sz, quality, lod);

    std::cout << timestamp()
              << "Distance field processed. Length: "
              << distanceData.size()
              << std::endl;

    // Save to file

    if (saveToPoints)
    {
        savePoints((uint)sx, (uint)sy, (uint)sz, paddedTopLodCellSize, sceneMin.x, sceneMin.y, sceneMin.z, sceneMax.x, sceneMax.y, sceneMax.z, distanceData, outputFile, false, false);
    }
    else
    {
        if (distanceData.size() == 1)
        {
            std::vector<float> floatData(distanceData[0].begin(), distanceData[0].end());
            saveKTX(KTX_R32F, (uint)sx, (uint)sy, (uint)sz, floatData, outputFile, 4);
        }
        else
        {
            int curSX = sx;
            int curSY = sy;
            int curSZ = sz;

            for (size_t i = 0; i < distanceData.size(); i++)
            {
                std::filesystem::path p(outputFile);
                std::string ext = p.extension().string();
                p.replace_extension();  // Remove last extension
                ext = p.extension().string() + ext;  // Combine extensions
                p.replace_extension("");  // Remove all extensions
                std::string newPath = p.string() + ".lod" + std::to_string(i + 1) + ext;
                // std::filesystem::path p(outputFile);
                // std::string newPath = p.parent_path().string() + "/" + p.stem().string() +
                //                       ".lod" + std::to_string(i + 1) + p.extension().string();
                                      
                std::vector<float> floatData(distanceData[i].begin(), distanceData[i].end());
                saveKTX(KTX_R32F, (uint)curSX, (uint)curSY, (uint)curSZ, floatData, newPath, 4);

                curSX = curSX * 2 - 1;
                curSY = curSY * 2 - 1;
                curSZ = curSZ * 2 - 1;
            }
        }
    }

    std::cout << timestamp()
              << "Distance field saved to "
              << outputFile
              << std::endl;

    return ProcessingMetadata{
        glm::ivec3(sx, sy, sz),
        sceneMin,
        sceneMax,
        sceneToPixels,
        pixelsToScene,
        triangles.size()};
}

void printUsage()
{
    std::cout << "Usage: process_model [options] input_file output_file\n"
              << "Options:\n"
              << "  --points              Save points file instead of KTX 3D texture\n"
              << "  --scale value         Scale factor to apply to the model\n"
              << "  --pixels value        Maximum pixel size for LOD 0\n"
              << "  --top-lod-cell-size value  Cell size for highest LOD\n"
              << "  --indices i1 i2 ...   Indices of meshes to process\n"
              << "  --lod n               Number of LOD levels to process. By default 1\n"
              << "  --quality value       SDF quality: 0-3\n";
}

int main(int argc, char *argv[])
{
    try
    {
        std::vector<std::string> args(argv + 1, argv + argc);
        if (args.empty())
        {
            printUsage();
            return 1;
        }

        std::string inputFile;
        std::string outputFile;
        double scale = 1.0f;
        int pixels = 256;
        int quality = 0;
        int lod = 1;
        int topLodCellSize = 8;
        std::vector<int> selectedIndices;
        bool points = false;

        for (size_t i = 0; i < args.size(); ++i)
        {
            if (args[i] == "--points")
            {
                points = true;
            }
            else if (args[i] == "--scale" && i + 1 < args.size())
            {
                scale = std::stof(args[++i]);
            }
            else if (args[i] == "--pixels" && i + 1 < args.size())
            {
                pixels = std::stoi(args[++i]);
            }
            else if (args[i] == "--quality" && i + 1 < args.size())
            {
                quality = std::stoi(args[++i]);
            }
            else if (args[i] == "--lod" && i + 1 < args.size())
            {
                lod = std::stoi(args[++i]);
            }
            else if (args[i] == "--top-lod-cell-size" && i + 1 < args.size())
            {
                topLodCellSize = std::stoi(args[++i]);
            }
            else if (args[i] == "--indices")
            {
                while (i + 1 < args.size() && args[i + 1][0] != '-')
                {
                    selectedIndices.push_back(std::stoi(args[++i]));
                }
            }
            else if (inputFile.empty())
            {
                inputFile = args[i];
            }
            else if (outputFile.empty())
            {
                outputFile = args[i];
            }
        }

        if (!std::filesystem::exists(inputFile))
        {
            std::cerr << "Error: Input file '" << inputFile << "' does not exist" << std::endl;
            return 1;
        }

        if (inputFile.empty() || outputFile.empty())
        {
            std::cerr << "Error: Both input and output files must be specified" << std::endl;
            printUsage();
            return 1;
        }

        if (lod < 1)
        {
            std::cerr << "Error: LOD must be at least 1" << std::endl;
            return 1;
        }

        if (lod > 10)
        {
            std::cerr << "Error: LOD must be at most 10" << std::endl;
            return 1;
        }

        // Create output directory if it doesn't exist
        std::filesystem::path outputPath(outputFile);
        if (auto parentPath = outputPath.parent_path(); !parentPath.empty())
        {
            std::filesystem::create_directories(parentPath);
        }

        // Process the model
        auto metadata = processModel(
            inputFile,
            outputFile,
            pixels,
            quality,
            lod,
            topLodCellSize,
            scale,
            selectedIndices.empty() ? nullptr : &selectedIndices,
            points);

        // Print summary
        std::cout << "\nProcessing Summary:" << std::endl;
        std::cout << "Grid dimensions: " << metadata.gridDimensions.x << "x"
                  << metadata.gridDimensions.y << "x" << metadata.gridDimensions.z << std::endl;
        std::cout << "Triangle count: " << metadata.triangleCount << std::endl;

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}