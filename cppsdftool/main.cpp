#include <iostream>
#include <filesystem>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "PreparedTriangle.hpp"
#include "TriangleGrid.hpp"
#include "utils.hpp"

struct ProcessingMetadata {
    glm::ivec3 gridDimensions;
    glm::dvec3 sceneMin;
    glm::dvec3 sceneMax;
    double sceneToPixels;
    double pixelsToScene;
    size_t triangleCount;
};

void printModelInfo(const std::string& filename) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, 
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

    if (!scene) {
        std::cerr << "Error loading file: " << importer.GetErrorString() << std::endl;
        return;
    }

    std::cout << "\nAvailable meshes in the model:" << std::endl;
    std::cout << "Index | Name" << std::endl;
    std::cout << std::string(30, '-') << std::endl;

    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[i];
        std::cout << std::setw(5) << i << " | " 
                 << (mesh->mName.length > 0 ? mesh->mName.C_Str() : "unnamed_mesh") 
                 << std::endl;
    }
    std::cout << std::endl;
}

std::tuple<std::vector<PreparedTriangle>, glm::dvec3, glm::dvec3> 
prepareScene(const std::string& filename, double scale, const std::vector<int>* selectedIndices) {
    std::vector<PreparedTriangle> triangles;
    glm::dvec3 sceneMin(std::numeric_limits<double>::infinity());
    glm::dvec3 sceneMax(-std::numeric_limits<double>::infinity());

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename,
        aiProcess_Triangulate/* | aiProcess_JoinIdenticalVertices*/);

    if (!scene) {
        throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
    }

    int triangleIdx = 0;
    for (unsigned int meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx) {
        // Skip if not in selected indices
        if (selectedIndices && 
            std::find(selectedIndices->begin(), selectedIndices->end(), meshIdx) == selectedIndices->end()) {
            continue;
        }

        const aiMesh* mesh = scene->mMeshes[meshIdx];
        std::vector<glm::dvec3> vertices;
        vertices.reserve(mesh->mNumVertices);

        // Convert vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            const auto& v = mesh->mVertices[i];
            glm::dvec3 vertex(v.x * scale, v.y * scale, v.z * scale);
            vertices.push_back(vertex);

            // Update scene bounds
            sceneMin = glm::min(sceneMin, vertex);
            sceneMax = glm::max(sceneMax, vertex);
        }

        triangles.reserve(mesh->mNumFaces);

        std::unordered_map<uint64_t, glm::dvec3> edgeNormals;

        std::vector<glm::dvec3> vertexNormals;
        vertexNormals.reserve(mesh->mNumVertices);

        // helper method
        auto addEdgeNormal = [&](const int i, const int j, const glm::dvec3& normal) {
            const uint64_t key = ((uint64_t)std::min(i, j) << 32) + (uint64_t)std::max(i, j);
            
            auto [it, inserted] = edgeNormals.try_emplace(key, normal);
            
            if (!inserted) {
                // Key exists, add the new value to existing value
                it->second = glm::normalize(it->second + normal);
            }
        };

        auto getEdgeNormal = [&](const int i, const int j) {
            const uint64_t key = ((uint64_t)std::min(i, j) << 32) + (uint64_t)std::max(i, j);
            return edgeNormals.find(key)->second;
        };

        auto corner = [](
            const glm::dvec3 & x, 
            const glm::dvec3 & y, 
            const glm::dvec3 & z)
        {
            auto v1 = x - y;
            auto v2 = z - y;
            return 2.0 * glm::atan(glm::length(v1/glm::length(v1) - v2/glm::length(v2)), glm::length((v1/glm::length(v1) + v2/glm::length(v2))));
        };

        // Create triangles
        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            const aiFace& face = mesh->mFaces[i];
            if (face.mNumIndices == 3) {

                int a = face.mIndices[0];
                int b = face.mIndices[1];
                int c = face.mIndices[2];

                auto& triangle = triangles.emplace_back(
                    triangleIdx++,
                    vertices,
                    a, b, c
                );

                // Vertex
                const double alpha_0 = corner(triangle.getVertexB(), triangle.getVertexA(), triangle.getVertexC());
                const double alpha_1 = corner(triangle.getVertexA(), triangle.getVertexB(), triangle.getVertexC());
                const double alpha_2 = corner(triangle.getVertexB(), triangle.getVertexC(), triangle.getVertexA());

                vertexNormals[a] += alpha_0 * triangle.getNormal();
                vertexNormals[b] += alpha_1 * triangle.getNormal();
                vertexNormals[c] += alpha_2 * triangle.getNormal();

                // Edge
                addEdgeNormal(a, b, triangle.getNormal());
                addEdgeNormal(b, c, triangle.getNormal());
                addEdgeNormal(a, c, triangle.getNormal());
            }
        }

        for (size_t i = 0; i < vertexNormals.size(); i++) {
		    vertexNormals[i] = glm::normalize(vertexNormals[i]);
	    }

        for (auto & triangle : triangles) {
            int a = triangle.getIndexA();
            int b = triangle.getIndexB();
            int c = triangle.getIndexC();

            triangle.setPseudoNormals(
                vertexNormals[a], vertexNormals[b], vertexNormals[c],
                getEdgeNormal(a, b), getEdgeNormal(b, c), getEdgeNormal(a, c));
        }
    }

    return {triangles, sceneMin, sceneMax};
}

ProcessingMetadata processModel(const std::string& filename, const std::string& outputFile,
                              int pixels = 256, int topLodCellSize = 8, double scale = 1.0,
                              const std::vector<int>* selectedIndices = nullptr) {
    std::cout   << timestamp()
                << "Processing file "
                << filename 
                << std::endl;

    // Load and prepare scene
    auto [triangles, sceneMin, sceneMax] = prepareScene(filename, scale, selectedIndices);
    
    // Calculate scene dimensions
    double maxSide = std::max({
        sceneMax.z - sceneMin.z,
        sceneMax.y - sceneMin.y,
        sceneMax.x - sceneMin.x
    });

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

    std::cout   << timestamp() 
                << "File preprocessed. "
                << "X: " << sx << ", Y: " << sy << ", Z: " << sz 
                << ", maximum distance: " << maxSide 
                << std::endl;

    // Create triangle grid
    TriangleGrid triangleGrid(lowerBound, upperBound,
        sx / topLodCellSize + (sx % topLodCellSize != 0),
        sy / topLodCellSize+ (sy % topLodCellSize != 0),
        sz / topLodCellSize + (sz % topLodCellSize != 0),
        triangles);

    std::cout   << timestamp()
                << "Triangle grid ready: " 
                << triangleGrid.getTriangleCount() 
                << std::endl;

    // Generate distance field
    std::vector<double> distanceData = triangleGrid.dispatch(
        lowerBound,
        pixelsToScene,
        sceneToPixels / paddedTopLodCellSize,
        sx, sy, sz
    );

    std::cout   << timestamp()
                << "Distance field processed. Length: "
                << distanceData.size() 
                << std::endl;

    // Save to file
    std::vector<float> floatData(distanceData.begin(), distanceData.end());

    saveKTX(KTX_R32F, (uint)sx, (uint)sy, (uint)sz, floatData, outputFile, 4);

    std::cout   << timestamp()
                << "Distance field saved to " 
                << outputFile 
                << std::endl;

    return ProcessingMetadata{
        glm::ivec3(sx, sy, sz),
        sceneMin,
        sceneMax,
        sceneToPixels,
        pixelsToScene,
        triangles.size()
    };
}

void printUsage() {
    std::cout << "Usage: process_model [options] input_file output_file\n"
              << "Options:\n"
              << "  --list                List available meshes and exit\n"
              << "  --scale value         Scale factor to apply to the model\n"
              << "  --pixels value        Maximum pixel size for LOD 0\n"
              << "  --top-lod-cell-size value  Cell size for highest LOD\n"
              << "  --indices i1 i2 ...   Indices of meshes to process\n";
}

int main(int argc, char* argv[]) {
    try {
        std::vector<std::string> args(argv + 1, argv + argc);
        if (args.empty()) {
            printUsage();
            return 1;
        }

        std::string inputFile;
        std::string outputFile;
        double scale = 1.0f;
        int pixels = 256;
        int topLodCellSize = 8;
        std::vector<int> selectedIndices;
        bool listOnly = false;

        for (size_t i = 0; i < args.size(); ++i) {
            if (args[i] == "--list") {
                listOnly = true;
            } else if (args[i] == "--scale" && i + 1 < args.size()) {
                scale = std::stof(args[++i]);
            } else if (args[i] == "--pixels" && i + 1 < args.size()) {
                pixels = std::stoi(args[++i]);
            } else if (args[i] == "--top-lod-cell-size" && i + 1 < args.size()) {
                topLodCellSize = std::stoi(args[++i]);
            } else if (args[i] == "--indices") {
                while (i + 1 < args.size() && args[i + 1][0] != '-') {
                    selectedIndices.push_back(std::stoi(args[++i]));
                }
            } else if (inputFile.empty()) {
                inputFile = args[i];
            } else if (outputFile.empty()) {
                outputFile = args[i];
            }
        }

        if (!std::filesystem::exists(inputFile)) {
            std::cerr << "Error: Input file '" << inputFile << "' does not exist" << std::endl;
            return 1;
        }

        if (listOnly) {
            printModelInfo(inputFile);
            return 0;
        }

        if (inputFile.empty() || outputFile.empty()) {
            std::cerr << "Error: Both input and output files must be specified" << std::endl;
            printUsage();
            return 1;
        }

        // Create output directory if it doesn't exist
        std::filesystem::path outputPath(outputFile);
        if (auto parentPath = outputPath.parent_path(); !parentPath.empty()) {
            std::filesystem::create_directories(parentPath);
        }

        // Process the model
        auto metadata = processModel(
            inputFile,
            outputFile,
            pixels,
            topLodCellSize,
            scale,
            selectedIndices.empty() ? nullptr : &selectedIndices
        );

        // Print summary
        std::cout << "\nProcessing Summary:" << std::endl;
        std::cout << "Grid dimensions: " << metadata.gridDimensions.x << "x" 
                 << metadata.gridDimensions.y << "x" << metadata.gridDimensions.z << std::endl;
        std::cout << "Triangle count: " << metadata.triangleCount << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}