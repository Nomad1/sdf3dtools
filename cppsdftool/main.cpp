#include <iostream>
#include <fstream>  // Add this for ofstream
#include <chrono>
#include <filesystem>
#include <cstring>
#include <vector>
#include <memory>
#include <sstream>
#include <iomanip>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "PreparedTriangle.hpp"
#include "TriangleGrid.hpp"

struct ProcessingMetadata {
    glm::ivec3 gridDimensions;
    glm::vec3 sceneMin;
    glm::vec3 sceneMax;
    float sceneToPixels;
    float pixelsToScene;
    size_t triangleCount;
};

std::string timestamp(std::chrono::steady_clock::time_point & startTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);

    auto hours = std::chrono::duration_cast<std::chrono::hours>(ms);
    ms -= hours;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(ms);
    ms -= minutes;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(ms);
    ms -= seconds;

    std::stringstream ss;
    ss << "["
       << std::setfill('0') 
       << std::setw(2) << hours.count() << ":"
       << std::setw(2) << minutes.count() << ":"
       << std::setw(2) << seconds.count() << "."
       << std::setw(3) << ms.count() << "] ";
    return ss.str();
}

// Forward declarations
std::tuple<std::vector<PreparedTriangle>, glm::vec3, glm::vec3> 
prepareScene(const std::string& filename, float scale, const std::vector<int>* selectedIndices);
void printModelInfo(const std::string& filename);
ProcessingMetadata processModel(const std::string& filename, const std::string& outputFile,
                              int pixels, int topLodCellSize, float scale,
                              const std::vector<int>* selectedIndices);

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

std::tuple<std::vector<PreparedTriangle>, glm::vec3, glm::vec3> 
prepareScene(const std::string& filename, float scale, const std::vector<int>* selectedIndices) {
    std::vector<PreparedTriangle> triangles;
    glm::vec3 sceneMin(std::numeric_limits<float>::infinity());
    glm::vec3 sceneMax(-std::numeric_limits<float>::infinity());

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

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
        std::vector<glm::vec3> vertices;
        vertices.reserve(mesh->mNumVertices);

        // Convert vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            const auto& v = mesh->mVertices[i];
            glm::vec3 vertex(v.x * scale, v.y * scale, v.z * scale);
            vertices.push_back(vertex);

            // Update scene bounds
            sceneMin = glm::min(sceneMin, vertex);
            sceneMax = glm::max(sceneMax, vertex);
        }

        triangles.reserve(mesh->mNumFaces);

        // Create triangles
        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            const aiFace& face = mesh->mFaces[i];
            if (face.mNumIndices == 3) {
                triangles.push_back(PreparedTriangle::fromVertexArray(
                    triangleIdx++,
                    vertices,
                    face.mIndices[0],
                    face.mIndices[1],
                    face.mIndices[2]
                ));
            }
        }
    }

    return {triangles, sceneMin, sceneMax};
}

const uint8_t KTX_Signature [] = { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A };
const int KTX_FLOAT = 0x1406;
const int KTX_R32F = 0x822E;

class binary_ofstream : public std::ofstream {
public:
    using std::ofstream::ofstream;  // inherit constructors
    
    // Original write method
    binary_ofstream& write(const char* data, std::streamsize count) {
        std::ofstream::write(data, count);
        return *this;
    }
    
    // Template write for arithmetic types
    template<typename T>
    typename std::enable_if_t<std::is_arithmetic_v<T>, binary_ofstream&>
    write(const T& value) {
        std::ofstream::write(reinterpret_cast<const char*>(&value), sizeof(T));
        return *this;
    }
};

void saveKTX(int format, int width, int height, int depth, std::vector<float> & data, std::string outputFile) {
    binary_ofstream writer(outputFile, std::ios::binary);
    if (!writer) {
        throw std::runtime_error("Failed to open output file: " + outputFile);
    }

    writer.write(reinterpret_cast<const char*>(KTX_Signature), sizeof(KTX_Signature));
    writer.write(0x04030201);
    writer.write(KTX_FLOAT);
    writer.write(4); // raw size
    writer.write(format); // raw format
    writer.write(format); // format
    writer.write(format); 
    writer.write(width);
    writer.write(height);
    writer.write(depth);
    writer.write(0); // elements
    writer.write(1); // faces
    writer.write(data.size()); // mipmaps
    writer.write(0); // metadata
    writer.write(data.size() * sizeof(float)); // current mipmap size

    for (size_t i = 0; i < data.size(); i+=4)
        writer.write(data[i]);

    writer.close();
}

ProcessingMetadata processModel(const std::string& filename, const std::string& outputFile,
                              int pixels = 256, int topLodCellSize = 8, float scale = 1.0f,
                              const std::vector<int>* selectedIndices = nullptr) {
    std::chrono::steady_clock::time_point startTime = std::chrono::high_resolution_clock::now();
    std::cout << timestamp(startTime) << "Processing file " << filename << std::endl;

    // Load and prepare scene
    auto [triangles, sceneMin, sceneMax] = prepareScene(filename, scale, selectedIndices);
    
    // Calculate scene dimensions
    float maxSide = std::max({
        sceneMax.z - sceneMin.z,
        sceneMax.y - sceneMin.y,
        sceneMax.x - sceneMin.x
    });

    // Calculate conversion factors and dimensions
    int paddedTopLodCellSize = topLodCellSize;
    topLodCellSize -= 1;

    float sceneToPixels = pixels / maxSide;
    float pixelsToScene = 1.0f / sceneToPixels;

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
    glm::vec3 padding(padx, pady, padz);
    padding *= pixelsToScene * 0.5f;
    glm::vec3 lowerBound = sceneMin - padding;
    glm::vec3 upperBound = sceneMax + padding;

    std::cout << timestamp(startTime) 
              << "File preprocessed. "
              << "X: " << sx << ", Y: " << sy << ", Z: " << sz 
              << ", maximum distance: " << maxSide 
              << std::endl;

    // Create triangle grid
    TriangleGrid triangleGrid(sceneMin, sceneMax, sx / topLodCellSize + (sx % topLodCellSize != 0), sy / topLodCellSize+ (sy % topLodCellSize != 0), sz / topLodCellSize + (sz % topLodCellSize != 0), triangles);

    std::cout << timestamp(startTime)
              << "Triangle grid ready: " 
              << triangleGrid.getTriangleCount() 
              << std::endl;

    // Generate distance field
    std::vector<float> distanceData = triangleGrid.dispatch(
        lowerBound,
        pixelsToScene,
        sceneToPixels / paddedTopLodCellSize,
        sx, sy, sz
    );

    std::cout << timestamp(startTime)
              << "Triangle grid " << std::endl;

    // Save to file
    saveKTX(KTX_R32F, sx, sy, sz, distanceData, outputFile);

    std::cout << timestamp(startTime)
              << "Distance field saved to " << outputFile << std::endl;

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
        float scale = 1.0f;
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