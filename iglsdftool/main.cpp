#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/signed_distance.h>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include "utils.hpp"

void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName << " input.obj [resolution=50] [cellSize=4]" << std::endl;
    std::cerr << "  input.obj   : Input mesh file" << std::endl;
    std::cerr << "  resolution  : Grid resolution (default: 50)" << std::endl;
    std::cerr << "  cellSize     : Cell size (default: 4)" << std::endl;
}

template<typename T>
void printMatrixInfo(const std::string& name, const T& mat) {
    std::cout << name << ":" << std::endl;
    std::cout << "  Size: " << mat.rows() << " x " << mat.cols() << std::endl;
    std::cout << "  Min: " << mat.minCoeff() << std::endl;
    std::cout << "  Max: " << mat.maxCoeff() << std::endl;
    if (mat.size() < 10) {
        std::cout << "  Values: \n" << mat << std::endl;
    } else {
        std::cout << "  First few values: \n" << mat.topRows(3) << std::endl;
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[]) {
    std::cout << "=== SDF Generator Started ===" << std::endl;
    
    if (argc < 2 || argc > 4) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse command line arguments
    const char* input_file = argv[1];
    int res = 50;  // default resolution
    int cellSize = 4;  // default cell size

    if (argc >= 3) {
        try {
            res = std::stoi(argv[2]);
            if (res <= 0) {
                std::cerr << "Error: Resolution must be positive" << std::endl;
                return 1;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid resolution value" << std::endl;
            return 1;
        }
    }

    if (argc >= 4) {
        try {
            cellSize = std::stoi(argv[3]);
            if (cellSize <= 0) {
                std::cerr << "Error: Cell size must be positive" << std::endl;
                return 1;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid Cell size value" << std::endl;
            return 1;
        }
    }

    std::cout << "\nParameters:" << std::endl;
    std::cout << "  Input file: " << input_file << std::endl;
    std::cout << "  Resolution: " << res << std::endl;
    std::cout << "  Cell size: " << cellSize << std::endl;

    // Mesh vertices and faces
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

// Get file extension
    std::string filename = input_file;
    std::string ext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    std::cout << "\nReading mesh..." << std::endl;
    bool read_success = false;
    
    if (ext == "ply") {
        read_success = igl::readPLY(input_file, V, F);
    } else if (ext == "obj") {
        read_success = igl::readOBJ(input_file, V, F);
    } else {
        std::cerr << "Unsupported file format. Please use .obj or .ply files" << std::endl;
        return 1;
    }

    if (!read_success) {
        std::cerr << "Failed to read mesh: " << input_file << std::endl;
        return 1;
    }

    std::cout << "Mesh loaded successfully:" << std::endl;
    printMatrixInfo("Vertices (V)", V);
    printMatrixInfo("Faces (F)", F);

    // Create a regular grid
    std::cout << "Computing bounding box..." << std::endl;
    
    // Compute min and max manually
    double min_x = V(0,0), min_y = V(0,1), min_z = V(0,2);
    double max_x = min_x, max_y = min_y, max_z = min_z;
    
    for(int i = 0; i < V.rows(); i++) {
        min_x = std::min(min_x, V(i,0));
        min_y = std::min(min_y, V(i,1));
        min_z = std::min(min_z, V(i,2));
        max_x = std::max(max_x, V(i,0));
        max_y = std::max(max_y, V(i,1));
        max_z = std::max(max_z, V(i,2));
    }
    
    std::cout << "Original bounding box:" << std::endl;
    std::cout << "  Min: " << min_x << " " << min_y << " " << min_z << std::endl;
    std::cout << "  Max: " << max_x << " " << max_y << " " << max_z << std::endl;

    // Calculate scene dimensions
    double max_side = std::max({max_z - min_z,
                              max_y - min_y,
                              max_x - min_x});

    // Calculate conversion factors
    int top_lod_cell_size = cellSize;  // Using padding parameter as cell size
    top_lod_cell_size -= 1;
    double scene_to_pixels = res / max_side;  // Using resolution parameter as pixels
    double pixels_to_scene = 1.0 / scene_to_pixels;

    // Calculate grid dimensions
    int res_x = static_cast<int>(std::ceil((max_x - min_x) * scene_to_pixels));
    int res_y = static_cast<int>(std::ceil((max_y - min_y) * scene_to_pixels));
    int res_z = static_cast<int>(std::ceil((max_z - min_z) * scene_to_pixels));

    // Calculate padding
    int padx = res_x % top_lod_cell_size != 0 ? top_lod_cell_size - (res_x % top_lod_cell_size) : top_lod_cell_size;
    int pady = res_y % top_lod_cell_size != 0 ? top_lod_cell_size - (res_y % top_lod_cell_size) : top_lod_cell_size;
    int padz = res_z % top_lod_cell_size != 0 ? top_lod_cell_size - (res_z % top_lod_cell_size) : top_lod_cell_size;

    // Apply padding to dimensions
    res_x = res_x + padx + 1;
    res_y = res_y + pady + 1;
    res_z = res_z + padz + 1;

    std::cout << "Grid dimensions:" << std::endl;
    std::cout << "  Resolution X: " << res_x << std::endl;
    std::cout << "  Resolution Y: " << res_y << std::endl;
    std::cout << "  Resolution Z: " << res_z << std::endl;

    // Calculate bounds with padding
    double pad_x = padx * pixels_to_scene * 0.5;
    double pad_y = pady * pixels_to_scene * 0.5;
    double pad_z = padz * pixels_to_scene * 0.5;

    // Compute padded bounding box corners
    Eigen::RowVector3d corner_min(
        min_x - pad_x,
        min_y - pad_y,
        min_z - pad_z
    );
    Eigen::RowVector3d corner_max(
        max_x + pad_x,
        max_y + pad_y,
        max_z + pad_z
    );

    std::cout << "Grid dimensions:" << std::endl;
    std::cout << "  Resolution X: " << res_x << std::endl;
    std::cout << "  Resolution Y: " << res_y << std::endl;
    std::cout << "  Resolution Z: " << res_z << std::endl;
    
    // Create grid points
    std::cout << "\nGenerating grid points..." << std::endl;
    Eigen::MatrixXd GV;
    GV.resize(res_x * res_y * res_z, 3);
    for(int x = 0; x < res_x; x++) {
        for(int y = 0; y < res_y; y++) {
            for(int z = 0; z < res_z; z++) {
                const double xx = corner_min(0) + (corner_max(0) - corner_min(0)) * x / (res_x-1);
                const double yy = corner_min(1) + (corner_max(1) - corner_min(1)) * y / (res_y-1);
                const double zz = corner_min(2) + (corner_max(2) - corner_min(2)) * z / (res_z-1);
                GV.row(x + res_x * (y + res_y * z)) << xx, yy, zz;
            }
        }
    }

    max_side = std::max({corner_max(0) - corner_min(0),
                              corner_max(1) - corner_min(1),
                              corner_max(2) - corner_min(2)});

    printMatrixInfo("Grid Points (GV)", GV);

    // Prepare variables for signed distance computation
    std::cout << "\nComputing signed distances..." << std::endl;
    Eigen::VectorXd S;
    Eigen::VectorXi I;
    Eigen::MatrixXd C, N;
    
    // Compute signed distances
    igl::signed_distance(
        GV,        // Query points
        V,         // Mesh vertices
        F,         // Mesh faces
        igl::SIGNED_DISTANCE_TYPE_FAST_WINDING_NUMBER, // Distance type
        S,         // Signed distances
        I,         // Face indices
        C,         // Closest points
        N          // Normals
    );

    std::cout << "Signed distance computation completed." << std::endl;
    printMatrixInfo("Signed Distances (S)", S);
    printMatrixInfo("Face Indices (I)", I);
    printMatrixInfo("Closest Points (C)", C);
    printMatrixInfo("Normals (N)", N);

    // Save the SDF to a file
    std::cout << "\nSaving SDF to file..." << std::endl;
    std::string output_filename = std::string(input_file) + ".points";
    std::vector<float> sdf_values(S.size());
    for(int i = 0; i < S.size(); i++) {
        sdf_values[i] = static_cast<float>(S(i)) * scene_to_pixels / cellSize;
    }
    savePoints((uint)res_x, (uint)res_y, (uint)res_z, cellSize, corner_min(0), corner_min(1), corner_min(2), corner_max(0), corner_max(1), corner_max(2), sdf_values, output_filename, 1);
    
    std::cout << "\nSDF generation completed:" << std::endl;
    std::cout << "  Output file: " << output_filename << std::endl;
    std::cout << "  Grid resolution: " << res_x << "x" << res_y << "x" << res_z << std::endl;
    std::cout << "  Number of samples: " << S.size() << std::endl;
    std::cout << "  Distance range: [" << S.minCoeff() << ", " << S.maxCoeff() << "]" << std::endl;
    
    std::cout << "\n=== SDF Generator Finished ===" << std::endl;
    return 0;
}