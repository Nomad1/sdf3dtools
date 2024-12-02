# CPP SDF Tool

High-performance C++ implementation for generating Signed Distance Fields (SDFs) from 3D models, optimized for real-time graphics and physics simulations.

## Features
- Fast parallel SDF generation using OpenMP
- Support for common 3D model formats via Assimp
- High-precision distance field calculations
- LOD (Level of Detail) support

## Installation (macOS)

```bash
# Install required dependencies
brew install cmake llvm libomp glm assimp

# Clone and build
git clone https://github.com/yourusername/cppsdftool.git
cd cppsdftool
mkdir build && cd build
cmake ..
make
```

## Usage

Basic usage:
```bash
./process_model input.obj output.ktx
```

Available options:
```
--list                    List available meshes in the model and exit
--scale <value>          Scale factor for the model (default: 1.0)
--pixels <value>         Maximum pixel size for LOD 0 (default: 256)
--quality <0-3>          SDF quality level (default: 0)
--top-lod-cell-size <n>  Cell size for highest LOD (default: 8)
--indices <i1 i2 ...>    Process only specified mesh indices
```

Example:
```bash
# Generate high-quality SDF at 512 resolution
./process_model --pixels 512 --quality 2 input.obj output.ktx

# Process specific meshes from a model
./process_model --list input.obj                    # List available meshes
./process_model --indices 0 2 input.obj output.ktx  # Process meshes 0 and 2
```

## Contributing

Contributions are welcome! Please feel free to submit pull requests or create issues.