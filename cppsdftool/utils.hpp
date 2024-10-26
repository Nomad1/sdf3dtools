#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <type_traits>

// KTX file format constants
constexpr uint8_t KTX_SIGNATURE[] = { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A };

constexpr uint KTX_RGBA16F = 0x881A;
constexpr uint KTX_RGBA16 = 0x805B;
constexpr uint KTX_RG = 0x8227;
constexpr uint KTX_RG16 = 0x822C;
constexpr uint KTX_RG16F = 0x822F;
constexpr uint KTX_R16F = 0x822D;
constexpr uint KTX_R32F = 0x822E;
constexpr uint KTX_R8 = 0x1903;
constexpr uint KTX_RGBA8 = 0x8058;
constexpr uint KTX_RGB8 = 0x1907;

constexpr uint KTX_FLOAT = 0x1406;
constexpr uint KTX_HALF_FLOAT = 0x140B;
constexpr uint KTX_KTX_UNSIGNED_BYTE = 0x1401;

// Binary output stream wrapper
class binary_ofstream : public std::ofstream {
public:
    using std::ofstream::ofstream;
    
    binary_ofstream& write(const char* data, std::streamsize count);
    
    template<typename T>
    typename std::enable_if_t<std::is_arithmetic_v<T>, binary_ofstream&>
    write(const T& value);

    void pad_to(std::streampos position);
};

// Utility functions
std::string timestamp();
void saveKTX(uint format, uint width, uint height, uint depth, 
             std::vector<float>& data, const std::string& outputFile, uint stride = 0);