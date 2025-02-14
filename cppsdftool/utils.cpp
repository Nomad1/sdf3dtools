#include "utils.hpp"
#include <iomanip>
#include <sstream>

namespace
{
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
}

binary_ofstream &binary_ofstream::write(const char *data, std::streamsize count)
{
    std::ofstream::write(data, count);
    return *this;
}

void binary_ofstream::pad_to(std::streampos position)
{
    while (tellp() < position)
    {
        write((int)0);
    }
}

template <typename T>
typename std::enable_if_t<std::is_arithmetic_v<T>, binary_ofstream &>
binary_ofstream::write(const T &value)
{
    std::ofstream::write(reinterpret_cast<const char *>(&value), sizeof(T));
    return *this;
}

std::string timestamp()
{
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

void saveKTX(uint format, uint width, uint height, uint depth,
             std::vector<float> &data, const std::string &outputFile, uint stride)
{

    binary_ofstream writer(outputFile, std::ios::binary);
    if (!writer)
    {
        throw std::runtime_error("Failed to open output file: " + outputFile);
    }

    writer.write(reinterpret_cast<const char *>(KTX_SIGNATURE), sizeof(KTX_SIGNATURE));
    writer.write(0x04030201);
    writer.write(KTX_FLOAT);
    writer.write(4);      // raw size
    writer.write(format); // raw format
    writer.write(format); // format
    writer.write(format);
    writer.write(width);
    writer.write(height);
    writer.write(depth);
    writer.write(0); // elements
    writer.write(1); // faces
    writer.write(1); // mipmaps
    writer.write(0); // metadata

    writer.pad_to(64); // Ensure header is properly padded

    if (stride == 0)
        stride = 1;

    writer.write((uint)(data.size() * 4 / stride)); // current mipmap size

    for (size_t i = 0; i < data.size(); i += stride)
    {
        writer.write(data[i]);
    }
}


// Explicit template instantiation for double
template void savePoints<double>(uint width, uint height, uint depth, uint cellSize, 
                                 float lbx, float lby, float lbz,
                                 float ubx, float uby, float ubz,
                                 const std::vector<std::vector<double>>& data, const std::string& outputFile,
                                 bool hasUv,
                                 bool hasBones);


// Explicit template instantiation for float
template void savePoints<float>(uint width, uint height, uint depth, uint cellSize, 
                                 float lbx, float lby, float lbz,
                                 float ubx, float uby, float ubz,
                                 const std::vector<std::vector<float>>& data, const std::string& outputFile,
                                 bool hasUv,
                                 bool hasBones);


template<typename T>
void savePoints(uint width, uint height, uint depth, uint cellSize, 
                float lbx, float lby, float lbz,
                float ubx, float uby, float ubz,
                const std::vector<std::vector<T>>& data, const std::string& outputFile,
                bool hasUv,
                bool hasBones)
{
    static_assert(std::is_floating_point_v<T>, "Type must be floating point");

    binary_ofstream writer(outputFile, std::ios::binary);
    if (!writer)
    {
        throw std::runtime_error("Failed to open output file: " + outputFile);
    }

    writer.write(reinterpret_cast<const char *>(POINTS_SIGNATURE), sizeof(POINTS_SIGNATURE));
    writer.write(width);
    writer.write(height);
    writer.write(depth);
    writer.write(cellSize);
    writer.write(lbx);
    writer.write(lby);
    writer.write(lbz);
    writer.write(ubx);
    writer.write(uby);
    writer.write(ubz);

    uint flags = 0;
    if (hasUv)
        flags |= POINTS_FLAG_UVS;
    if (hasBones)
        flags |= POINTS_FLAG_BONES;
    if (data.size() > 1)
        flags |= POINTS_FLAG_LODS;

    writer.write(flags);

    writer.write((uint)data.size());

    int stride = 4; // TODO: change if bones are included

    for (size_t l = 0; l < data.size(); ++l)
    {
        const std::vector<T> &lodData = data[l];

        writer.write((uint)(lodData.size() / stride));

        for (size_t i = 0; i < lodData.size(); i += stride)
        {
            writer.write(static_cast<float>(lodData[i + 0]));

            if (hasUv)
            {
                writer.write(static_cast<float>(lodData[i + 1]));
                writer.write(static_cast<float>(lodData[i + 2]));
            }

            if (hasBones)
            {
                writer.write(0);
                writer.write(0);
                writer.write(0);
                writer.write(0);
                writer.write(0.0f);
                writer.write(0.0f);
                writer.write(0.0f);
                writer.write(0.0f);
            }
        }
    }
}
