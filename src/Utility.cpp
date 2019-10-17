// Copyright (c) 2019, Danilo Peixoto. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <aurora/Utility.h>
#include <aurora/Math.h>
#include <aurora/Vector.h>
#include <aurora/Color.h>
#include <aurora/Image.h>

#include <vector>
#include <fstream>
#include <cstdlib>
#include <chrono>

AURORA_NAMESPACE_BEGIN

Image3 * readImage(const std::string & filename) {
    std::ifstream file(filename, std::ifstream::in | std::ofstream::binary);

    if (!file.is_open())
        return nullptr;

    std::string magic, comment;
    size_t width, height, depth;

    file >> magic;

    file.ignore();

    if (file.peek() == '#')
        std::getline(file, comment);

    file >> width >> height >> depth;

    file.ignore();

    if (magic != "P6" || depth == 0) {
        file.close();
        return nullptr;
    }

    size_t size = width * height;

    std::vector<Color3> pixels;
    pixels.resize(size);

    for (size_t i = 0; i < size; i++) {
        unsigned char data[3];

        file.read((char *)data, sizeof(data));

        pixels[i] = Color3(
            clamp(data[0] / (double)depth, 0, 1.0),
            clamp(data[1] / (double)depth, 0, 1.0),
            clamp(data[2] / (double)depth, 0, 1.0));
    }

    file.close();

    return new Image3(width, height, pixels);
}
bool writeImage(const std::string & filename, const Image3 * image3) {
    std::ofstream file(filename, std::ofstream::out | std::ofstream::trunc | std::ofstream::binary);

    if (!file.is_open())
        return false;

    size_t depth = 255;

    file << "P6" << ' ' << image3->getWidth() << ' ' << image3->getHeight() << ' ' << depth << std::endl;

    for (size_t i = 0; i < image3->getPixelCount(); i++) {
        const Color3 & pixel = (*image3)[i];
        unsigned char data[3];

        data[0] = (unsigned char)(clamp(pixel.r, 0, 1.0) * depth);
        data[1] = (unsigned char)(clamp(pixel.g, 0, 1.0) * depth);
        data[2] = (unsigned char)(clamp(pixel.b, 0, 1.0) * depth);

        file.write((const char *)data, sizeof(data));
    }

    file.close();

    return true;
}

size_t time() {
    std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
    std::chrono::system_clock::duration duration = time.time_since_epoch();

    return (size_t)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void randomSeed(size_t seed) {
    std::srand(seed);
}
double uniformRandom() {
    return std::rand() / (RAND_MAX + 1.0);
}

Color3 toColor3(const Vector3 & vector3) {
	return Color3(vector3.x, vector3.y, vector3.z);
}
Color4 toColor4(const Vector3 & vector3) {
	return Color4(vector3.x, vector3.y, vector3.z, 1.0);
}

AURORA_NAMESPACE_END
