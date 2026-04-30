#ifndef TEXTURE_H
#define TEXTURE_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdio>

#include "vec3.h"
#include "imports.h"

// ============================================================
// TextureData — lightweight, GPU-copyable view (no ownership)
// This is what gets uploaded to device memory and used in kernels.
// ============================================================
struct TextureData {
    int width    = 0;
    int height   = 0;
    int channels = 0;
    const unsigned char* data = nullptr;  // pointer to pixel bytes (RGB or RGBA)

    // Sample the texture at (u, v) in [0,1]^2.  Returns linear-space RGB.
    // Uses bilinear filtering with wrap addressing.
    HYBRID_FUNC inline Vec3 sample(float u, float v) const {
        if (data == nullptr || width <= 0 || height <= 0) {
            return make_vec3(1.0f, 0.0f, 1.0f);  // magenta = missing texture
        }

        // Wrap UVs to [0, 1)
        u = u - floorf(u);
        v = v - floorf(v);

        // Flip V: OBJ convention has (0,0) at bottom-left, image has it at top-left
        v = 1.0f - v;

        // Pixel coordinates (continuous)
        float fx = u * (float)width  - 0.5f;
        float fy = v * (float)height - 0.5f;

        int x0 = (int)floorf(fx);
        int y0 = (int)floorf(fy);
        float dx = fx - (float)x0;
        float dy = fy - (float)y0;

        // Wrap pixel coords
        int x1 = (x0 + 1) % width;
        int y1 = (y0 + 1) % height;
        x0 = ((x0 % width) + width) % width;
        y0 = ((y0 % height) + height) % height;

        // Fetch four texels
        auto fetch = [&](int px, int py) -> Vec3 {
            int idx = (py * width + px) * channels;
            float r = (float)data[idx + 0] / 255.0f;
            float g = (float)data[idx + 1] / 255.0f;
            float b = (float)data[idx + 2] / 255.0f;
            return make_vec3(r, g, b);
        };

        Vec3 c00 = fetch(x0, y0);
        Vec3 c10 = fetch(x1, y0);
        Vec3 c01 = fetch(x0, y1);
        Vec3 c11 = fetch(x1, y1);

        // Bilinear interpolation
        Vec3 top    = c00 * (1.0f - dx) + c10 * dx;
        Vec3 bottom = c01 * (1.0f - dx) + c11 * dx;
        return top * (1.0f - dy) + bottom * dy;
    }

    // Sample the alpha channel at (u, v) in [0,1]^2.  Returns [0,1].
    // If the texture has >= 4 channels, reads channel 3 (RGBA alpha).
    // Otherwise reads channel 0 (grayscale/R-only mask texture).
    HYBRID_FUNC inline float sampleAlpha(float u, float v) const {
        if (data == nullptr || width <= 0 || height <= 0) return 1.0f;

        u = u - floorf(u);
        v = 1.0f - (v - floorf(v));

        float fx = u * (float)width  - 0.5f;
        float fy = v * (float)height - 0.5f;

        int x0 = (int)floorf(fx);
        int y0 = (int)floorf(fy);
        float dx = fx - (float)x0;
        float dy = fy - (float)y0;

        int x1 = (x0 + 1) % width;
        int y1 = (y0 + 1) % height;
        x0 = ((x0 % width)  + width)  % width;
        y0 = ((y0 % height) + height) % height;

        auto fetchA = [&](int px, int py) -> float {
            int idx = (py * width + px) * channels;
            if (channels >= 4) return (float)data[idx + 3] / 255.0f;
            return (float)data[idx + 0] / 255.0f;  // grayscale / R-channel mask
        };

        float a00 = fetchA(x0, y0);
        float a10 = fetchA(x1, y0);
        float a01 = fetchA(x0, y1);
        float a11 = fetchA(x1, y1);

        float top    = a00 * (1.0f - dx) + a10 * dx;
        float bottom = a01 * (1.0f - dx) + a11 * dx;
        return top * (1.0f - dy) + bottom * dy;
    }

    // Sample a tangent-space normal map.  Returns a Vec3 in [-1, 1]^3.
    HYBRID_FUNC inline Vec3 sampleNormal(float u, float v) const {
        Vec3 rgb = sample(u, v);
        // Convert from [0,1] to [-1,1]
        return make_vec3(rgb.x * 2.0f - 1.0f,
                         rgb.y * 2.0f - 1.0f,
                         rgb.z * 2.0f - 1.0f);
    }
};


// ============================================================
// Texture — host-side owner of pixel data
// ============================================================
struct Texture {
    int width    = 0;
    int height   = 0;
    int channels = 0;                       // 3 or 4
    std::vector<unsigned char> data;         // interleaved RGB/RGBA, row-major

    TextureData sampled;                     // lightweight view for GPU

    void refreshSampledView() {
        sampled.width    = width;
        sampled.height   = height;
        sampled.channels = channels;
        sampled.data     = data.empty() ? nullptr : data.data();
    }
};

// Global texture cache (host side) — keyed by file path.
extern std::unordered_map<std::string, std::unique_ptr<Texture>> g_textureCache;

// ============================================================
// HDRTextureData — GPU-copyable view into a float-precision
// equirectangular environment map.  data points to interleaved
// RGB floats in linear light (as loaded by stbi_loadf).
// ============================================================
struct HDRTextureData {
    int          width  = 0;
    int          height = 0;
    const float* data            = nullptr;
    const float* marginal_cdf    = nullptr;
    const float* marginal_pdf    = nullptr;
    const float* conditional_cdf = nullptr;
    const float* conditional_pdf = nullptr;

    HYBRID_FUNC inline bool hasImportanceSampling() const {
        return marginal_cdf != nullptr && marginal_pdf != nullptr &&
               conditional_cdf != nullptr && conditional_pdf != nullptr;
    }

    // Bilinear sample at (u, v) in [0,1]^2, linear-light RGB.
    // u wraps; v is clamped (no wrap at poles).
    HYBRID_FUNC inline Vec3 sample(float u, float v) const {
        if (!data || width <= 0 || height <= 0)
            return make_vec3(0.5f, 0.72f, 0.98f);  // fallback blue

        // Wrap u, clamp v
        u = u - floorf(u);
        v = fminf(fmaxf(v, 0.0f), 1.0f);

        // Flip V: image row-0 = top = sky (our V=1 = zenith)
        v = 1.0f - v;

        float fx = u * (float)(width  - 1);
        float fy = v * (float)(height - 1);

        int x0 = (int)fx;
        int y0 = (int)fy;
        float dx = fx - (float)x0;
        float dy = fy - (float)y0;

#ifdef __CUDA_ARCH__
        int x1 = min(x0 + 1, width  - 1);
        int y1 = min(y0 + 1, height - 1);
        x0 = max(x0, 0);  y0 = max(y0, 0);
#else
        int x1 = std::min(x0 + 1, width  - 1);
        int y1 = std::min(y0 + 1, height - 1);
        x0 = std::max(x0, 0);  y0 = std::max(y0, 0);
#endif

        auto fetch = [&](int px, int py) -> Vec3 {
            int idx = (py * width + px) * 3;
            return make_vec3(data[idx], data[idx + 1], data[idx + 2]);
        };

        Vec3 c00 = fetch(x0, y0),  c10 = fetch(x1, y0);
        Vec3 c01 = fetch(x0, y1),  c11 = fetch(x1, y1);
        Vec3 top    = c00 * (1.0f - dx) + c10 * dx;
        Vec3 bottom = c01 * (1.0f - dx) + c11 * dx;
        return top * (1.0f - dy) + bottom * dy;
    }

    HYBRID_FUNC inline Vec3 sampleDirection(float xi1, float xi2, float& out_pdf) const {
        constexpr float TWO_PI = 6.28318530717958647692f;
        constexpr float PI     = 3.14159265358979323846f;

        int row = 0;
        {
            int lo = 0, hi = height - 1;
            while (lo < hi) {
                int mid = (lo + hi) / 2;
                if (marginal_cdf[mid] < xi1) lo = mid + 1;
                else hi = mid;
            }
            row = lo;
        }

        int col = 0;
        {
            const float* row_cdf = conditional_cdf + row * width;
            int lo = 0, hi = width - 1;
            while (lo < hi) {
                int mid = (lo + hi) / 2;
                if (row_cdf[mid] < xi2) lo = mid + 1;
                else hi = mid;
            }
            col = lo;
        }

        float dir_u = (col + 0.5f) / float(width);
        float dir_v = 1.0f - (row + 0.5f) / float(height);

        float phi      = TWO_PI * (dir_u - 0.5f);
        float elev     = PI * (dir_v - 0.5f);
        float z        = sinf(elev);
        float cos_elev = cosf(elev);
        Vec3 dir = make_vec3(cos_elev * cosf(phi), cos_elev * sinf(phi), z);

        float p_row     = marginal_pdf[row];
        float p_col     = conditional_pdf[row * width + col];
        float pdf_uv    = p_row * p_col * float(width * height);
        float sin_theta = sinf(PI * dir_v);
        out_pdf = pdf_uv / (2.0f * PI * PI * fmaxf(sin_theta, 1e-5f));

        return dir;
    }

    HYBRID_FUNC inline float pdfDirection(const Vec3& dir) const {
        if (!hasImportanceSampling()) return 0.0f;
        constexpr float INV_2PI = 0.15915494309f;
        constexpr float INV_PI  = 0.31830988618f;
        constexpr float PI      = 3.14159265358979323846f;

        float dz = fminf(fmaxf(dir.z, -1.0f), 1.0f);
        float u  = 0.5f + atan2f(dir.y, dir.x) * INV_2PI;
        float v  = 0.5f + asinf(dz) * INV_PI;

        int row = int(fminf(fmaxf((1.0f - v) * float(height), 0.0f), float(height - 1)));
        int col = int(fminf(fmaxf(u * float(width),  0.0f), float(width  - 1)));

        float p_row     = marginal_pdf[row];
        float p_col     = conditional_pdf[row * width + col];
        float pdf_uv    = p_row * p_col * float(width * height);
        float sin_theta = sinf(PI * v);
        return pdf_uv / (2.0f * PI * PI * fmaxf(sin_theta, 1e-5f));
    }
};

// Host-side HDR texture owner (not uploaded until main.cu does so)
struct HDRTexture {
    int width = 0, height = 0;
    std::vector<float> data;
    HDRTextureData sampled;

    std::vector<float> marginal_cdf;
    std::vector<float> marginal_pdf;
    std::vector<float> conditional_cdf;
    std::vector<float> conditional_pdf;

    void buildCDF() {
        if (width <= 0 || height <= 0 || data.empty()) return;

        constexpr float PI = 3.14159265358979323846f;
        const float inv_h = 1.0f / float(height);

        marginal_pdf.resize(height);
        marginal_cdf.resize(height);
        conditional_pdf.resize(size_t(width) * height);
        conditional_cdf.resize(size_t(width) * height);

        for (int i = 0; i < height; ++i) {
            float sin_w = sinf(PI * (i + 0.5f) * inv_h);
            float row_total = 0.0f;
            for (int j = 0; j < width; ++j) {
                int idx = (i * width + j) * 3;
                float lum = 0.2126f * data[idx] + 0.7152f * data[idx+1] + 0.0722f * data[idx+2];
                float w = fmaxf(lum * sin_w, 0.0f);
                conditional_pdf[i * width + j] = w;
                row_total += w;
            }
            marginal_pdf[i] = row_total;

            if (row_total > 0.0f) {
                const float inv_row = 1.0f / row_total;
                float cumulative = 0.0f;
                for (int j = 0; j < width; ++j) {
                    conditional_pdf[i * width + j] *= inv_row;
                    cumulative += conditional_pdf[i * width + j];
                    conditional_cdf[i * width + j] = cumulative;
                }
                conditional_cdf[i * width + width - 1] = 1.0f;
            } else {
                const float inv_w = 1.0f / float(width);
                for (int j = 0; j < width; ++j) {
                    conditional_pdf[i * width + j] = inv_w;
                    conditional_cdf[i * width + j] = (j + 1) * inv_w;
                }
            }
        }

        float total = 0.0f;
        for (int i = 0; i < height; ++i) total += marginal_pdf[i];

        if (total > 0.0f) {
            const float inv_total = 1.0f / total;
            float cumulative = 0.0f;
            for (int i = 0; i < height; ++i) {
                marginal_pdf[i] *= inv_total;
                cumulative += marginal_pdf[i];
                marginal_cdf[i] = cumulative;
            }
            marginal_cdf[height - 1] = 1.0f;
        } else {
            const float inv_h_f = 1.0f / float(height);
            for (int i = 0; i < height; ++i) {
                marginal_pdf[i] = inv_h_f;
                marginal_cdf[i] = (i + 1) * inv_h_f;
            }
        }
    }

    void refreshSampledView() {
        sampled.width  = width;
        sampled.height = height;
        sampled.data            = data.empty()            ? nullptr : data.data();
        sampled.marginal_cdf    = marginal_cdf.empty()    ? nullptr : marginal_cdf.data();
        sampled.marginal_pdf    = marginal_pdf.empty()    ? nullptr : marginal_pdf.data();
        sampled.conditional_cdf = conditional_cdf.empty() ? nullptr : conditional_cdf.data();
        sampled.conditional_pdf = conditional_pdf.empty() ? nullptr : conditional_pdf.data();
    }
};

// ============================================================
// Texture loading helper (uses stb_image, call from host only)
// Returns a pointer to the cached Texture, or nullptr on failure.
// ============================================================
#ifndef STB_IMAGE_IMPLEMENTATION
// We only declare the function; stb_image.h must be included with
// STB_IMAGE_IMPLEMENTATION defined in exactly one .cpp / .cu file.
extern "C" unsigned char* stbi_load (const char*, int*, int*, int*, int);
extern "C" float*         stbi_loadf(const char*, int*, int*, int*, int);
extern "C" void stbi_image_free(void*);
#endif

// Load a Radiance .hdr equirectangular environment map.
// Returns nullptr on failure.  Caller owns the result.
inline HDRTexture* LoadHDRTexture(const std::string& path) {
    int w, h, c;
    float* pixels = stbi_loadf(path.c_str(), &w, &h, &c, 3);  // force RGB
    if (!pixels) {
        std::fprintf(stderr, "Warning: failed to load HDR '%s'\n", path.c_str());
        return nullptr;
    }
    auto* tex = new HDRTexture();
    tex->width  = w;
    tex->height = h;
    tex->data.assign(pixels, pixels + (size_t)w * h * 3);
    stbi_image_free(pixels);
    tex->refreshSampledView();
    tex->buildCDF();
    std::printf("  -> Loaded HDR env map: %s (%dx%d)\n", path.c_str(), w, h);
    return tex;
}

inline Texture* LoadTexture(const std::string& path) {
    // Check cache first
    auto it = g_textureCache.find(path);
    if (it != g_textureCache.end()) {
        return it->second.get();
    }

    int w, h, c;
    unsigned char* pixels = stbi_load(path.c_str(), &w, &h, &c, 0);
    if (!pixels) {
        std::fprintf(stderr, "Warning: failed to load texture '%s'\n", path.c_str());
        return nullptr;
    }

    // Force to 3 or 4 channels (keep alpha if present)
    auto tex = std::make_unique<Texture>();
    tex->width    = w;
    tex->height   = h;
    tex->channels = c;  // 3 or 4 typically
    tex->data.assign(pixels, pixels + w * h * c);
    stbi_image_free(pixels);

    tex->refreshSampledView();

    Texture* ptr = tex.get();
    g_textureCache[path] = std::move(tex);
    return ptr;
}

#endif // TEXTURE_H