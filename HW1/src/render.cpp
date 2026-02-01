// src/render.cpp
#include "MeshOBJ.h"
#include "camera.h"
#include "ray.h"
#include "raytracer.h"
#include "vec3.h"
#include "material.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// -------- Material presets for BRDF testing --------

// Matte diffuse (pure Lambert)
static Material matte_red() {
    Material m;
    m.type = MAT_LAMBERTIAN;
    m.albedo = make_vec3(0.8f, 0.2f, 0.2f);
    m.kd = 1.0f;
    m.ks = 0.0f;
    m.shininess = 1.0f;
    return m;
}

// Plastic (diffuse + soft specular highlight)
static Material plastic_red() {
    Material m;
    m.type = MAT_LAMBERTIAN;
    m.albedo = make_vec3(0.8f, 0.2f, 0.2f);
    m.kd = 1.0f;
    m.ks = 0.25f;
    m.specularColor = make_vec3(0.04f, 0.04f, 0.04f);
    m.shininess = 64.0f;
    return m;
}

// Glossy plastic (tight highlight)
static Material glossy_plastic() {
    Material m;
    m.type = MAT_LAMBERTIAN;
    m.albedo = make_vec3(0.8f, 0.2f, 0.2f);
    m.kd = 1.0f;
    m.ks = 0.5f;
    m.specularColor = make_vec3(0.04f, 0.04f, 0.04f);
    m.shininess = 128.0f;
    return m;
}

// Brushed metal (tinted specular, no diffuse)
static Material metal_red() {
    Material m;
    m.type = MAT_METAL;
    m.albedo = make_vec3(0.8f, 0.2f, 0.2f);
    m.kd = 0.0f;
    m.ks = 1.0f;
    m.specularColor = m.albedo; // metal = colored specular
    m.shininess = 64.0f;
    return m;
}

// Mirror-like metal (very sharp)
static Material mirror_metal() {
    Material m;
    m.type = MAT_METAL;
    m.albedo = make_vec3(1.0f, 1.0f, 1.0f);
    m.kd = 0.0f;
    m.ks = 1.0f;
    m.specularColor = m.albedo;
    m.shininess = 512.0f;
    return m;
}

// -------- Render + write one PNG --------
static void render_and_write_png(const MeshSOA& mesh,
                                 const camera& cam,
                                 const Light& light,
                                 const Material& mat,
                                 const std::string& output_filename)
{
    const int pixel_width  = cam.pixel_width;
    const int pixel_height = cam.pixel_height;
    const size_t indexCount = mesh.indices.size();

    std::vector<Vec3> image(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height));

    const Vec3 center = cam.get_center();

    for (int j = 0; j < pixel_height; ++j) {
        if (j % 10 == 0) {
            std::cout << "\r  scanlines remaining: " << (pixel_height - j) << "   " << std::flush;
        }

        for (int i = 0; i < pixel_width; ++i) {
            Ray r(center, cam.get_pixel_position(i, j) - center);

            HitRecord closest{};
            closest.hit = false;
            closest.t = std::numeric_limits<float>::max();

            Vec3 color = shade(r, closest, light); // background

            for (size_t k = 0; k < indexCount; k += 3) {
                Triangle tri;
                tri.mat = mat;

                tri.v0 = mesh.positions[mesh.indices[k]];
                tri.v1 = mesh.positions[mesh.indices[k + 1]];
                tri.v2 = mesh.positions[mesh.indices[k + 2]];

                if (mesh.hasNormals()) {
                    tri.n0 = mesh.normals[mesh.indices[k]];
                    tri.n1 = mesh.normals[mesh.indices[k + 1]];
                    tri.n2 = mesh.normals[mesh.indices[k + 2]];
                } else {
                    Vec3 faceN = unit_vector(cross(tri.v1 - tri.v0, tri.v2 - tri.v0));
                    tri.n0 = tri.n1 = tri.n2 = faceN;
                }

                HitRecord rec = ray_intersection(r, tri);
                if (rec.hit && rec.t < closest.t) {
                    color = shade(r, rec, light);
                    closest = rec;
                }
            }

            image[static_cast<size_t>(j) * static_cast<size_t>(pixel_width) + static_cast<size_t>(i)] = color;
        }
    }

    std::cout << "\r  writing: " << output_filename << "                         \n";

    std::vector<unsigned char> png_data(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height) * 3);

    for (int k = 0; k < pixel_width * pixel_height; ++k) {
        Vec3 c = clamp(image[static_cast<size_t>(k)]);
        png_data[static_cast<size_t>(k) * 3 + 0] = static_cast<unsigned char>(255.99f * c.x);
        png_data[static_cast<size_t>(k) * 3 + 1] = static_cast<unsigned char>(255.99f * c.y);
        png_data[static_cast<size_t>(k) * 3 + 2] = static_cast<unsigned char>(255.99f * c.z);
    }

    stbi_write_png(output_filename.c_str(),
                   pixel_width, pixel_height,
                   3, png_data.data(),
                   pixel_width * 3);
}

int main(int argc, char** argv)
{
    using vec3   = Vec3;
    using point3 = Vec3;

    std::string path = "../assets/meshes/frog.obj"; // change as wanted.
    // the sphere is too close for me for some reason so I stuck with the frog
    if (argc >= 2) path = argv[1];

    std::cout << "Loading OBJ: " << path << "\n";

    MeshSOA mesh;
    if (!LoadOBJ_ToMeshSOA(path, mesh)) {
        std::cerr << "Failed to load OBJ: " << path << "\n";
        return 1;
    }

    const size_t vertexCount = mesh.positions.size();
    const size_t triCount = mesh.indices.size() / 3;

    std::cout << "Loaded OBJ: " << path << "\n";
    std::cout << "Vertices:   " << vertexCount << "\n";
    std::cout << "Triangles:  " << triCount << "\n";
    std::cout << "Has UVs:    " << (mesh.hasUVs() ? "yes" : "no") << "\n";
    std::cout << "Has Normals:" << (mesh.hasNormals() ? "yes" : "no") << "\n";

    // Camera (frog-friendly)
    const point3 camera_position{0.0f, -1.0f, 1.0f};
    const point3 look_at{0.0f, 0.15f, 0.0f};
    const vec3   up{0.0f, 0.0f, 1.0f};

    const double focal_length_mm  = 255.0;
    const double sensor_height_mm = 24.0; // full-frame-ish
    const int pixel_width  = 320;
    const int pixel_height = 180;

    camera cam(camera_position, look_at, up,
               focal_length_mm, sensor_height_mm,
               pixel_width, pixel_height);

    // Light moved: near camera makes specular differences obvious for debugging.
    Light light;
    light.position = make_vec3(0.0f, -1.0f, 1.0f);
    light.color    = make_vec3(1.0f, 1.0f, 1.0f);

    struct NamedMat { const char* name; Material mat; };
    const std::vector<NamedMat> mats = {
        {"matte_red",       matte_red()},
        {"plastic_red",     plastic_red()},
        {"glossy_plastic",  glossy_plastic()},
        {"metal_red",       metal_red()},
        {"mirror_metal",    mirror_metal()},
    };

    std::string meshName = "mesh";
    if (path.find("frog") != std::string::npos)   meshName = "frog";
    if (path.find("sphere") != std::string::npos) meshName = "sphere";

    for (const auto& nm : mats) {
        std::cout << "Rendering: " << meshName << " + " << nm.name << "\n";
        std::string out = meshName + "_" + std::string(nm.name) + ".png";
        render_and_write_png(mesh, cam, light, nm.mat, out);
    }

    std::cout << "All renders complete.\n";
    return 0;
}
