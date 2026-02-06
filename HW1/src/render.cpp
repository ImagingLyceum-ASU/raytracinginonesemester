// src/render.cpp
#include "MeshOBJ.h"
#include "camera.h"
#include "ray.h"
#include "raytracer.h"
#include "scene_loader.h"
#include "transform.h"
#include "vec3.h"
#include "material.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <limits>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


int main(int argc, char** argv)
{
    // 1) Choose scene JSON
    std::string config_path = "config/sphere.json";
    if (argc >= 2) config_path = argv[1];

    // Run from project root
    std::string base_dir = ".";

    SceneConfig config;
    try {
        config = SceneLoader::load(config_path, base_dir);
    } catch (const std::exception& e) {
        std::cerr << "SceneLoader error: " << e.what() << "\n";
        return 1;
    }

    // 2) Camera from config
    camera cam = SceneLoader::make_camera(config.camera);

    // 3) Lights from config (should hold for multiple lights)
    std::vector<Light> lights;
    {
        Light L;
        L.position = make_vec3(config.light.position.x, config.light.position.y, config.light.position.z);
        L.color    = make_vec3(config.light.color.x,    config.light.color.y,    config.light.color.z);
        L.intensity = config.light.intensity;
        lights.push_back(L);
    }

    // 4) Load all scene meshes and bake transforms into triangles; material per node from JSON
    std::vector<Triangle> tris;
    for (const auto& node : config.scene) {
        if (node.type != "mesh" || node.path.empty()) continue;

        std::cout << "Loading mesh node '" << node.name << "': " << node.path << "\n";

        MeshSOA mesh;
        if (!LoadOBJ_ToMeshSOA(node.path, mesh)) {
            std::cerr << "Failed to load OBJ: " << node.path << "\n";
            return 1;
        }
        const size_t vc = mesh.positions.size();
        const size_t tc = mesh.indices.size() / 3;
        std::cout << "  Vertices: " << vc << ", Triangles: " << tc << "\n";

        // Apply node transform to mesh in-place (positions + normals)
        ApplyTransformToMeshSOA(mesh, node.transform);

        Material mat = SceneLoader::make_material(node.material);

        // Convert mesh to triangles once
        const size_t indexCount = mesh.indices.size();
        tris.reserve(tris.size() + indexCount / 3);

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

            tris.push_back(tri);
        }
    }

    if (tris.empty()) {
        std::cerr << "Scene contains no triangles.\n";
        return 1;
    }

    // 6) Render the image
    const int pixel_width  = cam.pixel_width;
    const int pixel_height = cam.pixel_height;

    std::vector<Vec3> image(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height));
    const Vec3 center = cam.get_center();

    const int maxDepth = std::max(1, config.settings.max_bounces);

    const int bar_width = 40;
    for (int j = 0; j < pixel_height; ++j) {
        const int pct = (j + 1) * 100 / pixel_height;
        const int filled = (j + 1) * bar_width / pixel_height;
        std::cerr << "\r[" << std::string(filled, '=') << std::string(bar_width - filled, ' ') << "] " << pct << "%" << std::flush;

        for (int i = 0; i < pixel_width; ++i) {
            Ray r(center, cam.get_pixel_position(i, j) - center);
            Vec3 color = TraceRay(r, tris, lights, maxDepth);
            image[static_cast<size_t>(j) * static_cast<size_t>(pixel_width) + static_cast<size_t>(i)] = color;
        }
    }
    std::cerr << "\r[" << std::string(bar_width, '=') << "] 100%\n";

    // 7) Write PNG
    std::string out = "output.png";
    std::cout << "writing: " << out << "\n";

    std::vector<unsigned char> png_data(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height) * 3);
    for (int k = 0; k < pixel_width * pixel_height; ++k) {
        Vec3 c = clamp(image[static_cast<size_t>(k)]);
        png_data[static_cast<size_t>(k) * 3 + 0] = static_cast<unsigned char>(255.99f * c.x);
        png_data[static_cast<size_t>(k) * 3 + 1] = static_cast<unsigned char>(255.99f * c.y);
        png_data[static_cast<size_t>(k) * 3 + 2] = static_cast<unsigned char>(255.99f * c.z);
    }

    stbi_write_png(out.c_str(), pixel_width, pixel_height, 3, png_data.data(), pixel_width * 3);

    std::cout << "Done.\n";
    return 0;
}

