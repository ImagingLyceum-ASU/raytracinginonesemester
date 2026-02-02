#include "MeshOBJ.h"
#include "camera.h"
#include "ray.h"
#include "raytracer.h"
#include "scene_loader.h"
#include "vec3.h"
#include <vector>
#include <iostream>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main(int argc, char** argv)
{
    using vec3 = Vec3;
    using point3 = Vec3;

    // Load scene config from JSON, default config/frog.json
    std::string config_path = "config/frog.json";
    if (argc >= 2) config_path = argv[1];

    std::string base_dir = ".";
    SceneConfig config;
    try {
        config = SceneLoader::load(config_path, base_dir);
    } catch (const std::exception& e) {
        std::cerr << "SceneLoader error: " << e.what() << "\n";
        return 1;
    }

    camera cam = SceneLoader::make_camera(config.camera);
    const int pixel_width = config.camera.pixel_width;
    const int pixel_height = config.camera.pixel_height;

    // Load all meshes from scene config
    std::vector<MeshSOA> meshes;
    for (const auto& node : config.scene) {
        if (node.type != "mesh" || node.path.empty()) continue;
        std::cout << "Loading OBJ: " << node.path << "\n";
        MeshSOA mesh;
        if (!LoadOBJ_ToMeshSOA(node.path, mesh)) {
            std::cerr << "Failed to load OBJ: " << node.path << "\n";
            return 1;
        }
        const size_t vc = mesh.positions.size();
        const size_t tc = mesh.indices.size() / 3;
        std::cout << "  Vertices: " << vc << ", Triangles: " << tc << "\n";
        meshes.push_back(std::move(mesh));
    }

    Light light;
    light.position = make_vec3(-3.0f, 0.0f, 1.0f);
    light.color = make_vec3(1.0f, 1.0f, 1.0f);

    std::vector<Vec3> image(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height));
    std::string output_filename = "output.png";

    auto center = cam.get_center();
    const int bar_width = 40;
    for (int j = 0; j < pixel_height; j++) {
        int pct = pixel_height ? (j * 100 / pixel_height) : 0;
        int filled = pixel_height ? (j * bar_width / pixel_height) : 0;
        std::cerr << "\r[" << std::string(filled, '=') << std::string(bar_width - filled, ' ') << "] " << pct << "%" << std::flush;
        for (int i = 0; i < pixel_width; i++) {
            Ray r = Ray(center, cam.get_pixel_position(i, j) - center);
            HitRecord prev;
            prev.hit = false;
            prev.t = std::numeric_limits<float>::max();
            auto color = shade(r, prev, light);

            for (const auto& mesh : meshes) {
                const size_t indexCount = mesh.indices.size();
                for (size_t k = 0; k < indexCount; k += 3) {
                    Triangle tri;
                    tri.v0 = mesh.positions[mesh.indices[k]];
                    tri.v1 = mesh.positions[mesh.indices[k + 1]];
                    tri.v2 = mesh.positions[mesh.indices[k + 2]];
                    tri.n0 = mesh.normals[mesh.indices[k]];
                    tri.n1 = mesh.normals[mesh.indices[k + 1]];
                    tri.n2 = mesh.normals[mesh.indices[k + 2]];

                    HitRecord rec = ray_intersection(r, tri);
                    if (rec.hit && rec.t < prev.t) {
                        color = shade(r, rec, light);
                        prev = rec;
                    }
                }
            }

            image[static_cast<size_t>(j) * static_cast<size_t>(pixel_width) + static_cast<size_t>(i)] = color;
        }
    }
    std::cerr << "\r[" << std::string(bar_width, '=') << "] 100%\n";
    std::vector<unsigned char> png_data(static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height) * 3);
    const size_t pixel_count = static_cast<size_t>(pixel_width) * static_cast<size_t>(pixel_height);
    for (size_t k = 0; k < pixel_count; ++k) {
        png_data[k*3 + 0] = (unsigned char)(255.99f * image[k].x);
        png_data[k*3 + 1] = (unsigned char)(255.99f * image[k].y);
        png_data[k*3 + 2] = (unsigned char)(255.99f * image[k].z);
    }
    stbi_write_png(output_filename.c_str(), pixel_width, pixel_height, 3, png_data.data(), pixel_width * 3);
    std::cout << "Saved " << output_filename << "\n";

    return 0;
}
