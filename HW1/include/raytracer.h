#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include "ray.h"
#include "brdf.h"

// Random [0, 1) for diffuse sampling
inline float random_float() {
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(gen);
}

inline Vec3 random_unit_vector() {
    for (;;) {
        float x = 2.0f * random_float() - 1.0f;
        float y = 2.0f * random_float() - 1.0f;
        float z = 2.0f * random_float() - 1.0f;
        float lensq = x*x + y*y + z*z;
        if (lensq > 1e-10f && lensq <= 1.0f) {
            float inv = 1.0f / sqrtf(lensq);
            return make_vec3(x * inv, y * inv, z * inv);
        }
    }
}

inline Vec3 random_on_hemisphere(const Vec3& normal) {
    Vec3 on_unit_sphere = random_unit_vector();
    if (dot(on_unit_sphere, normal) > 0.0f)
        return on_unit_sphere;
    return make_vec3(-on_unit_sphere.x, -on_unit_sphere.y, -on_unit_sphere.z);
}

struct Light {
    Vec3 position;
    Vec3 color;
    float intensity = 1.0f;  // light intensity multiplier
};

// Small offset to avoid self-intersection / "shadow acne"
static constexpr float RT_EPS = 1e-4f;

HYBRID_FUNC inline Vec3 clamp(Vec3 color) {
    if (color.x > 1.0f) color.x = 1.0f;
    if (color.y > 1.0f) color.y = 1.0f;
    if (color.z > 1.0f) color.z = 1.0f;
    if (color.x < 0.0f) color.x = 0.0f;
    if (color.y < 0.0f) color.y = 0.0f;
    if (color.z < 0.0f) color.z = 0.0f;
    return color;
}

HYBRID_FUNC inline float length3(const Vec3& v) {
    return sqrtf(dot(v, v));
}

HYBRID_FUNC inline Vec3 reflect_dir(const Vec3& I, const Vec3& N) {
    // I points *in the ray direction* (from origin toward scene)
    // Reflection: R = I - 2*(I·N)*N
    return I - (2.0f * dot(I, N)) * N;
}

// Scene intersection: find closest hit among triangles
inline bool IntersectScene(const Ray& r,
                           const std::vector<Triangle>& tris,
                           double t_min,
                           double t_max,
                           HitRecord& outRec)
{
    bool hit_anything = false;
    double closest = t_max;

    HitRecord temp{};
    for (const auto& tri : tris) {
        temp = ray_intersection(r, tri);
        if (!temp.hit) continue;

        if (temp.t >= t_min && temp.t < closest) {
            hit_anything = true;
            closest = temp.t;
            outRec = temp;
        }
    }

    return hit_anything;
}

// Check if there is any occluder between P and the light
inline bool IsInShadow(const Vec3& P,
                       const Vec3& N,
                       const Light& light,
                       const std::vector<Triangle>& tris)
{
    Vec3 toL = light.position - P;
    float distToL = length3(toL);
    if (distToL <= 0.0f) return false;

    Vec3 Ldir = toL / distToL;

    // offset along normal to avoid self-hit
    Ray shadowRay(P + N * RT_EPS, Ldir);

    HitRecord shadowHit{};
    // Only count occluders strictly before the light !!!
    return IntersectScene(shadowRay, tris, RT_EPS, double(distToL) - RT_EPS, shadowHit);
}

// Direct lighting with hard shadows (should work with multiple lights)
inline Vec3 ShadeDirect(const Ray& r,
                        const HitRecord& rec,
                        const std::vector<Light>& lights,
                        const std::vector<Triangle>& tris)
{
    // Assuming rec.hit == true already
    Vec3 N = unit_vector(rec.normal);
    Vec3 V = unit_vector(r.origin() - rec.p);

    Vec3 Lo = make_vec3(0,0,0);

    // small ambient (looks nicer)
    Vec3 ambient = rec.mat.albedo * 0.05f;
    Lo = Lo + ambient;

    // add emission (placeholder math for now)
    Lo = Lo + rec.mat.emission;

    for (const auto& light : lights) {
        Vec3 L = unit_vector(light.position - rec.p);
        float NdotL = fmaxf(dot(N, L), 0.0f);
        if (NdotL <= 0.0f) continue;

        // Hard shadows
        if (IsInShadow(rec.p, N, light, tris)) {
            continue;
        }

        // BRDF value
        Vec3 f = EvaluateBRDF(rec.mat, N, V, L);

        // Light contribution
        Vec3 radiance = light.color * light.intensity;
        Vec3 direct = (radiance * f) * NdotL;

        Lo = Lo + direct;
    }

    return Lo;
}

// Recursive tracer: direct + perfect mirror where possible
// diffuse_bounce: true = randomly select diffuse/mirror (Russian Roulette); false = only mirror
inline Vec3 TraceRay(const Ray& r,
                     const std::vector<Triangle>& tris,
                     const std::vector<Light>& lights,
                     int depth,
                     bool diffuse_bounce = true)
{
    // If out of bounces
    if (depth <= 0) return make_vec3(0,0,0);

    HitRecord rec{};
    if (!IntersectScene(r, tris, RT_EPS, std::numeric_limits<double>::infinity(), rec)) {
        // same !rec.hit logic as before (sky gradient)
        Vec3 unit_dir = unit_vector(r.direction());
        float t = 0.5f * (unit_dir.z + 1.0f);
        return make_vec3(1.0f, 1.0f, 1.0f)*(1.0f-t) + make_vec3(0.5f, 0.7f, 1.0f)*t;
    }

    // Geometry terms
    Vec3 N = unit_vector(rec.normal);

    // 1) direct lighting (with shadows)
    Vec3 Lo = ShadeDirect(r, rec, lights, tris);

    // 2) indirect: randomly select diffuse/mirror (diffuse_bounce is true, otherwise only mirror)
    float kd = rec.mat.kd;
    float kr = rec.mat.kr;
    float total = kd + kr;
    if (total > 0.0f) {
        float xi = random_float();
        if (diffuse_bounce && xi < kd / total) {
            Vec3 diffuse_dir = random_on_hemisphere(N);
            Ray diffuse_ray(rec.p + N * RT_EPS, diffuse_dir);
            Vec3 bounced = TraceRay(diffuse_ray, tris, lights, depth - 1, diffuse_bounce);
            float NdotL = fmaxf(dot(N, diffuse_dir), 0.0f);
            Lo = Lo + (rec.mat.albedo * total * 2.0f * NdotL * bounced);
        } else if (kr > 0.0f) {
            Vec3 refl = reflect_dir(unit_vector(r.direction()), N);
            Ray rr(rec.p + N * RT_EPS, refl);
            Vec3 bounced = TraceRay(rr, tris, lights, depth - 1, diffuse_bounce);
            Vec3 tint = rec.mat.specularColor;
            Lo = Lo + ((diffuse_bounce ? total : rec.mat.kr) * (tint * bounced));
        }
    }

    return clamp(Lo);
}

#endif