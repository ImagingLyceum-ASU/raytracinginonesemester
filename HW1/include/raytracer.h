#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <cmath>
#include <iostream>
#include "ray.h"
#include "brdf.h"

struct Light {
    Vec3 position;
    Vec3 color;
};

HYBRID_FUNC inline Vec3 clamp(Vec3 color) {
    if (color.x > 1.0f) color.x = 1.0f;
    if (color.y > 1.0f) color.y = 1.0f;
    if (color.z > 1.0f) color.z = 1.0f;
    if (color.x < 0.0f) color.x = 0.0f;
    if (color.y < 0.0f) color.y = 0.0f;
    if (color.z < 0.0f) color.z = 0.0f;
    return color;
}

HYBRID_FUNC Vec3 shade(const Ray& r, const HitRecord& rec, const Light& light) {
    if (!rec.hit) {
        Vec3 unit_dir = unit_vector(r.direction());
        float t = 0.5f * (unit_dir.z + 1.0f);
        return make_vec3(1.0f, 1.0f, 1.0f)*(1.0f-t) + make_vec3(0.5f, 0.7f, 1.0f)*t;
    }

    // Geometry terms
    Vec3 N = unit_vector(rec.normal);
    Vec3 V = unit_vector(r.origin() - rec.p);
    Vec3 L = unit_vector(light.position - rec.p);

    float NdotL = fmaxf(dot(N, L), 0.0f);

    Vec3 ambient = rec.mat.albedo * 0.05f;

    if (NdotL <= 0.0f) return clamp(ambient);

    // BRDF value
    Vec3 f = EvaluateBRDF(rec.mat, N, V, L);

    // Treat light.color as radiance/intensity for now
    Vec3 radiance = light.color;

    Vec3 direct = (radiance * f) * NdotL;

    return clamp(ambient + direct);
}

#endif