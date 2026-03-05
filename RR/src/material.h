#pragma once
#include "vec3.h"
#include "texture.h"

struct Material {
    Vec3  albedo        = {0.8f,0.8f,0.8f};
    float kd            = 1.0f;
    Vec3  specularColor = {0.04f,0.04f,0.04f};
    float ks            = 0.0f;
    float shininess     = 32.0f;
    float kr            = 0.0f;
    Vec3  emission      = {0.0f,0.0f,0.0f};
    Texture albedo_map;   // valid()==false when no texture
    Texture bump_map;
};
