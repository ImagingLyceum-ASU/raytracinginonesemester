#pragma once
#include "ray.h"

struct Camera {
    Vec3 origin, lower_left, horiz, vert;
    Vec3 u,v,w;
    float lens_radius;
    int   pw, ph;

    // Build camera from standard params
    static Camera make(Vec3 lookfrom, Vec3 lookat, Vec3 vup,
                       float vfov_deg, float aspect,
                       float aperture, float focus_dist,
                       int pw, int ph)
    {
        Camera c;
        c.pw=pw; c.ph=ph;
        c.lens_radius=aperture*0.5f;
        float theta=vfov_deg*3.14159265f/180.f;
        float h=tanf(theta*0.5f);
        float vp_h=2.f*h; float vp_w=aspect*vp_h;
        c.w=normalize(lookfrom-lookat);
        c.u=normalize(cross(vup,c.w));
        c.v=cross(c.w,c.u);
        c.origin=lookfrom;
        c.horiz=c.u*(vp_w*focus_dist);
        c.vert =c.v*(vp_h*focus_dist);
        c.lower_left=c.origin-c.horiz*0.5f-c.vert*0.5f-c.w*focus_dist;
        return c;
    }

    HYBRID Ray get_ray(float s, float t, unsigned int& rng) const {
        // Thin lens for DOF
        float r1=rng_f(rng)*2.f-1.f, r2=rng_f(rng)*2.f-1.f;
        float lx=lens_radius*r1, ly=lens_radius*r2;
        Vec3 offset=u*lx+v*ly;
        Vec3 dir=lower_left+horiz*s+vert*t-origin-offset;
        return Ray(origin+offset, normalize(dir));
    }

private:
    HYBRID static float rng_f(unsigned int& st){
        st=st*1664525u+1013904223u;
        unsigned int h=st^(st>>16); h*=0x45d9f3bu; h^=h>>16;
        return float(h)/float(0xFFFFFFFFu);
    }
};
