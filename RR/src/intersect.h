#pragma once
#include "ray.h"
#include "mesh.h"
#include "material.h"

struct HitRecord {
    Vec3  p, normal, tangent, bitangent;
    float t=1e30f;
    float u=0,v=0;
    int   matId=-1;
    bool  hit=false;
    bool  front_face=true;
};

HYBRID inline HitRecord intersectTri(const Ray& r, const Triangle& tri,
                                     float tmin, float tmax)
{
    HitRecord rec;
    Vec3 e1=tri.v1.pos-tri.v0.pos;
    Vec3 e2=tri.v2.pos-tri.v0.pos;
    Vec3 pv=cross(r.dir,e2);
    float det=dot(e1,pv);
    if(fabsf(det)<1e-8f)return rec;
    float inv=1.f/det;
    Vec3 tv=r.orig-tri.v0.pos;
    float bu=dot(tv,pv)*inv;
    if(bu<0||bu>1)return rec;
    Vec3 qv=cross(tv,e1);
    float bv=dot(r.dir,qv)*inv;
    if(bv<0||bu+bv>1)return rec;
    float t=dot(e2,qv)*inv;
    if(t<tmin||t>tmax)return rec;

    float bw=1.f-bu-bv;
    rec.hit=true; rec.t=t;
    rec.p=r.at(t);
    rec.matId=tri.matId;

    // Interpolate shading normal
    Vec3 N=normalize(tri.v0.normal*bw+tri.v1.normal*bu+tri.v2.normal*bv);
    Vec3 gN=normalize(cross(e1,e2));
    rec.front_face=dot(r.dir,gN)<0;
    if(!rec.front_face){N=-1.f*N; gN=-1.f*gN;}
    if(dot(N,gN)<0)N=-1.f*N;
    rec.normal=N;

    // UV
    rec.u=tri.v0.u*bw+tri.v1.u*bu+tri.v2.u*bv;
    rec.v=tri.v0.v*bw+tri.v1.v*bu+tri.v2.v*bv;

    // Tangent frame
    float du1=tri.v1.u-tri.v0.u, dv1=tri.v1.v-tri.v0.v;
    float du2=tri.v2.u-tri.v0.u, dv2=tri.v2.v-tri.v0.v;
    float ddet=du1*dv2-du2*dv1;
    if(fabsf(ddet)>1e-8f){
        float fi=1.f/ddet;
        rec.tangent=normalize((e1*dv2-e2*dv1)*fi);
    } else {
        Vec3 up=fabsf(N.z)<0.9f?make_vec3(0,0,1):make_vec3(1,0,0);
        rec.tangent=normalize(cross(up,N));
    }
    rec.tangent=normalize(rec.tangent-N*dot(N,rec.tangent));
    rec.bitangent=cross(N,rec.tangent);
    return rec;
}
