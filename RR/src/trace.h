#pragma once
#include "ray.h"
#include "intersect.h"
#include "material.h"
#include "scene.h"
#include "bvh.h"

// ---- RNG ----
HYBRID inline float rng(unsigned int& s){
    s=s*1664525u+1013904223u;
    unsigned int h=s^(s>>16); h*=0x45d9f3bu; h^=h>>16;
    return float(h)/float(0xFFFFFFFFu);
}
HYBRID inline unsigned int rngSeed(int x,int y,int samp){
    return (unsigned int)x*73856093u^(unsigned int)y*19349663u^(unsigned int)samp*83492791u;
}
HYBRID inline Vec3 randUnitVec(unsigned int& s){
    for(;;){
        float x=2.f*rng(s)-1.f,y=2.f*rng(s)-1.f,z=2.f*rng(s)-1.f;
        float l=x*x+y*y+z*z;
        if(l>1e-10f&&l<=1.f){float inv=1.f/sqrtf(l);return {x*inv,y*inv,z*inv};}
    }
}
HYBRID inline Vec3 randHemi(Vec3 N,unsigned int& s){
    Vec3 v=randUnitVec(s);
    return dot(v,N)>0?v:-1.f*v;
}
// Disk sample for soft shadows
HYBRID inline Vec3 randDisk(unsigned int& s){
    for(;;){
        float x=2.f*rng(s)-1.f,y=2.f*rng(s)-1.f;
        if(x*x+y*y<=1.f)return {x,y,0};
    }
}

// ---- BVH traversal (GPU-safe iterative) ----
HYBRID inline HitRecord traverseBVH(
    const Ray& ray,
    const BVHNode* nodes,
    const int* triOrder,
    const GPUTri* tris,
    int /*numTris*/,
    float tmin=1e-4f)
{
    HitRecord best; best.hit=false; best.t=1e30f;
    int stack[64]; int sp=0;
    stack[sp++]=0;
    while(sp>0){
        int ni=stack[--sp];
        const BVHNode& node=nodes[ni];
        if(!node.box.hit(ray,tmin,best.t)) continue;
        if(node.left<0&&node.right<0){
            // leaf
            for(int i=node.triStart;i<node.triStart+node.triCount;++i){
                int ti=triOrder[i];
                const GPUTri& gt=tris[ti];
                // intersect inline
                Vec3 e1=gt.v1-gt.v0, e2=gt.v2-gt.v0;
                Vec3 pv=cross(ray.dir,e2);
                float det=dot(e1,pv);
                if(fabsf(det)<1e-8f)continue;
                float inv=1.f/det;
                Vec3 tv=ray.orig-gt.v0;
                float bu=dot(tv,pv)*inv;
                if(bu<0||bu>1)continue;
                Vec3 qv=cross(tv,e1);
                float bv=dot(ray.dir,qv)*inv;
                if(bv<0||bu+bv>1)continue;
                float t=dot(e2,qv)*inv;
                if(t<tmin||t>=best.t)continue;
                float bw=1.f-bu-bv;
                best.hit=true; best.t=t;
                best.p=ray.at(t);
                best.matId=gt.matId;
                // Normal
                Vec3 N=normalize(gt.n0*bw+gt.n1*bu+gt.n2*bv);
                Vec3 gN=normalize(cross(e1,e2));
                best.front_face=dot(ray.dir,gN)<0;
                if(!best.front_face){N=-1.f*N;gN=-1.f*gN;}
                if(dot(N,gN)<0)N=-1.f*N;
                best.normal=N;
                best.u=gt.u0*bw+gt.u1*bu+gt.u2*bv;
                best.v=gt.v0_*bw+gt.v1_*bu+gt.v2_*bv;
                // Tangent
                float du1=gt.u1-gt.u0,dv1=gt.v1_-gt.v0_;
                float du2=gt.u2-gt.u0,dv2=gt.v2_-gt.v0_;
                float dd=du1*dv2-du2*dv1;
                Vec3 T;
                if(fabsf(dd)>1e-8f){T=normalize((e1*dv2-e2*dv1)*(1.f/dd));}
                else{Vec3 up=fabsf(N.z)<0.9f?make_vec3(0,0,1):make_vec3(1,0,0);T=normalize(cross(up,N));}
                T=normalize(T-N*dot(N,T));
                best.tangent=T; best.bitangent=cross(N,T);
            }
        } else {
            if(node.left >=0&&sp<64)stack[sp++]=node.left;
            if(node.right>=0&&sp<64)stack[sp++]=node.right;
        }
    }
    return best;
}

// ---- Shadow test ----
HYBRID inline bool inShadow(
    Vec3 P, Vec3 N, Vec3 lightPos, float lightRadius,
    const BVHNode* nodes, const int* triOrder, const GPUTri* tris,
    int numTris, unsigned int& rng_st)
{
    // Soft shadow: jitter light position within disk
    Vec3 toL=lightPos-P;
    if(lightRadius>0){
        Vec3 disk=randDisk(rng_st);
        Vec3 right=normalize(cross(toL,make_vec3(0,1,0)));
        Vec3 up=cross(normalize(toL),right);
        toL=toL+(right*disk.x+up*disk.y)*lightRadius;
    }
    float dist=length(toL);
    if(dist<1e-6f)return false;
    Ray sr(P+N*1e-3f, toL*(1.f/dist));
    HitRecord sh=traverseBVH(sr,nodes,triOrder,tris,numTris,1e-3f);
    return sh.hit&&sh.t<dist-1e-3f;
}

// ---- BRDF ----
HYBRID inline Vec3 evalBRDF(const HitRecord& rec, const Material* mats,
                             Vec3 V, Vec3 L, Vec3 Ns)
{
    const Material& m=mats[rec.matId];
    Vec3 albedo=m.albedo;
    if(m.albedo_map.valid()) albedo=albedo*m.albedo_map.sample(rec.u,rec.v);

    float NdotL=fmaxf(dot(Ns,L),0.f);
    float NdotV=fmaxf(dot(Ns,V),0.f);
    if(NdotL<=0||NdotV<=0)return make_vec3(0,0,0);

    // Lambertian
    Vec3 fd=albedo*(m.kd*0.31830988f);

    // Blinn-Phong specular
    Vec3 H=normalize(L+V);
    float NdotH=fmaxf(dot(Ns,H),0.f);
    float spec=(m.shininess+2.f)*0.15915494f*powf(NdotH,m.shininess);
    Vec3 fs=m.specularColor*(m.ks*spec);

    return fd+fs;
}

// ---- Bump normal ----
HYBRID inline Vec3 bumpNormal(const HitRecord& rec, const Texture& bmap, float str=100.f){
    float H =bmap.sampleGray(rec.u,rec.v);
    float Hx=bmap.sampleGray(rec.u+0.01f,rec.v);
    float Hy=bmap.sampleGray(rec.u,rec.v+0.01f);
    Vec3 bump=normalize(make_vec3(-(Hx-H)*str,(Hy-H)*str,1.f));
    return normalize(rec.tangent*bump.x+rec.bitangent*bump.y+rec.normal*bump.z);
}

// ---- Main path tracer with Russian Roulette ----
HYBRID inline Vec3 trace(
    Ray ray, int maxDepth, Vec3 missColor,
    const BVHNode* nodes, const int* triOrder, const GPUTri* tris, int numTris,
    const Material* mats,
    const Light* lights, int numLights,
    unsigned int rng_st, bool diffuseBounce)
{
    Vec3 radiance={0,0,0};
    Vec3 throughput={1,1,1};

    for(int depth=0;depth<maxDepth;++depth){

        // Russian Roulette after depth 3
        if(depth>=3){
            float p=fmaxf3(throughput.x,throughput.y,throughput.z);
            p=fmaxf(0.05f,fminf(p,0.95f));
            if(rng(rng_st)>p)break;
            throughput=throughput*(1.f/p);
        }

        HitRecord rec=traverseBVH(ray,nodes,triOrder,tris,numTris);
        if(!rec.hit){radiance=radiance+throughput*missColor;break;}

        const Material& m=mats[rec.matId];

        // Emission
        radiance=radiance+throughput*m.emission;

        // Bump normal
        Vec3 N=rec.normal;
        if(m.bump_map.valid()) N=bumpNormal(rec,m.bump_map);

        Vec3 V=normalize(ray.orig-rec.p);

        // Direct lighting (soft shadows)
        Vec3 direct={0,0,0};
        for(int li=0;li<numLights;++li){
            const Light& l=lights[li];
            Vec3 toL=l.pos-rec.p;
            float dist=length(toL);
            Vec3 Ldir=toL*(1.f/dist);
            float NdotL=fmaxf(dot(N,Ldir),0.f);
            if(NdotL<=0)continue;
            if(!inShadow(rec.p,N,l.pos,l.radius,nodes,triOrder,tris,numTris,rng_st)){
                Vec3 f=evalBRDF(rec,mats,V,Ldir,N);
                direct=direct+l.color*(l.intensity*NdotL)*f;
            }
        }
        // Ambient
        Vec3 albedo=m.albedo;
        if(m.albedo_map.valid())albedo=albedo*m.albedo_map.sample(rec.u,rec.v);
        direct=direct+albedo*0.05f;
        radiance=radiance+throughput*direct;

        // Bounce
        float kd=m.kd, kr=m.kr, total=kd+kr;
        if(total<=0)break;
        float xi=rng(rng_st);
        if(diffuseBounce&&xi<kd/total){
            Vec3 d=randHemi(N,rng_st);
            ray=Ray(rec.p+N*1e-3f,d);
            float NdotD=fmaxf(dot(N,d),0.f);
            throughput=throughput*(albedo*(2.f*NdotD));
        } else {
            Vec3 refl=reflect(normalize(ray.dir),N);
            ray=Ray(rec.p+N*1e-3f,refl);
            throughput=throughput*(kr*m.specularColor);
        }
        if(throughput.x<1e-6f&&throughput.y<1e-6f&&throughput.z<1e-6f)break;
    }
    return clamp3(radiance);
}
