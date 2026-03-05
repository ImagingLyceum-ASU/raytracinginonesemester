#include "vec3.h"
#include "ray.h"
#include "texture.h"
#include "material.h"
#include "mesh.h"
#include "bvh.h"
#include "camera.h"
#include "scene.h"
#include "trace.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <vector>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstring>

// ----------------------------------------------------------------
// Texture upload helpers
// ----------------------------------------------------------------
static std::vector<unsigned char*> g_hostTexData; // track CPU allocs

static Texture loadTextureCPU(const std::string& path){
    Texture t{};
    if(path.empty())return t;
    int w,h,ch;
    unsigned char* px=stbi_load(path.c_str(),&w,&h,&ch,0);
    if(!px){fprintf(stderr,"Cannot load texture: %s\n",path.c_str());return t;}
    size_t bytes=(size_t)w*h*ch;
    unsigned char* buf=new unsigned char[bytes];
    memcpy(buf,px,bytes);
    stbi_image_free(px);
    g_hostTexData.push_back(buf);
    t.data=buf; t.w=w; t.h=h; t.ch=ch;
    return t;
}

#ifdef __CUDACC__
static std::vector<unsigned char*> g_devTexPtrs;
static Texture uploadTexture(const Texture& cpu){
    if(!cpu.valid())return Texture{};
    size_t bytes=(size_t)cpu.w*cpu.h*cpu.ch;
    unsigned char* d=nullptr;
    cudaMalloc(&d,bytes);
    cudaMemcpy(d,cpu.data,bytes,cudaMemcpyHostToDevice);
    g_devTexPtrs.push_back(d);
    Texture t=cpu; t.data=d; return t;
}
#endif

// ----------------------------------------------------------------
// GPU kernel
// ----------------------------------------------------------------
#ifdef __CUDACC__
__global__ void renderKernel(
    int W, int H, int spp, int maxDepth, Vec3 missColor,
    Camera cam,
    const BVHNode* nodes, const int* triOrder, const GPUTri* tris, int numTris,
    const Material* mats,
    const Light* lights, int numLights,
    bool diffuseBounce,
    Vec3* out)
{
    int x=blockIdx.x*blockDim.x+threadIdx.x;
    int y=blockIdx.y*blockDim.y+threadIdx.y;
    if(x>=W||y>=H)return;
    Vec3 accum={0,0,0};
    for(int s=0;s<spp;++s){
        unsigned int rng_st=rngSeed(x,y,s);
        float u=(x+rng(rng_st))/(float)W;
        float v=(y+rng(rng_st))/(float)H;
        Ray ray=cam.get_ray(u,v,rng_st);
        accum=accum+trace(ray,maxDepth,missColor,nodes,triOrder,tris,numTris,
                          mats,lights,numLights,rng_st,diffuseBounce);
    }
    out[y*W+x]=accum*(1.f/spp);
}
#endif

// ----------------------------------------------------------------
// Build scene
// ----------------------------------------------------------------
static SceneDesc buildScene(const std::string& assetsDir){
    SceneDesc s;
    s.spp=128; s.maxDepth=8; s.diffuseBounce=true;
    s.missColor={0.5f,0.7f,1.f};

    // Camera
    s.cam=Camera::make(
        {0.f,-4.f,1.5f}, {0.f,0.f,0.5f}, {0.f,0.f,1.f},
        45.f, 1280.f/720.f, 0.0f, 1.f, 1280, 720);

    // Lights (area lights with radius for soft shadows)
    Light l1; l1.pos={-2.f,-1.f,3.f}; l1.color={1,1,1}; l1.intensity=5.f; l1.radius=0.5f;
    Light l2; l2.pos={ 2.f,-1.f,3.f}; l2.color={0.8f,0.9f,1.f}; l2.intensity=3.f; l2.radius=0.3f;
    s.lights={l1,l2};

    // Materials
    Material ground; ground.albedo={0.6f,0.55f,0.5f}; ground.kd=1.f;
    ground.bump_map=loadTextureCPU(assetsDir+"/textures/spec_bump.png");

    Material red;   red.albedo={0.8f,0.2f,0.2f}; red.kd=1.f; red.ks=0.3f; red.shininess=32;
    Material green; green.albedo={0.2f,0.7f,0.2f}; green.kd=1.f;
    Material earth; earth.albedo={1,1,1}; earth.kd=1.f;
    earth.albedo_map=loadTextureCPU(assetsDir+"/textures/earth.png");
    Material stone; stone.albedo={0.7f,0.7f,0.65f}; stone.kd=0.8f; stone.ks=0.2f; stone.shininess=16;
    stone.albedo_map=loadTextureCPU(assetsDir+"/textures/stone.png");
    stone.bump_map =loadTextureCPU(assetsDir+"/textures/stone_bump.png");
    Material mirror; mirror.albedo={0.9f,0.9f,0.9f}; mirror.kd=0.1f; mirror.kr=0.9f;
    mirror.specularColor={0.9f,0.9f,0.9f};

    s.materials={ground,red,green,earth,stone,mirror};
    // IDs: 0=ground 1=red 2=green 3=earth 4=stone 5=mirror

    // Geometry — load spheres as meshes
    std::string meshDir=assetsDir+"/meshes";
    loadOBJ(meshDir+"/plane_5x5.obj",  s.tris, 0, {0,0,0}, 1.f);
    // Back row: earth, stone, mirror
    loadOBJ(meshDir+"/unit_sphere.obj",s.tris, 3, {-1.5f, 1.5f, 0.5f}, 0.5f);
    loadOBJ(meshDir+"/unit_sphere.obj",s.tris, 4, { 0.0f, 1.5f, 0.5f}, 0.5f);
    loadOBJ(meshDir+"/unit_sphere.obj",s.tris, 5, { 1.5f, 1.5f, 0.5f}, 0.5f);
    // Front row: small red and green
    loadOBJ(meshDir+"/unit_sphere.obj",s.tris, 1, {-1.5f,-0.5f, 0.25f}, 0.25f);
    loadOBJ(meshDir+"/unit_sphere.obj",s.tris, 2, { 0.0f,-0.5f, 0.25f}, 0.25f);

    return s;
}

int main(int argc, char** argv){
    std::string assetsDir="../assets";
    if(argc>=2) assetsDir=argv[1];

    printf("Building scene...\n");
    SceneDesc scene=buildScene(assetsDir);
    int W=scene.cam.pw, H=scene.cam.ph;
    printf("Scene: %d tris, %d materials, %d lights\n",
           (int)scene.tris.size(),(int)scene.materials.size(),(int)scene.lights.size());

    // Build BVH on CPU
    BVHTree bvhTree;
    bvhTree.build(scene.tris);
    int numNodes=(int)bvhTree.nodes.size();
    int numTris =(int)scene.tris.size();
    printf("BVH: %d nodes\n",numNodes);

    // Build flat GPUTri array
    std::vector<GPUTri> gpuTris(numTris);
    for(int i=0;i<numTris;++i) gpuTris[i]=toGPUTri(scene.tris[i]);

    std::vector<Vec3> image(W*H,{0,0,0});

#ifdef __CUDACC__
    // Upload textures for all materials
    std::vector<Material> gpuMats=scene.materials;
    for(auto& m:gpuMats){
        m.albedo_map=uploadTexture(m.albedo_map);
        m.bump_map  =uploadTexture(m.bump_map);
    }

    // Upload scene data
    BVHNode* d_nodes; cudaMalloc(&d_nodes,numNodes*sizeof(BVHNode));
    cudaMemcpy(d_nodes,bvhTree.nodes.data(),numNodes*sizeof(BVHNode),cudaMemcpyHostToDevice);

    int* d_triOrder; cudaMalloc(&d_triOrder,numTris*sizeof(int));
    cudaMemcpy(d_triOrder,bvhTree.triOrder.data(),numTris*sizeof(int),cudaMemcpyHostToDevice);

    GPUTri* d_tris; cudaMalloc(&d_tris,numTris*sizeof(GPUTri));
    cudaMemcpy(d_tris,gpuTris.data(),numTris*sizeof(GPUTri),cudaMemcpyHostToDevice);

    Material* d_mats; cudaMalloc(&d_mats,gpuMats.size()*sizeof(Material));
    cudaMemcpy(d_mats,gpuMats.data(),gpuMats.size()*sizeof(Material),cudaMemcpyHostToDevice);

    Light* d_lights; cudaMalloc(&d_lights,scene.lights.size()*sizeof(Light));
    cudaMemcpy(d_lights,scene.lights.data(),scene.lights.size()*sizeof(Light),cudaMemcpyHostToDevice);

    Vec3* d_out; cudaMalloc(&d_out,W*H*sizeof(Vec3));
    cudaMemset(d_out,0,W*H*sizeof(Vec3));

    dim3 block(16,16);
    dim3 grid((W+15)/16,(H+15)/16);

    printf("Rendering %dx%d @ %dspp on GPU...\n",W,H,scene.spp);
    auto t0=std::chrono::high_resolution_clock::now();

    renderKernel<<<grid,block>>>(
        W,H,scene.spp,scene.maxDepth,scene.missColor,scene.cam,
        d_nodes,d_triOrder,d_tris,numTris,d_mats,
        d_lights,(int)scene.lights.size(),scene.diffuseBounce,d_out);
    cudaDeviceSynchronize();

    auto t1=std::chrono::high_resolution_clock::now();
    printf("GPU Render: %.1f ms\n",std::chrono::duration<double,std::milli>(t1-t0).count());

    cudaMemcpy(image.data(),d_out,W*H*sizeof(Vec3),cudaMemcpyDeviceToHost);

    // Cleanup
    for(auto* p:g_devTexPtrs) cudaFree(p);
    cudaFree(d_nodes);cudaFree(d_triOrder);cudaFree(d_tris);
    cudaFree(d_mats);cudaFree(d_lights);cudaFree(d_out);

#else
    printf("Rendering %dx%d @ %dspp on CPU...\n",W,H,scene.spp);
    auto t0=std::chrono::high_resolution_clock::now();
    int numMats=(int)scene.materials.size();
    int numLights=(int)scene.lights.size();
    for(int y=0;y<H;++y){
        for(int x=0;x<W;++x){
            Vec3 accum={0,0,0};
            for(int s=0;s<scene.spp;++s){
                unsigned int rng_st=rngSeed(x,y,s);
                float u=(x+rng(rng_st))/(float)W;
                float v=(y+rng(rng_st))/(float)H;
                Ray ray=scene.cam.get_ray(u,v,rng_st);
                accum=accum+trace(ray,scene.maxDepth,scene.missColor,
                    bvhTree.nodes.data(),bvhTree.triOrder.data(),gpuTris.data(),numTris,
                    scene.materials.data(),scene.lights.data(),numLights,
                    rng_st,scene.diffuseBounce);
            }
            image[y*W+x]=accum*(1.f/scene.spp);
        }
        if(y%50==0)printf("  row %d/%d\n",y,H);
    }
    auto t1=std::chrono::high_resolution_clock::now();
    printf("CPU Render: %.1f ms\n",std::chrono::duration<double,std::milli>(t1-t0).count());
#endif

    // Write PNG — flip Y so image is right-side up
    std::vector<unsigned char> img(W*H*3);
    for(int y=0;y<H;++y){
        for(int x=0;x<W;++x){
            int src=(H-1-y)*W+x; // flip
            int dst=y*W+x;
            img[dst*3+0]=(unsigned char)(255.f*fminf(image[src].x,1.f));
            img[dst*3+1]=(unsigned char)(255.f*fminf(image[src].y,1.f));
            img[dst*3+2]=(unsigned char)(255.f*fminf(image[src].z,1.f));
        }
    }
    stbi_write_png("render.png",W,H,3,img.data(),W*3);
    printf("Saved render.png\n");

    for(auto* p:g_hostTexData) delete[] p;
    return 0;
}
