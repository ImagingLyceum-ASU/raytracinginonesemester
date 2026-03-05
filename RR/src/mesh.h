#pragma once
#include "vec3.h"
#include "material.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdio>

struct Vertex { Vec3 pos, normal; float u,v; };

struct Triangle {
    Vertex v0,v1,v2;
    int matId;
};

// Minimal OBJ loader
inline bool loadOBJ(const std::string& path,
                    std::vector<Triangle>& tris,
                    int matId,
                    Vec3 translate={0,0,0},
                    float scale=1.f,
                    Vec3 rotateDeg={0,0,0})
{
    std::ifstream f(path);
    if(!f){fprintf(stderr,"Cannot open %s\n",path.c_str());return false;}

    auto deg2rad=[](float d){return d*0.01745329f;};
    float rx=deg2rad(rotateDeg.x),ry=deg2rad(rotateDeg.y),rz=deg2rad(rotateDeg.z);

    auto rotXYZ=[&](Vec3 v)->Vec3{
        // X
        float y2=cosf(rx)*v.y-sinf(rx)*v.z, z2=sinf(rx)*v.y+cosf(rx)*v.z; v.y=y2;v.z=z2;
        // Y
        float x2=cosf(ry)*v.x+sinf(ry)*v.z, z3=-sinf(ry)*v.x+cosf(ry)*v.z; v.x=x2;v.z=z3;
        // Z
        float x3=cosf(rz)*v.x-sinf(rz)*v.y, y3=sinf(rz)*v.x+cosf(rz)*v.y; v.x=x3;v.y=y3;
        return v;
    };

    std::vector<Vec3> pos,nrm;
    std::vector<float> uvU,uvV;
    std::string line;
    size_t before=tris.size();

    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        std::istringstream ss(line);
        std::string tok; ss>>tok;
        if(tok=="v"){
            float x,y,z; ss>>x>>y>>z;
            Vec3 p=rotXYZ({x*scale,y*scale,z*scale})+translate;
            pos.push_back(p);
        } else if(tok=="vn"){
            float x,y,z; ss>>x>>y>>z;
            nrm.push_back(rotXYZ({x,y,z}));
        } else if(tok=="vt"){
            float u,v; ss>>u>>v; uvU.push_back(u); uvV.push_back(v);
        } else if(tok=="f"){
            // Parse up to 4 verts (quads split to 2 tris)
            struct Idx{int p,t,n;};
            std::vector<Idx> idxs;
            std::string token;
            while(ss>>token){
                Idx idx={0,-1,-1};
                // formats: v  v/t  v/t/n  v//n
                int a=sscanf(token.c_str(),"%d/%d/%d",&idx.p,&idx.t,&idx.n);
                if(a<3){a=sscanf(token.c_str(),"%d//%d",&idx.p,&idx.n); idx.t=-1;}
                if(a<2){sscanf(token.c_str(),"%d",&idx.p); idx.t=-1;idx.n=-1;}
                idx.p--; if(idx.t>0)idx.t--; if(idx.n>0)idx.n--;
                idxs.push_back(idx);
            }
            auto makeVert=[&](Idx i)->Vertex{
                Vertex vt{};
                vt.pos=(i.p>=0&&i.p<(int)pos.size())?pos[i.p]:make_vec3(0,0,0);
                vt.normal=(i.n>=0&&i.n<(int)nrm.size())?nrm[i.n]:make_vec3(0,0,1);
                vt.u=(i.t>=0&&i.t<(int)uvU.size())?uvU[i.t]:0;
                vt.v=(i.t>=0&&i.t<(int)uvV.size())?uvV[i.t]:0;
                return vt;
            };
            for(int i=1;i+1<(int)idxs.size();++i){
                Triangle tri;
                tri.v0=makeVert(idxs[0]);
                tri.v1=makeVert(idxs[i]);
                tri.v2=makeVert(idxs[i+1]);
                tri.matId=matId;
                tris.push_back(tri);
            }
        }
    }
    printf("Loaded %s -> %zu triangles\n",path.c_str(),tris.size()-before);
    return true;
}
