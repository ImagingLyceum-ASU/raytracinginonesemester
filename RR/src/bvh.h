#pragma once
#include "vec3.h"
#include "ray.h"
#include "mesh.h"
#include <vector>
#include <algorithm>
#include <cfloat>

struct AABB {
    Vec3 mn={FLT_MAX,FLT_MAX,FLT_MAX};
    Vec3 mx={-FLT_MAX,-FLT_MAX,-FLT_MAX};
    HYBRID void expand(Vec3 p){
        mn.x=fminf(mn.x,p.x);mn.y=fminf(mn.y,p.y);mn.z=fminf(mn.z,p.z);
        mx.x=fmaxf(mx.x,p.x);mx.y=fmaxf(mx.y,p.y);mx.z=fmaxf(mx.z,p.z);
    }
    HYBRID static AABB merge(AABB a,AABB b){
        AABB r;
        r.mn.x=fminf(a.mn.x,b.mn.x);r.mn.y=fminf(a.mn.y,b.mn.y);r.mn.z=fminf(a.mn.z,b.mn.z);
        r.mx.x=fmaxf(a.mx.x,b.mx.x);r.mx.y=fmaxf(a.mx.y,b.mx.y);r.mx.z=fmaxf(a.mx.z,b.mx.z);
        return r;
    }
    HYBRID bool hit(const Ray& r,float tmin,float tmax)const{
        for(int i=0;i<3;++i){
            float invD=1.f/(i==0?r.dir.x:i==1?r.dir.y:r.dir.z);
            float o=(i==0?r.orig.x:i==1?r.orig.y:r.orig.z);
            float mn_=(i==0?mn.x:i==1?mn.y:mn.z);
            float mx_=(i==0?mx.x:i==1?mx.y:mx.z);
            float t0=(mn_-o)*invD, t1=(mx_-o)*invD;
            if(invD<0){float tmp=t0;t0=t1;t1=tmp;}
            tmin=fmaxf(tmin,t0); tmax=fminf(tmax,t1);
            if(tmax<=tmin)return false;
        }
        return true;
    }
};

struct BVHNode {
    AABB box;
    int left,right;   // -1 if leaf
    int triStart,triCount; // for leaves
};

struct BVHTree {
    std::vector<BVHNode> nodes;
    std::vector<int>     triOrder; // reordered triangle indices

    void build(const std::vector<Triangle>& tris){
        int n=(int)tris.size();
        triOrder.resize(n);
        for(int i=0;i<n;++i)triOrder[i]=i;
        nodes.clear();
        buildNode(tris,0,n);
    }

private:
    AABB triAABB(const Triangle& t){
        AABB b; b.expand(t.v0.pos);b.expand(t.v1.pos);b.expand(t.v2.pos); return b;
    }
    Vec3 triCentroid(const Triangle& t){
        return (t.v0.pos+t.v1.pos+t.v2.pos)*0.333333f;
    }
    int buildNode(const std::vector<Triangle>& tris,int start,int end){
        BVHNode node;
        node.triStart=start; node.triCount=end-start;
        node.left=node.right=-1;
        AABB box;
        for(int i=start;i<end;++i)box=AABB::merge(box,triAABB(tris[triOrder[i]]));
        node.box=box;
        if(end-start<=4){
            nodes.push_back(node);
            return (int)nodes.size()-1;
        }
        // Split on longest axis
        Vec3 ext={box.mx.x-box.mn.x,box.mx.y-box.mn.y,box.mx.z-box.mn.z};
        int axis=0;
        if(ext.y>ext.x)axis=1;
        if(ext.z>(axis==0?ext.x:ext.y))axis=2;
        float mid=(axis==0?(box.mn.x+box.mx.x):axis==1?(box.mn.y+box.mx.y):(box.mn.z+box.mx.z))*0.5f;
        auto pivot=std::partition(triOrder.begin()+start,triOrder.begin()+end,[&](int i){
            Vec3 c=triCentroid(tris[i]);
            return (axis==0?c.x:axis==1?c.y:c.z)<mid;
        });
        int mid_idx=(int)(pivot-triOrder.begin());
        if(mid_idx==start||mid_idx==end) mid_idx=(start+end)/2;
        nodes.push_back(node); // placeholder
        int nodeIdx=(int)nodes.size()-1;
        int L=buildNode(tris,start,mid_idx);
        int R=buildNode(tris,mid_idx,end);
        nodes[nodeIdx].left=L;
        nodes[nodeIdx].right=R;
        nodes[nodeIdx].triCount=0; // not a leaf
        return nodeIdx;
    }
};
