#pragma once
#include "vec3.h"

// GPU-safe flat texture — device pointer, no STL
struct Texture {
    unsigned char* data = nullptr;
    int w=0, h=0, ch=0;

    HYBRID bool valid()const{return data&&w>0&&h>0;}

    HYBRID Vec3 sample(float u,float v)const{
        if(!valid())return make_vec3(1,1,1);
        u=u-floorf(u); v=v-floorf(v);
        int x=(int)(u*(w-1)); int y=(int)((1.f-v)*(h-1));
        x=x<0?0:(x>=w?w-1:x); y=y<0?0:(y>=h?h-1:y);
        const unsigned char* p=data+(y*w+x)*ch;
        if(ch>=3)return make_vec3(p[0]/255.f,p[1]/255.f,p[2]/255.f);
        float g=p[0]/255.f; return make_vec3(g,g,g);
    }
    HYBRID float sampleGray(float u,float v)const{
        Vec3 c=sample(u,v); return 0.299f*c.x+0.587f*c.y+0.114f*c.z;
    }
};
