#pragma once
#include <string>
namespace inverse::shaders {
inline const std::string common = R"GLSL(#version 430 core
struct State {vec4 parameters;vec4 statistics;vec4 proposal;vec4 diagnostics;vec4 quality;};
layout(std430,binding=0) buffer Control {State state;};
layout(std430,binding=1) buffer Trajectories {vec4 trajectories[];};
layout(std430,binding=2) buffer Observations {vec4 observations[];};
layout(std430,binding=3) buffer Reference {vec4 reference[];};
layout(std430,binding=4) buffer Equations {vec4 equations[];};
layout(std430,binding=5) buffer Candidates {vec4 candidates[];};
layout(std430,binding=6) buffer History {vec4 history[];};
struct Vertex {vec4 p;vec4 color;};
layout(std430,binding=7) buffer Geometry {Vertex vertices[];};
uniform int samples,substeps,mode,scene,stiffnessOnly,hasReference,jobs,playback,selectedNode,showTrails,showReference;
uniform float noise,fdStep;
uniform vec3 truth;
int size(){return samples*8;}
vec3 rest(int i){return vec3((i&1)!=0?.25:-.25,(i&2)!=0?1.:.5,(i&4)!=0?.25:-.25);}
vec3 initialPosition(int i){vec3 p=rest(i);if(i>=6)return p;float x=p.x,z=p.z;p.x=.9393727*x+.3428978*z+.10;p.z=-.3428978*x+.9393727*z-.06;return p;}
vec3 initialVelocity(int i){vec3 p=rest(i);return i>=6?vec3(0):vec3(.4*(.5-p.z),.1*p.x,.35+.1*float(i)/8.);}
float springWeight(int a,int b){int bits=bitCount(a^b);return bits==1?1.:(bits==2?.7:.45);}
vec3 bounded(vec3 q){return clamp(q,log(vec3(4.,.03,.3)),log(vec3(50.,3.,12.)));}
bool bad(vec3 p){return any(isnan(p))||any(isinf(p));}
uint hash(uint x){x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;return x^(x>>16);}
float random(uint id){return float(hash(id)>>8)/16777216.;}
)GLSL";
inline const std::string initialize = R"GLSL(
layout(local_size_x=128) in;
void main(){uint id=gl_GlobalInvocationID.x;if(id<256u)history[id]=vec4(0);
    if(id==0u){state.parameters=vec4(log(vec3(10.,1.2,8.)),.01);if(stiffnessOnly!=0)state.parameters.yz=log(truth.yz);
        state.statistics=vec4(0);state.proposal=vec4(0);state.diagnostics=vec4(0);state.quality=vec4(0);}}
)GLSL";
inline const std::string prepare = R"GLSL(
layout(local_size_x=128) in;
void main(){int job=int(gl_GlobalInvocationID.x);if(job>=jobs)return;vec3 q=state.parameters.xyz;
    if(mode==1&&job>0)q[(job-1)/2]+=job%2==1?-fdStep:fdStep;
    if(mode==2)q+=state.proposal.xyz*exp2(-float(job));
    if(mode==3&&job>0){int j=job-1;q=log(vec3(10.+3.*float(j%8),.3+.8*float((j/8)%2),4.+3.*float(j/16)));}
    if(stiffnessOnly!=0&&mode!=1)q.yz=state.parameters.yz;
    candidates[job]=vec4(mode==1?q:bounded(q),1e30);
}
)GLSL";
inline const std::string forward = R"GLSL(
layout(local_size_x=8) in;
shared vec3 position[8],velocity[8];
vec3 acceleration(int i,vec3 parameters){if(i>=6)return vec3(0);vec3 a=vec3(0,-parameters.z,0)-parameters.y*velocity[i];
    for(int j=0;j<8;++j)if(j!=i){vec3 d=position[j]-position[i];float lengthD=length(d),restLength=length(rest(j)-rest(i));
        a+=parameters.x*springWeight(i,j)*(lengthD-restLength)*d/max(lengthD,1e-6);}
    return a;
}
void main(){int i=int(gl_LocalInvocationID.x),job=int(gl_WorkGroupID.x);vec3 physical=mode==0?truth:exp(candidates[job].xyz);
    position[i]=initialPosition(i);velocity[i]=initialVelocity(i);barrier();float h=1./(60.*float(substeps));
    for(int s=0;s<samples;++s){
        if(s>0)for(int stepIndex=0;stepIndex<substeps;++stepIndex){
            vec3 p=position[i],v=velocity[i],p1=v,v1=acceleration(i,physical);barrier();
            position[i]=p+.5*h*p1;velocity[i]=v+.5*h*v1;barrier();
            vec3 p2=velocity[i],v2=acceleration(i,physical);barrier();
            position[i]=p+.5*h*p2;velocity[i]=v+.5*h*v2;barrier();
            vec3 p3=velocity[i],v3=acceleration(i,physical);barrier();
            position[i]=p+h*p3;velocity[i]=v+h*v3;barrier();
            vec3 p4=velocity[i],v4=acceleration(i,physical);barrier();
            position[i]=p+h*(p1+2.*p2+2.*p3+p4)/6.;velocity[i]=v+h*(v1+2.*v2+2.*v3+v4)/6.;barrier();
        }
        if(mode==0)reference[s*8+i]=vec4(position[i],1.);
        else trajectories[job*size()+s*8+i]=vec4(position[i],1.);
        barrier();
    }
}
)GLSL";
inline const std::string observe = R"GLSL(
layout(local_size_x=128) in;
void main(){int i=int(gl_GlobalInvocationID.x);if(i>=size())return;int node=i%8,s=i/8;vec3 p=reference[i].xyz;float weight=node<6?1.:0.;
    if(scene==2)weight=weight*((s%4==0&&(node==0||node==2||node==5))?1.:0.);
    if(scene==1||scene==2)p+=noise*(vec3(random(uint(i)*3u+333u),random(uint(i)*3u+334u),random(uint(i)*3u+335u))*2.-1.);
    observations[i]=vec4(p,weight);
}
)GLSL";
inline const std::string cost = R"GLSL(
layout(local_size_x=128) in;
shared vec2 sums[128];
void main(){uint lane=gl_LocalInvocationID.x;int job=int(gl_WorkGroupID.x);vec2 total=vec2(0);
    for(int i=int(lane);i<size();i+=128){vec3 r=trajectories[job*size()+i].xyz-observations[i].xyz;float w=observations[i].w;
        total+=vec2(w*dot(r,r),w);if(bad(r))total.x=1e30;}
    sums[lane]=total;barrier();for(uint stride=64u;stride>0u;stride>>=1u){if(lane<stride)sums[lane]+=sums[lane+stride];barrier();}
    if(lane==0u)candidates[job].w=sums[0].y>0.?sums[0].x/(3.*sums[0].y):1e30;
}
)GLSL";
inline const std::string normals = R"GLSL(
layout(local_size_x=128) in;
shared vec4 a[128],b[128],c[128];
void main(){uint lane=gl_LocalInvocationID.x;int i=int(gl_GlobalInvocationID.x);a[lane]=vec4(0);b[lane]=vec4(0);c[lane]=vec4(0);
    if(i<size()){vec3 r=trajectories[i].xyz-observations[i].xyz;float w=observations[i].w;
        vec3 j0=(trajectories[2*size()+i].xyz-trajectories[size()+i].xyz)/(2.*fdStep);
        vec3 j1=(trajectories[4*size()+i].xyz-trajectories[3*size()+i].xyz)/(2.*fdStep);
        vec3 j2=(trajectories[6*size()+i].xyz-trajectories[5*size()+i].xyz)/(2.*fdStep);
        a[lane]=w*vec4(dot(j0,j0),dot(j0,j1),dot(j0,j2),dot(j1,j1));
        b[lane]=w*vec4(dot(j1,j2),dot(j2,j2),dot(j0,r),dot(j1,r));c[lane]=vec4(w*dot(j2,r),w*dot(r,r),w,0);
        if(bad(r)||bad(j0)||bad(j1)||bad(j2))c[lane].w=1.;}
    barrier();for(uint stride=64u;stride>0u;stride>>=1u){if(lane<stride){a[lane]+=a[lane+stride];b[lane]+=b[lane+stride];c[lane]+=c[lane+stride];}barrier();}
    if(lane==0u){int o=int(gl_WorkGroupID.x)*3;equations[o]=a[0];equations[o+1]=b[0];equations[o+2]=c[0];}
}
)GLSL";
inline const std::string solve = R"GLSL(
layout(local_size_x=1) in;
void main(){vec4 a=vec4(0),b=vec4(0),c=vec4(0);for(int group=0;group<(size()+127)/128;++group){a+=equations[group*3];b+=equations[group*3+1];c+=equations[group*3+2];}
    state.statistics.xy=vec2(c.y/max(3.*c.z,1e-20),c.z);
    if(c.z<=0.||c.w>0.||a.x<1e-9){state.diagnostics.z=4.;state.proposal=vec4(0);return;}
    mat3 gram=mat3(a.x,a.y,a.z,a.y,a.w,b.x,a.z,b.x,b.y);float normalizedDet=determinant(gram)/max(a.x*a.w*b.y,1e-30);
    state.diagnostics.x=stiffnessOnly!=0?1.:normalizedDet;
    if(stiffnessOnly==0&&normalizedDet<1e-7){state.diagnostics.z=4.;state.proposal=vec4(0);return;}
    float matrix[12];matrix[0]=a.x;matrix[1]=a.y;matrix[2]=a.z;matrix[3]=-b.z;
    matrix[4]=a.y;matrix[5]=a.w;matrix[6]=b.x;matrix[7]=-b.w;matrix[8]=a.z;matrix[9]=b.x;matrix[10]=b.y;matrix[11]=-c.x;
    if(stiffnessOnly!=0){matrix[1]=0.;matrix[2]=0.;matrix[4]=0.;matrix[5]=1.;matrix[6]=0.;matrix[7]=0.;matrix[8]=0.;matrix[9]=0.;matrix[10]=1.;matrix[11]=0.;}
    for(int row=0;row<3;++row)matrix[row*4+row]+=state.parameters.w*max(matrix[row*4+row],1e-6)+1e-9;
    // Small pivoted solve runs on the GPU; all expensive trajectories run in parallel workgroups.
    for(int col=0;col<3;++col){int pivot=col;for(int row=col+1;row<3;++row)if(abs(matrix[row*4+col])>abs(matrix[pivot*4+col]))pivot=row;
        if(abs(matrix[pivot*4+col])<1e-15){state.diagnostics.z=4.;return;}
        for(int j=0;j<4;++j){float temp=matrix[col*4+j];matrix[col*4+j]=matrix[pivot*4+j];matrix[pivot*4+j]=temp;}
        float divisor=matrix[col*4+col];for(int j=col;j<4;++j)matrix[col*4+j]/=divisor;
        for(int row=0;row<3;++row)if(row!=col){float factor=matrix[row*4+col];for(int j=col;j<4;++j)matrix[row*4+j]-=factor*matrix[col*4+j];}}
    vec3 step=vec3(matrix[3],matrix[7],matrix[11]);float lengthStep=length(step);if(lengthStep>.7)step*=.7/lengthStep;
    state.proposal=vec4(step,0);state.diagnostics.y=length(step);state.diagnostics.z=1.;
}
)GLSL";
inline const std::string choose = R"GLSL(
layout(local_size_x=1) in;
void main(){if(state.diagnostics.z==4.)return;float old=state.statistics.x;int best=-1;float loss=old;
    for(int i=0;i<jobs;++i)if(candidates[i].w<loss){best=i;loss=candidates[i].w;}
    if(mode==3){if(best>=0){state.parameters.xyz=candidates[best].xyz;state.statistics.x=loss;}state.diagnostics.w=1.;return;}
    bool accepted=best>=0&&loss<old-max(1e-11,old*1e-6);
    if(accepted){state.parameters.xyz=candidates[best].xyz;state.statistics.x=loss;state.parameters.w=max(1e-7,state.parameters.w*.4);state.statistics.w+=1.;state.proposal.w=exp2(-float(best));}
    else {state.parameters.w=min(1e6,state.parameters.w*10.);state.proposal.w=0.;}
    state.statistics.z+=1.;
    if(state.statistics.x<1e-10||state.diagnostics.y<1e-4)state.diagnostics.z=2.;
    else if(state.statistics.z>=100.||state.parameters.w>=1e6)state.diagnostics.z=3.;
    int iteration=int(state.statistics.z);history[min(iteration,255)]=vec4(state.statistics.x,state.parameters.w,accepted?1.:0.,1.);
}
)GLSL";
inline const std::string diagnostics = R"GLSL(
layout(local_size_x=128) in;
shared vec4 sums[128];shared float peaks[128];
void main(){uint lane=gl_LocalInvocationID.x;vec4 total=vec4(0);float peak=0.;
    for(int i=int(lane);i<size();i+=128){vec3 p=trajectories[i].xyz,r=p-observations[i].xyz;float w=observations[i].w;
        total.xy+=vec2(w*dot(r,r),w);if(i%8<6&&hasReference!=0)total.z+=dot(p-reference[i].xyz,p-reference[i].xyz);
        if(bad(p))total.w+=1.;if(w>0.)peak=max(peak,length(r));}
    sums[lane]=total;peaks[lane]=peak;barrier();for(uint stride=64u;stride>0u;stride>>=1u){if(lane<stride){sums[lane]+=sums[lane+stride];peaks[lane]=max(peaks[lane],peaks[lane+stride]);}barrier();}
    if(lane==0u){float mse=sums[0].x/max(3.*sums[0].y,1e-20);state.statistics.xy=vec2(mse,sums[0].y);
        state.quality=vec4(sqrt(mse),hasReference!=0?sqrt(sums[0].z/float(samples*18)):-1.,peaks[0],sums[0].w);
        if(sums[0].w>0.||sums[0].y<=0.)state.diagnostics.z=4.;if(state.statistics.z==0.)history[0]=vec4(mse,state.parameters.w,0,1);}
}
)GLSL";
inline const std::string pack = R"GLSL(
layout(local_size_x=128) in;
void main(){int i=int(gl_GlobalInvocationID.x),trajectoryVertices=(samples-1)*8*2;
    if(i<trajectoryVertices){int sampleIndex=i/16,node=(i/2)%8,endpoint=i%2;vec3 p=trajectories[(sampleIndex+endpoint)*8+node].xyz;
        float alpha=showTrails!=0&&(selectedNode==8||selectedNode==node)?.32:0.;vertices[i].p=vec4(p,1);vertices[i].color=vec4(1.,.54,.18,alpha);return;}
    int j=i-trajectoryVertices;
    if(j<112){int which=j/56,edge=(j%56)/2,endpoint=j%2,a=0,b=1;int k=0;
        for(int x=0;x<8;++x)for(int y=x+1;y<8;++y){if(k==edge){a=x;b=y;}++k;}
        int node=endpoint==0?a:b;vec3 p=which==0?trajectories[playback*8+node].xyz:reference[playback*8+node].xyz;
        vertices[i].p=vec4(p,1);vertices[i].color=which==0?vec4(1.,.58,.18,.85):vec4(.15,.85,1.,hasReference!=0&&showReference!=0?.38:0.);return;}
    j-=112;
    if(j<16){int node=j/2;vec4 observation=observations[playback*8+node];vertices[i].p=vec4(j%2==0?observation.xyz:trajectories[playback*8+node].xyz,1);vertices[i].color=vec4(1.,.2,.65,observation.w>0.?.9:0.);return;}
    j-=16;
    if(j<510){int index=j/2+j%2;float horizontal=max(10.,ceil(state.statistics.z/10.)*10.);float value=clamp((log(max(history[index].x,1e-12))/log(10.)+10.)/10.,0.,1.);
        vertices[i].p=vec4(-.92+1.84*float(index)/horizontal,-.75+1.5*value,0,1);vertices[i].color=vec4(.85,.65,.2,j/2+1<=int(state.statistics.z)?1.:0.);}
}
)GLSL";
inline const std::string lineVertex = R"GLSL(#version 430 core
layout(location=0) in vec4 position;layout(location=1) in vec4 vertexColor;
uniform mat4 view,projection;uniform int screenSpace;out vec4 tint;
void main(){gl_Position=screenSpace!=0?position:projection*view*position;tint=vertexColor;}
)GLSL";
inline const std::string lineFragment = R"GLSL(#version 430 core
in vec4 tint;out vec4 color;void main(){color=tint;}
)GLSL";
inline const std::string pointVertex = R"GLSL(#version 430 core
layout(location=0) in vec4 position;uniform mat4 view,projection;uniform int selectedNode,pointMode;out vec4 tint;
void main(){gl_Position=projection*view*vec4(position.xyz,1);gl_PointSize=pointMode==0?3.:10.;
    tint=pointMode==0?vec4(.18,.85,1.,.7):vec4(1.,.65,.22,1.);
    if(pointMode==0&&(position.w<=0.||(selectedNode!=8&&gl_VertexID%8!=selectedNode)))gl_Position=vec4(2,2,2,1);}
)GLSL";
inline const std::string pointFragment = R"GLSL(#version 430 core
in vec4 tint;out vec4 color;void main(){vec2 d=gl_PointCoord*2.-1.;float r=dot(d,d);if(r>1.)discard;color=vec4(tint.rgb,tint.a*(1.-.5*r));}
)GLSL";
} // namespace inverse::shaders
