#include "../include/gpu_pipe.h"
#include "../include/gas_thermo.h"
#include <cuda_runtime.h>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace gpu_pipe {
namespace {
__constant__ double cp[5][2][5];
constexpr double R=8.31446261815324;
__device__ double cv(const double *a,double t) {
    return R*((((a[4]*t+a[3])*t+a[2])*t+a[1])*t+a[0]-1);
}
__device__ double integral(const double *a,double t) {
    return R*t*((((a[4]*t/5+a[3]/4)*t+a[2]/3)*t+a[1]/2)*t+a[0]-1);
}
__device__ double density(const double *u,double fuelMass) {
    return u[0]*fuelMass+u[1]*.0319988+u[2]*.028014+u[3]*.0440098+u[4]*.0180154;
}
__device__ int properties(const double *u,double fuelMass,double &p,double &v,double &speed) {
    double n=0;
    for (int k=0;k<5;++k) {
        if (!isfinite(u[k]) || u[k]<-1e-10) return 2;
        n+=fmax(0.0,u[k]);
    }
    const double rho=density(u,fuelMass);
    if (n==0 && rho==0 && u[5]==0 && u[6]==0) {
        p=v=speed=0;
        return 0;
    }
    if (!(n>0) || !(rho>0)) return 3;
    v=u[5]/rho;
    const double internal=(u[6]-.5*u[5]*v)/n;
    if (!isfinite(internal)) return 4;
    // Port splitting can temporarily exhaust a cell's sensible energy. Match
    // GasSystem::temperature's nonnegative inversion target; keep the conserved
    // energy untouched, and require positive internal energy after the update.
    const double target=fmax(0.0,internal);
    if(target==0) {p=0; speed=fabs(v); return 0;}
    const int order[5]={4,1,0,2,3};
    double a[2][5]={{0}};
    for (int s=0;s<5;++s)
        for (int r=0;r<2;++r)
            for (int k=0;k<5;++k) a[r][k]+=fmax(0.0,u[s])/n*cp[order[s]][r][k];
    const double c200=cv(a[0],200),c6000=cv(a[1],6000);
    const double i200=integral(a[0],200),i1000=integral(a[1],1000);
    const double e200=c200*200;
    const double e1000=e200+integral(a[0],1000)-i200;
    const double e6000=e1000+integral(a[1],6000)-i1000;
    double lo=0,hi=fmax(6000.0,target/R),t=300;
    for (int iteration=0;iteration<32;++iteration) {
        const double energy=t<=200 ? c200*t : t<=1000 ? e200+integral(a[0],t)-i200
            : t<=6000 ? e1000+integral(a[1],t)-i1000 : e6000+c6000*(t-6000);
        const double capacity=t<=200 ? c200 : t>=6000 ? c6000 : cv(a[t<=1000?0:1],t);
        const double residual=energy-target;
        if (fabs(residual)<=1e-10*fmax(1.0,target)) break;
        if (residual>0) hi=t; else lo=t;
        const double next=t-residual/capacity;
        t=next>lo && next<hi ? next : .5*(lo+hi);
    }
    const double capacity=t<=200 ? c200 : t>=6000 ? c6000 : cv(a[t<=1000?0:1],t);
    p=n*R*t;
    speed=fabs(v)+sqrt((1+R/capacity)*p/rho);
    return isfinite(speed) && isfinite(p) ? 0 : 5;
}

// One block per pipe, one lane per cell. All substeps stay on the device.
// The host submits all pipe interiors together. Each cell is read from mapped
// host memory once, evolves in shared memory, then is written back once.
// Host access resumes only after stream completion, including status reads.
__global__ void solve(Pipe *pipes,int *error) {
    Pipe &pipe=pipes[blockIdx.x];
    double dt=pipe.timestep;
    const int i=threadIdx.x,n=pipe.count;
    const double fuelMass=pipe.fuelMass,dx=pipe.dx,friction=pipe.friction,diameter=pipe.diameter;
    __shared__ int failureCode;
    if(i==0) failureCode=0;
    __shared__ double u[64][8],f[65][8],p[64],v[64],speed[64],h;
    if (i<n) for (int k=0;k<8;++k) u[i][k]=pipe.u[i][k];
    __syncthreads();
    int steps=0;
    while (dt>0) {
        if (++steps>10000) { if(i==0) error[blockIdx.x]=1; return; }
        if (i<n) {
            const int failure=properties(u[i],fuelMass,p[i],v[i],speed[i]);
            if(failure) { atomicCAS(&failureCode,0,failure); speed[i]=1; p[i]=0; v[i]=0; }
        }
        __syncthreads();
        if(i==0) {
            double maximum=1;
            for(int j=0;j<n;++j) maximum=fmax(maximum,speed[j]);
            h=fmin(dt,.25*dx/maximum);
            for(int k=0;k<8;++k) f[0][k]=f[n][k]=0;
            f[0][5]=p[0]; f[n][5]=p[n-1];
        }
        if(i>0 && i<n) {
            const double a=fmax(speed[i-1],speed[i]);
            for(int k=0;k<8;++k) {
                double left=u[i-1][k]*v[i-1],right=u[i][k]*v[i];
                if(k==5) {left+=p[i-1]; right+=p[i];}
                if(k==6) {left+=p[i-1]*v[i-1]; right+=p[i]*v[i];}
                f[i][k]=.5*(left+right)-.5*a*(u[i][k]-u[i-1][k]);
            }
        }
        __syncthreads();
        if(i<n) {
            for(int k=0;k<8;++k) u[i][k]-=h/dx*(f[i+1][k]-f[i][k]);
            for(int k=0;k<5;++k) {
                if(!isfinite(u[i][k]) || u[i][k]<-1e-10) atomicCAS(&failureCode,0,6);
                u[i][k]=fmax(0.0,u[i][k]);
            }
            const double rho=density(u[i],fuelMass);
            const double internal=u[i][6]-.5*u[i][5]*u[i][5]/rho;
            if(!(rho>0) || !(internal>0) || !isfinite(internal)) atomicCAS(&failureCode,0,6);
            u[i][7]=fmin(rho,fmax(0.0,u[i][7]));
            u[i][5]/=1+friction*fabs(u[i][5]/rho)*h/(2*diameter);
            // Keep total energy: friction converts bulk kinetic energy to heat.
        }
        __syncthreads();
        dt-=h;
    }
    if(i<n) for(int k=0;k<8;++k) pipe.u[i][k]=u[i][k];
    if(i==0) error[blockIdx.x]=failureCode;
}
void check(cudaError_t result,const char *operation) {
    if(result!=cudaSuccess) throw std::runtime_error(std::string(operation)+": "+cudaGetErrorString(result));
}
struct Context {
    Pipe *device=nullptr,*host=nullptr;
    int *error=nullptr,*hostError=nullptr;
    cudaStream_t stream=nullptr;
    cudaGraph_t graph=nullptr;
    cudaGraphExec_t executable=nullptr;
    int batchCount=0;
    char name[256]{};
    Context() {
        try {
            check(cudaSetDeviceFlags(cudaDeviceMapHost),"CUDA mapped-memory mode");
            int count=0; check(cudaGetDeviceCount(&count),"CUDA device discovery");
            if(!count) throw std::runtime_error("No CUDA device available");
            check(cudaSetDevice(0),"CUDA device selection");
            cudaDeviceProp properties{}; check(cudaGetDeviceProperties(&properties,0),"CUDA device properties");
            if(!properties.canMapHostMemory) throw std::runtime_error("CUDA device does not support mapped host memory");
            std::strncpy(name,properties.name,sizeof(name)-1);
            check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),"CUDA stream");
            check(cudaMemcpyToSymbol(cp,gas_thermo::coefficients,sizeof(gas_thermo::coefficients)),"CUDA thermodynamic coefficients");
        } catch(...) { release(); throw; }
    }
    void release() noexcept {
        if(stream) cudaStreamSynchronize(stream);
        if(executable) cudaGraphExecDestroy(executable);
        if(graph) cudaGraphDestroy(graph);
        if(host) cudaFreeHost(host);
        if(hostError) cudaFreeHost(hostError);
        if(stream) cudaStreamDestroy(stream);
    }
    void prepare(int count) {
        if(count==batchCount) return;
        check(cudaStreamSynchronize(stream),"CUDA resize synchronization");
        if(executable) {cudaGraphExecDestroy(executable); executable=nullptr;}
        if(graph) {cudaGraphDestroy(graph); graph=nullptr;}
        if(host) {cudaFreeHost(host); host=nullptr; device=nullptr;}
        if(hostError) {cudaFreeHost(hostError); hostError=nullptr; error=nullptr;}
        batchCount=0;
        const size_t bytes=static_cast<size_t>(count)*sizeof(Pipe);
        check(cudaHostAlloc(&host,bytes,cudaHostAllocMapped),"CUDA mapped pipe allocation");
        check(cudaHostGetDevicePointer(&device,host,0),"CUDA mapped pipe address");
        check(cudaHostAlloc(&hostError,count*sizeof(int),cudaHostAllocMapped),"CUDA mapped status allocation");
        check(cudaHostGetDevicePointer(&error,hostError,0),"CUDA mapped status address");
        check(cudaStreamBeginCapture(stream,cudaStreamCaptureModeThreadLocal),"CUDA graph capture");
        solve<<<count,64,0,stream>>>(device,error);
        check(cudaGetLastError(),"CUDA pipe launch");
        check(cudaStreamEndCapture(stream,&graph),"CUDA graph finish");
        check(cudaGraphInstantiate(&executable,graph,nullptr,nullptr,0),"CUDA graph instantiation");
        batchCount=count;
    }
    ~Context(){release();}
};
Context &context(){thread_local Context instance; return instance;}
}
bool enabled() {
    const char *value=std::getenv("ENGINE_SIM_GPU");
    return value && value[0]=='1';
}
const char *deviceName(){return enabled()?context().name:"CPU";}
void advance(Pipe *pipes,int count,double dt) {
    if(count<1 || count>4096 || !std::isfinite(dt) || dt<0) throw std::invalid_argument("Invalid CUDA pipe batch");
    for(int i=0;i<count;++i)
        if(pipes[i].count<2 || pipes[i].count>64 || !(pipes[i].dx>0) || !(pipes[i].diameter>0))
            throw std::invalid_argument("Invalid CUDA pipe geometry");
    auto &c=context();
    c.prepare(count);
    for(int i=0;i<count;++i) {
        auto &out=c.host[i]; const auto &in=pipes[i];
        out.count=in.count; out.dx=in.dx; out.diameter=in.diameter;
        out.friction=in.friction; out.fuelMass=in.fuelMass; out.timestep=dt;
        std::memcpy(out.u,in.u,in.count*sizeof(in.u[0]));
    }
    check(cudaGraphLaunch(c.executable,c.stream),"CUDA pipe graph launch");
    check(cudaStreamSynchronize(c.stream),"CUDA pipe completion");
    for(int i=0;i<count;++i) {
        if(c.hostError[i]==1) throw std::runtime_error("CUDA pipe exceeded substep limit");
        if(c.hostError[i]) throw std::runtime_error("CUDA pipe positivity failure, diagnostic code "+std::to_string(c.hostError[i]));
    }
    for(int i=0;i<count;++i)
        std::memcpy(pipes[i].u,c.host[i].u,pipes[i].count*sizeof(pipes[i].u[0]));
}
}
