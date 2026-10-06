#include "../include/gpu_pipe.h"
#include "../include/gpu_gas.h"
#include "../include/gpu_chamber.h"
#include "../include/gpu_coupled.h"
#include "../include/gas_transport.h"
#include <cooperative_groups.h>
#include <cstdio>
#include "../include/simulation_profile.h"
#include "../include/gas_thermo.h"
#include <cuda_runtime.h>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

// Compile the same numerical definitions for the device. CPU definitions remain
// in the ordinary engine-sim library; inline device definitions avoid host stubs.
#include "gas_system.cpp"

namespace gpu_pipe {
namespace {
__constant__ double cp[5][2][5];
__constant__ double minimumCvRatio[5];
__global__ void solveCylinders(gpu_chamber::Cylinder *cylinders,int count,double dt) {
    const int index=blockIdx.x*blockDim.x+threadIdx.x;
    if(index>=count) return;
    auto &c=cylinders[index];
    chamber_flow::advanceDistributed(chamber_flow::view(c.state),c.parameters,c.intake,c.exhaust,dt);
}
__global__ void solveTransfers(gpu_gas::Transfer *transfers,int count) {
    const int index=blockIdx.x*blockDim.x+threadIdx.x;
    if(index>=count) return;
    auto &transfer=transfers[index];
    if(transfer.fixedEnvironment) {
        transfer.transferredMoles=transfer.a.flow(transfer.conductance,transfer.dt,
            transfer.b.pressure(),transfer.b.temperature(),transfer.b.mix());
        return;
    }
    GasSystem::FlowParameters p{transfer.conductance,transfer.dt,transfer.directionX,transfer.directionY,
        transfer.areaA,transfer.areaB,&transfer.a,&transfer.b};
    transfer.transferredMoles=GasSystem::flow(p);
}
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
__device__ void solveBlock(Pipe &pipe,int *error) {
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
        if (++steps>10000) { if(i==0) *error=1; return; }
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
    if(i<n) {
        for(int k=0;k<8;++k) pipe.u[i][k]=u[i][k];
        const double rho=density(u[i],fuelMass), velocity=u[i][5]/rho;
        const double internal=u[i][6]-.5*u[i][5]*velocity;
        const double n=u[i][0]+u[i][1]+u[i][2]+u[i][3]+u[i][4];
        const double cvBound=u[i][0]*minimumCvRatio[4]+u[i][1]*minimumCvRatio[1]
            +u[i][2]*minimumCvRatio[0]+u[i][3]*minimumCvRatio[2]+u[i][4]*minimumCvRatio[3];
        // cv >= cv_min implies T <= u/cv_min and gamma <= 1+R/cv_min.
        // Consequently this bounds |v|+c from above, never enlarging the CFL step.
        const double q=n/cvBound;
        speed[i]=fabs(velocity)+sqrt(q*(1+q)*internal/rho);
        if(!isfinite(speed[i])) {atomicCAS(&failureCode,0,6); speed[i]=1;}
    }
    __syncthreads();
    if(i==0) {
        double maximum=1;
        for(int j=0;j<n;++j) maximum=fmax(maximum,speed[j]);
        pipe.stableTimestep=.25*dx/maximum;
        *error=failureCode;
    }
}
__global__ void solve(Pipe *pipes,int *error) {
    solveBlock(pipes[blockIdx.x],error+blockIdx.x);
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
            check(cudaMemcpyToSymbol(gas_thermo::deviceCoefficients,gas_thermo::coefficients,sizeof(gas_thermo::coefficients)),"CUDA shared gas coefficients");
            check(cudaMemcpyToSymbol(minimumCvRatio,gas_thermo::minimumCvRatio,sizeof(gas_thermo::minimumCvRatio)),"CUDA heat-capacity bounds");
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
    // Fixed at launch; cached so the CFL substep loop does not pay a CRT
    // getenv per substep.
    static const bool value=[] {
        const char *env=std::getenv("ENGINE_SIM_GPU");
        return env && env[0]=='1';
    }();
    return value;
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
    {
        ENGINE_SIM_PROFILE_SCOPE(DeviceWait);
        check(cudaGraphLaunch(c.executable,c.stream),"CUDA pipe graph launch");
        check(cudaStreamSynchronize(c.stream),"CUDA pipe completion");
    }
    for(int i=0;i<count;++i) {
        if(c.hostError[i]==1) throw std::runtime_error("CUDA pipe exceeded substep limit");
        if(c.hostError[i]) throw std::runtime_error("CUDA pipe positivity failure, diagnostic code "+std::to_string(c.hostError[i]));
    }
    for(int i=0;i<count;++i) {
        std::memcpy(pipes[i].u,c.host[i].u,pipes[i].count*sizeof(pipes[i].u[0]));
        pipes[i].stableTimestep=c.host[i].stableTimestep;
    }
}
}

namespace gpu_gas {
void advance(Transfer *transfers,int count) {
    if(!transfers || count<1 || count>4096) throw std::invalid_argument("Invalid CUDA gas transfer batch");
    for(int i=0;i<count;++i) {
        const auto &t=transfers[i];
        if(!std::isfinite(t.dt) || t.dt<0 || !std::isfinite(t.conductance) || t.conductance<0
            || !std::isfinite(t.areaA) || t.areaA<0 || !std::isfinite(t.areaB) || t.areaB<0
            || !std::isfinite(t.directionX) || !std::isfinite(t.directionY))
            throw std::invalid_argument("Invalid CUDA gas transfer parameters");
    }
    auto &context=gpu_pipe::context();
    std::vector<Transfer> result(count);
    Transfer *device=nullptr;
    const size_t bytes=static_cast<size_t>(count)*sizeof(Transfer);
    try {
        gpu_pipe::check(cudaMalloc(&device,bytes),"CUDA gas allocation");
        gpu_pipe::check(cudaMemcpyAsync(device,transfers,bytes,cudaMemcpyHostToDevice,context.stream),"CUDA gas upload");
        gpu_pipe::solveTransfers<<<(count+63)/64,64,0,context.stream>>>(device,count);
        gpu_pipe::check(cudaGetLastError(),"CUDA gas launch");
        gpu_pipe::check(cudaMemcpyAsync(result.data(),device,bytes,cudaMemcpyDeviceToHost,context.stream),"CUDA gas download");
        gpu_pipe::check(cudaStreamSynchronize(context.stream),"CUDA gas completion");
        gpu_pipe::check(cudaFree(device),"CUDA gas release"); device=nullptr;
    } catch(...) {
        if(device) cudaFree(device);
        throw;
    }
    std::copy(result.begin(),result.end(),transfers);
}
}

namespace gpu_chamber {
void advance(Cylinder *cylinders,int count,double dt) {
    if(!cylinders || count<1 || count>4096 || !std::isfinite(dt) || dt<0)
        throw std::invalid_argument("Invalid CUDA cylinder batch");
    for(int i=0;i<count;++i) {
        const auto &p=cylinders[i].parameters;
        for(double value : {p.volume,p.cylinderHeight,p.bore,p.boreArea,p.fuelMass})
            if(!std::isfinite(value) || value<=0) throw std::invalid_argument("Invalid CUDA cylinder geometry or fuel mass");
        for(double value : {p.surfaceArea,p.blowbyK,p.crankcasePressure,p.intakeK,p.exhaustK,
                p.intakeArea,p.exhaustArea,p.fuelEnergyDensity})
            if(!std::isfinite(value) || value<0) throw std::invalid_argument("Invalid CUDA cylinder flow parameters");
        if(!std::isfinite(p.meanPistonSpeed)) throw std::invalid_argument("Invalid CUDA mean piston speed");
    }
    auto &context=gpu_pipe::context();
    std::vector<Cylinder> result(count);
    Cylinder *device=nullptr;
    const size_t bytes=static_cast<size_t>(count)*sizeof(Cylinder);
    try {
        gpu_pipe::check(cudaMalloc(&device,bytes),"CUDA cylinder allocation");
        gpu_pipe::check(cudaMemcpyAsync(device,cylinders,bytes,cudaMemcpyHostToDevice,context.stream),"CUDA cylinder upload");
        gpu_pipe::solveCylinders<<<(count+63)/64,64,0,context.stream>>>(device,count,dt);
        gpu_pipe::check(cudaGetLastError(),"CUDA cylinder launch");
        gpu_pipe::check(cudaMemcpyAsync(result.data(),device,bytes,cudaMemcpyDeviceToHost,context.stream),"CUDA cylinder download");
        gpu_pipe::check(cudaStreamSynchronize(context.stream),"CUDA cylinder completion");
        gpu_pipe::check(cudaFree(device),"CUDA cylinder release"); device=nullptr;
    } catch(...) {
        if(device) cudaFree(device);
        throw;
    }
    std::copy(result.begin(),result.end(),cylinders);
}
}

namespace gpu_coupled {
namespace {
struct Control {
    double remaining=0,h=0;
    int error=0;
    int lastSubstep=0;
    bool profile=false;
    unsigned long long ticks[8]{};
};
struct DeviceBatch {
    GasSystem *cells;
    Pipe *pipes;
    Cylinder *cylinders;
    Intake *intakes;
    Exhaust *exhausts;
    Control *control;
    int pipeCount,cylinderCount,intakeCount,exhaustCount;
};
__global__ void solveCoupled(DeviceBatch b,double timestep,int fluidSteps) {
    const auto grid=cooperative_groups::this_grid();
    const int lane=threadIdx.x;
    const double fluidDt=timestep/fluidSteps;
    const bool leader=blockIdx.x==0 && lane==0;
    const bool profile=b.control->profile && leader;
    const int blocks=gridDim.x-1;
    // Reservoir-side port transfers and the cylinder stages touch opposite ends
    // of each pipe (manifold end vs chamber end), so the two commute and can
    // share one phase. Keep them separate when a cylinder's pipes alias or a
    // pipe has a single cell, and when the reservoirs would leave too few blocks
    // to give every cylinder its own.
    const int reservoirCount=b.intakeCount+b.exhaustCount;
    const int portOwners=(reservoirCount<blocks)?reservoirCount:blocks;
    bool portsMergeable=true;
    for(int i=0;i<b.cylinderCount;++i) {
        const auto &c=b.cylinders[i];
        if(c.intakePipe==c.exhaustPipe
            || b.pipes[c.intakePipe].solver.count<2
            || b.pipes[c.exhaustPipe].solver.count<2) {portsMergeable=false;break;}
    }
    const bool mergedPorts=portsMergeable && portOwners+b.cylinderCount<=blocks;
    const int portLimit=mergedPorts?portOwners:blocks;
    const int portSpan=portLimit;
    const int cylBase=mergedPorts?portOwners:0;
    const int cylSpan=blocks-cylBase;
    const int cylinderOwners=(b.cylinderCount<cylSpan)?b.cylinderCount:cylSpan;
    // Port chains write the same reservoir state the atmosphere stage does, so
    // atmospheres may share only the cylinder blocks (or a later stage), never
    // the port blocks.
    const bool atmRidesCylinders=!mergedPorts && blocks>cylinderOwners;
    // Cylinders own the leading blocks; atmosphere work for the next fluid step
    // commutes with them (reservoirs vs pipe cells), so the trailing blocks can
    // run it during the cylinder phase instead of waiting for a dedicated phase.
    const auto atmospheres=[&](int base,int span) {
        if(span<=0 || blockIdx.x==0 || lane!=0) return;
        const int owner=blockIdx.x-1-base;
        if(owner<0) return;
        for(int i=owner;i<b.exhaustCount;i+=span)
            reservoir_flow::process(reservoir_flow::view(b.exhausts[i].state),b.exhausts[i].parameters,fluidDt);
        for(int i=owner;i<b.intakeCount;i+=span) {
            auto &intake=b.intakes[i];
            reservoir_flow::process(reservoir_flow::view(intake.state),intake.parameters,fluidDt);
            intake.state.flowRate+=intake.state.flow;
        }
    };
    // Reservoirs are independent; each retains its own cylinder order. Keep the
    // active gas values local during flow evaluation, then publish once.
    const auto ports=[&](double dt) {
        if(blockIdx.x==0 || lane!=0 || blockIdx.x>portLimit) return;
        for(int reservoir=blockIdx.x-1;reservoir<reservoirCount;reservoir+=portSpan) {
            const bool isIntake=reservoir<b.intakeCount;
            const int index=isIntake?reservoir:reservoir-b.intakeCount;
            GasSystem system=isIntake?b.intakes[index].state.system:b.exhausts[index].state.system;
            for(int i=0;i<b.cylinderCount;++i) {
                const auto &c=b.cylinders[i];
                if((isIntake?c.intake:c.exhaust)!=index) continue;
                const auto &pipe=b.pipes[isIntake?c.intakePipe:c.exhaustPipe];
                const int cell=pipe.firstCell+(isIntake?0:pipe.solver.count-1);
                GasSystem endpoint=b.cells[cell];
                GasSystem::FlowParameters flow;
                flow.dt=dt; flow.direction_x=1; flow.direction_y=0;
                if(isIntake) {
                    flow.k_flow=c.manifoldK; flow.crossSectionArea_0=b.intakes[index].parameters.crossSectionArea;
                    flow.crossSectionArea_1=c.parameters.intakeArea; flow.system_0=&system; flow.system_1=&endpoint;
                } else {
                    flow.k_flow=c.collectorK; flow.crossSectionArea_0=c.parameters.exhaustArea;
                    flow.crossSectionArea_1=b.exhausts[index].parameters.collectorArea;
                    flow.system_0=&endpoint; flow.system_1=&system;
                }
                GasSystem::flow(flow); b.cells[cell]=endpoint;
            }
            if(isIntake) b.intakes[index].state.system=system; else b.exhausts[index].state.system=system;
        }
    };
    unsigned long long stamp=0;
    if(profile) stamp=clock64();
    for(int fluid=0;fluid<fluidSteps;++fluid) {
        // Fluid step f+1's atmosphere work only needs fluid step f's reservoir
        // ports, so a merged stage of f's final substep already did it. It can
        // never share a stage with the port chain itself: both rewrite the same
        // reservoir systems.
        if(fluid==0 || !b.control->lastSubstep || !(mergedPorts||atmRidesCylinders))
            atmospheres(0,blocks);
        if(leader) b.control->remaining=fluidDt;
        grid.sync();
        if(profile) {const auto now=clock64(); b.control->ticks[0]+=now-stamp; stamp=now;}
        int steps=0;
        while(true) {
            if(blockIdx.x==0 && lane==0) {
                b.control->h=b.control->remaining;
                if(b.control->remaining>0) {
                    if(++steps>10000) b.control->error=1;
                    for(int i=0;i<b.pipeCount;++i)
                        b.control->h=fmin(b.control->h,b.pipes[i].solver.stableTimestep);
                    if(!(b.control->h>0) || !isfinite(b.control->h)) b.control->error=7;
                    // No pipe clamped h, so this substep consumes the fluid step.
                    b.control->lastSubstep=!(b.control->h<b.control->remaining);
                }
            }
            grid.sync();
            if(profile) {const auto now=clock64(); b.control->ticks[1]+=now-stamp; stamp=now;}
            if(b.control->error) return; // Uniform exit after a grid barrier.
            if(b.control->remaining<=0) break;
            const double dt=b.control->h;
            // Ports and cylinders touch opposite pipe ends, so a merged engine
            // runs both in one stage; otherwise ports keep their own stage.
            const bool mergeAtm=b.control->lastSubstep && fluid+1<fluidSteps
                && (mergedPorts||atmRidesCylinders);
            if(!mergedPorts) {
                ports(dt);
                grid.sync();
                if(profile) {const auto now=clock64(); b.control->ticks[2]+=now-stamp; stamp=now;}
            }
            else ports(dt);
            if(!mergedPorts && mergeAtm) atmospheres(cylinderOwners,blocks-cylinderOwners);
            if(blockIdx.x>cylBase && lane==0) for(int i=blockIdx.x-1-cylBase;i<b.cylinderCount;i+=cylSpan) {
                auto &c=b.cylinders[i]; const auto &ip=b.pipes[c.intakePipe],&ep=b.pipes[c.exhaustPipe];
                auto state=c.state;
                GasSystem intake=b.cells[ip.firstCell+ip.solver.count-1],exhaust=b.cells[ep.firstCell];
                chamber_flow::advanceDistributed(chamber_flow::view(state),c.parameters,intake,exhaust,dt);
                c.state=state; b.cells[ip.firstCell+ip.solver.count-1]=intake; b.cells[ep.firstCell]=exhaust;
            }
            grid.sync();
            if(profile) {const auto now=clock64();
                b.control->ticks[(mergedPorts||mergeAtm)?6:3]+=now-stamp; stamp=now;}
            if(mergedPorts && mergeAtm) {
                // The port chain has published this fluid step's reservoirs, so
                // the next step's atmospheres can run as their own stage now.
                atmospheres(0,blocks);
                grid.sync();
                if(profile) {const auto now=clock64(); b.control->ticks[0]+=now-stamp; stamp=now;}
            }
            if(blockIdx.x>0) {
                auto &pipe=b.pipes[blockIdx.x-1];
                if(lane<pipe.solver.count) GasTransport::conserved(b.cells[pipe.firstCell+lane],pipe.solver.u[lane]);
                if(lane==0) pipe.solver.timestep=dt;
                __syncthreads();
                __shared__ int pipeError;
                gpu_pipe::solveBlock(pipe.solver,&pipeError);
                __syncthreads();
                if(lane==0 && pipeError) atomicCAS(&b.control->error,0,pipeError);
                if(lane<pipe.solver.count) {
                    auto gas=b.cells[pipe.firstCell+lane];
                    const int error=GasTransport::restore(gas,pipe.solver.u[lane]);
                    b.cells[pipe.firstCell+lane]=gas;
                    if(error) atomicCAS(&b.control->error,0,error);
                }
            }
            grid.sync();
            if(profile) {const auto now=clock64(); b.control->ticks[4]+=now-stamp; stamp=now;}
            if(b.control->error) return;
            if(blockIdx.x==0 && lane==0) b.control->remaining-=dt;
            grid.sync();
            if(profile) {const auto now=clock64(); b.control->ticks[5]+=now-stamp; stamp=now;}
        }
    }
}

template<class T> struct Buffer {
    T *host=nullptr,*device=nullptr;
    size_t capacity=0;
    ~Buffer() { if(host) cudaFreeHost(host); if(device) cudaFree(device); }
    void prepare(size_t count) {
        if(count<=capacity) return;
        if(host) {cudaFreeHost(host);host=nullptr;}
        if(device) {cudaFree(device);device=nullptr;}
        capacity=0;
        gpu_pipe::check(cudaHostAlloc(&host,count*sizeof(T),cudaHostAllocDefault),"Coupled host allocation");
        gpu_pipe::check(cudaMalloc(&device,count*sizeof(T)),"Coupled device allocation");
        capacity=count;
    }
    void upload(const T *data,size_t count,cudaStream_t stream) {
        if(!count) return;
        prepare(count); std::memcpy(host,data,count*sizeof(T));
        gpu_pipe::check(cudaMemcpyAsync(device,host,count*sizeof(T),cudaMemcpyHostToDevice,stream),"Coupled upload");
    }
    void download(size_t count,cudaStream_t stream) {
        if(count) gpu_pipe::check(cudaMemcpyAsync(host,device,count*sizeof(T),cudaMemcpyDeviceToHost,stream),"Coupled download");
    }
    void publish(T *data,size_t count) {if(count) std::memcpy(data,host,count*sizeof(T));}
};
struct Context {
    Buffer<GasSystem> cells;
    Buffer<Pipe> pipes;
    Buffer<Cylinder> cylinders;
    Buffer<Intake> intakes;
    Buffer<Exhaust> exhausts;
    Buffer<Control> control;
    int maximumBlocks=0;
    unsigned long long ticks[8]{},profiledSteps=0;
    ~Context() {
        if(profiledSteps) {
            unsigned long long total=0; for(auto value:ticks) total+=value;
            const char *names[]={"atmospheres","CFL/barrier","reservoir ports","cylinders","pipes",
                "final barrier","merged stage","unused"};
            std::fprintf(stderr,"Coupled GPU profile: %llu mechanical steps\n",profiledSteps);
            for(int i=0;i<8;++i) std::fprintf(stderr,"  %s: %.2f%%\n",names[i],100.0*ticks[i]/total);
        }
    }
    Context() {
        cudaDeviceProp properties{};
        gpu_pipe::check(cudaGetDeviceProperties(&properties,0),"Coupled device properties");
        if(!properties.cooperativeLaunch) return;
        int blocks=0;
        gpu_pipe::check(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks,solveCoupled,64,0),"Coupled kernel occupancy");
        maximumBlocks=blocks*properties.multiProcessorCount;
    }
};
}
bool requested() {
    // Fixed at launch; cached like gpu_pipe::enabled(). The short-circuit
    // order is preserved: enabled() is still only consulted when the coupled
    // variable itself is set.
    static const bool value=[] {
        const char *env=std::getenv("ENGINE_SIM_GPU_COUPLED");
        return env && env[0]=='1';
    }();
    return value && gpu_pipe::enabled();
}
bool advance(Batch &b,double timestep,int fluidSteps) {
    if(!(timestep>0) || !std::isfinite(timestep) || fluidSteps<1 || fluidSteps>10000
        || b.pipes.empty() || b.pipes.size()>4096 || b.cylinders.empty())
        throw std::invalid_argument("Invalid coupled GPU batch");
    for(const auto &p:b.pipes)
        if(p.solver.count<2 || p.solver.count>64 || p.firstCell<0
            || static_cast<size_t>(p.firstCell+p.solver.count)>b.cells.size())
            throw std::invalid_argument("Invalid coupled GPU pipe cells");
    for(const auto &c:b.cylinders)
        if(c.intake<0 || static_cast<size_t>(c.intake)>=b.intakes.size()
            || c.exhaust<0 || static_cast<size_t>(c.exhaust)>=b.exhausts.size()
            || c.intakePipe<0 || static_cast<size_t>(c.intakePipe)>=b.pipes.size()
            || c.exhaustPipe<0 || static_cast<size_t>(c.exhaustPipe)>=b.pipes.size())
            throw std::invalid_argument("Invalid coupled GPU connections");
    auto &runtime=gpu_pipe::context();
    thread_local Context context;
    auto &c=context;
    if(b.pipes.size()+1>static_cast<size_t>(c.maximumBlocks)) return false;
    const auto stream=runtime.stream;
    try {
    // No callers can touch pinned/device staging until the previous call finishes.
    c.cells.upload(b.cells.data(),b.cells.size(),stream);
    c.pipes.upload(b.pipes.data(),b.pipes.size(),stream);
    c.cylinders.upload(b.cylinders.data(),b.cylinders.size(),stream);
    c.intakes.upload(b.intakes.data(),b.intakes.size(),stream);
    c.exhausts.upload(b.exhausts.data(),b.exhausts.size(),stream);
    Control initial{};
    const char *profiling=std::getenv("ENGINE_SIM_GPU_PROFILE");
    initial.profile=profiling && profiling[0]=='1';
    c.control.upload(&initial,1,stream);
    DeviceBatch device{c.cells.device,c.pipes.device,c.cylinders.device,c.intakes.device,c.exhausts.device,
        c.control.device,static_cast<int>(b.pipes.size()),static_cast<int>(b.cylinders.size()),
        static_cast<int>(b.intakes.size()),static_cast<int>(b.exhausts.size())};
    void *args[]={&device,&timestep,&fluidSteps};
    gpu_pipe::check(cudaLaunchCooperativeKernel(reinterpret_cast<void *>(solveCoupled),
        static_cast<unsigned>(b.pipes.size()+1),64,args,0,stream),"Coupled kernel launch");
    c.cells.download(b.cells.size(),stream); c.pipes.download(b.pipes.size(),stream);
    c.cylinders.download(b.cylinders.size(),stream); c.intakes.download(b.intakes.size(),stream);
    c.exhausts.download(b.exhausts.size(),stream); c.control.download(1,stream);
    gpu_pipe::check(cudaStreamSynchronize(stream),"Coupled completion");
    } catch(...) {
        cudaStreamSynchronize(stream);
        throw;
    }
    if(c.control.host->profile) {
        ++c.profiledSteps;
        for(int i=0;i<8;++i) c.ticks[i]+=c.control.host->ticks[i];
    }
    if(c.control.host->error) throw std::runtime_error("Coupled GPU physics failed, diagnostic code "+std::to_string(c.control.host->error));
    c.cells.publish(b.cells.data(),b.cells.size()); c.pipes.publish(b.pipes.data(),b.pipes.size());
    c.cylinders.publish(b.cylinders.data(),b.cylinders.size()); c.intakes.publish(b.intakes.data(),b.intakes.size());
    c.exhausts.publish(b.exhausts.data(),b.exhausts.size());
    return true;
}
}
