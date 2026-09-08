#include <cuda_runtime.h>
#include <cuda/atomic>
#include <chrono>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <cstdint>
#include <new>
#include <cstdlib>

// Standalone transport experiment, not an engine backend. Build with NVCC for
// the target GPU; pass 8 or 64 for active cells per pipe. The current graph
// transport and bounded workers exchange and verify identical payloads.
// Only aligned 32-bit system-scope loads/stores cross the CPU/GPU boundary:
// https://docs.nvidia.com/cuda/cuda-programming-guide/05-appendices/cuda-cpp-memory-model.html

using Atomic=cuda::atomic_ref<uint32_t,cuda::thread_scope_system>;
struct alignas(64) Flag {uint32_t value=0;};
struct Packet {double input[64][8],output[64][8];};
struct Control {Flag request; Flag done[24]; Flag retired[24];};
void check(cudaError_t e) {if(e!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(e));}
__device__ void work(Packet &p,int cells) {
    int i=threadIdx.x; if(i>=cells) return;
    for(int k=0;k<8;++k) p.output[i][k]=p.input[i][k]+i+k;
}
__global__ void once(Packet *p,int cells) {work(p[blockIdx.x],cells);}
__global__ void worker(Packet *p,Control *c,int requests,unsigned long long lifetime,int cells) {
    __shared__ uint32_t sequence;
    uint32_t previous=0;
    const auto start=clock64();
    for(int step=0;step<requests;++step) {
        if(threadIdx.x==0) {
            sequence=0;
            do {
                auto next=Atomic(c->request.value).load(cuda::memory_order_acquire);
                if(next!=previous) {sequence=next;break;}
                if(clock64()-start>lifetime) break;
                __nanosleep(64);
            } while(true);
        }
        __syncthreads();
        if(sequence==0) break;
        previous=sequence;
        work(p[blockIdx.x],cells);
        // Publish every lane's ordinary output stores before the block flag.
        __threadfence_system();
        __syncthreads();
        if(threadIdx.x==0) Atomic(c->done[blockIdx.x].value).store(sequence,cuda::memory_order_release);
        __syncthreads();
    }
    if(threadIdx.x==0) Atomic(c->retired[blockIdx.x].value).store(1,cuda::memory_order_release);
}
int main(int argc,char **argv) {
    const int cells=argc>1?std::atoi(argv[1]):8;
    if(cells!=8 && cells!=64) {std::fprintf(stderr,"Expected 8 or 64 cells\n");return 2;}
    Packet *host=nullptr,*device=nullptr; Control *control=nullptr,*deviceControl=nullptr;
    cudaStream_t stream=nullptr; cudaGraph_t graph=nullptr; cudaGraphExec_t executable=nullptr;
    try {
        check(cudaSetDeviceFlags(cudaDeviceMapHost));
        cudaDeviceProp properties{}; check(cudaGetDeviceProperties(&properties,0));
        std::printf("device=%s hostNativeAtomicSupported=%d SMs=%d cells=%d\n",properties.name,properties.hostNativeAtomicSupported,properties.multiProcessorCount,cells);
        if(!properties.canMapHostMemory || properties.multiProcessorCount<24)
            throw std::runtime_error("Probe requires mapped memory and at least 24 SMs");
        check(cudaHostAlloc(&host,24*sizeof(Packet),cudaHostAllocMapped));
        check(cudaHostGetDevicePointer(&device,host,0));
        check(cudaHostAlloc(&control,sizeof(Control),cudaHostAllocMapped));
        new(control) Control{};
        check(cudaHostGetDevicePointer(&deviceControl,control,0));
        check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));
        for(int blocks : {8,24}) {
            check(cudaStreamBeginCapture(stream,cudaStreamCaptureModeThreadLocal));
            once<<<blocks,64,0,stream>>>(device,cells);
            check(cudaStreamEndCapture(stream,&graph));
            check(cudaGraphInstantiate(&executable,graph,nullptr,nullptr,0));
            for(int persistent : {0,1}) {
                auto begin=std::chrono::steady_clock::now();
                constexpr int batch=128,total=4096;
                for(int step=0;step<total;++step) {
                    uint32_t seq=step%batch+1;
                    for(int b=0;b<blocks;++b) for(int i=0;i<cells;++i) for(int k=0;k<8;++k)
                        host[b].input[i][k]=step+b+i+k;
                    if(persistent) {
                        if(seq==1) {
                            *control=Control{}; // Prior kernel has completed.
                            worker<<<blocks,64,0,stream>>>(device,deviceControl,batch,static_cast<unsigned long long>(properties.clockRate)*50,cells);
                            check(cudaGetLastError());
                        }
                        Atomic(control->request.value).store(seq,cuda::memory_order_release);
                        auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
                        unsigned spins=0;
                        for(int b=0;b<blocks;++b) while(Atomic(control->done[b].value).load(cuda::memory_order_acquire)!=seq) {
                            if(Atomic(control->retired[b].value).load(cuda::memory_order_acquire)
                                && Atomic(control->done[b].value).load(cuda::memory_order_acquire)!=seq)
                                throw std::runtime_error("Worker retired during request");
                            if(++spins%65536==0) {
                                auto state=cudaStreamQuery(stream);
                                if(state!=cudaErrorNotReady) check(state);
                                if(std::chrono::steady_clock::now()>deadline) throw std::runtime_error("Host handshake timeout");
                            }
                        }
                        if(seq==batch) check(cudaStreamSynchronize(stream));
                    } else {
                        check(cudaGraphLaunch(executable,stream));
                        check(cudaStreamSynchronize(stream));
                    }
                    for(int b=0;b<blocks;++b) for(int i=0;i<cells;++i) for(int k=0;k<8;++k)
                        if(host[b].output[i][k]!=step+b+2*i+2*k) throw std::runtime_error("Output mismatch");
                }
                const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count()/total;
                std::printf("blocks=%d persistent=%d roundtrip_us=%.3f verified_requests=%d\n",blocks,persistent,us,total);
            }
            check(cudaGraphExecDestroy(executable)); executable=nullptr;
            check(cudaGraphDestroy(graph));graph=nullptr;
        }
        *control=Control{};
        worker<<<24,64,0,stream>>>(device,deviceControl,128,static_cast<unsigned long long>(properties.clockRate)*10,cells);
        check(cudaGetLastError());
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
        check(cudaStreamSynchronize(stream));
        for(int b=0;b<24;++b) if(!Atomic(control->retired[b].value).load(cuda::memory_order_acquire)) throw std::runtime_error("Idle retirement failed");
        std::puts("idle retirement passed");
    } catch(const std::exception &e) {
        std::fprintf(stderr,"%s\n",e.what());
        if(stream) cudaStreamSynchronize(stream);
        return 1;
    }
    if(executable) cudaGraphExecDestroy(executable);
    if(graph) cudaGraphDestroy(graph);
    cudaStreamDestroy(stream);cudaFreeHost(host);cudaFreeHost(control);
}

