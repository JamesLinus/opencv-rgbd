#include <kfusion/cuda/device_memory.hpp>
#include <cassert>
#include <iostream>
#include <cstdlib>

////////////////////////    Array    /////////////////////////////
    
kf::cuda::Memory::Memory() : data_(0), sizeBytes_(0), refcount_(0) {}
kf::cuda::Memory::Memory(void *ptr_arg, size_t sizeBytes_arg) : data_(ptr_arg), sizeBytes_(sizeBytes_arg), refcount_(0){}
kf::cuda::Memory::Memory(size_t sizeBtes_arg)  : data_(0), sizeBytes_(0), refcount_(0) { create(sizeBtes_arg); }
kf::cuda::Memory::~Memory() { release(); }

kf::cuda::Memory::Memory(const Memory& other_arg)
    : data_(other_arg.data_), sizeBytes_(other_arg.sizeBytes_), refcount_(other_arg.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

kf::cuda::Memory& kf::cuda::Memory::operator = (const kf::cuda::Memory& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        release();
        
        data_      = other_arg.data_;
        sizeBytes_ = other_arg.sizeBytes_;                
        refcount_  = other_arg.refcount_;
    }
    return *this;
}

void kf::cuda::Memory::create(size_t sizeBytes_arg)
{
    if (sizeBytes_arg == sizeBytes_)
        return;
            
    if( sizeBytes_arg > 0)
    {        
        if( data_ )
            release();

        sizeBytes_ = sizeBytes_arg;
                        
        cudaSafeCall( cudaMalloc(&data_, sizeBytes_) );
        
        //refcount_ = (int*)cv::fastMalloc(sizeof(*refcount_));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void kf::cuda::Memory::copyTo(Memory& other) const
{
    if (empty())
        other.release();
    else
    {    
        other.create(sizeBytes_);    
        cudaSafeCall( cudaMemcpy(other.data_, data_, sizeBytes_, cudaMemcpyDeviceToDevice) );
    }
}

void kf::cuda::Memory::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }
    data_ = 0;
    sizeBytes_ = 0;
    refcount_ = 0;
}

void kf::cuda::Memory::upload(const void *host_ptr_arg, size_t sizeBytes_arg)
{
    create(sizeBytes_arg);
    cudaSafeCall( cudaMemcpy(data_, host_ptr_arg, sizeBytes_, cudaMemcpyHostToDevice) );
}

void kf::cuda::Memory::download(void *host_ptr_arg) const
{    
    cudaSafeCall( cudaMemcpy(host_ptr_arg, data_, sizeBytes_, cudaMemcpyDeviceToHost) );
}          

void kf::cuda::Memory::swap(Memory& other_arg)
{
    std::swap(data_, other_arg.data_);
    std::swap(sizeBytes_, other_arg.sizeBytes_);
    std::swap(refcount_, other_arg.refcount_);
}

bool kf::cuda::Memory::empty() const { return !data_; }
size_t kf::cuda::Memory::sizeBytes() const { return sizeBytes_; }


////////////////////////    Array2D    /////////////////////////////

kf::cuda::Memory2D::Memory2D() : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0) {}

kf::cuda::Memory2D::Memory2D(int rows_arg, int colsBytes_arg)
    : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0)
{ 
    create(rows_arg, colsBytes_arg); 
}

kf::cuda::Memory2D::Memory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg)
    :  data_(data_arg), step_(step_arg), colsBytes_(colsBytes_arg), rows_(rows_arg), refcount_(0) {}

kf::cuda::Memory2D::~Memory2D() { release(); }


kf::cuda::Memory2D::Memory2D(const Memory2D& other_arg) :
    data_(other_arg.data_), step_(other_arg.step_), colsBytes_(other_arg.colsBytes_), rows_(other_arg.rows_), refcount_(other_arg.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

kf::cuda::Memory2D& kf::cuda::Memory2D::operator = (const kf::cuda::Memory2D& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        release();
        
        colsBytes_ = other_arg.colsBytes_;
        rows_ = other_arg.rows_;
        data_ = other_arg.data_;
        step_ = other_arg.step_;
                
        refcount_ = other_arg.refcount_;
    }
    return *this;
}

void kf::cuda::Memory2D::create(int rows_arg, int colsBytes_arg)
{
    if (colsBytes_ == colsBytes_arg && rows_ == rows_arg)
        return;
            
    if( rows_arg > 0 && colsBytes_arg > 0)
    {        
        if( data_ )
            release();
              
        colsBytes_ = colsBytes_arg;
        rows_ = rows_arg;
                        
        cudaSafeCall( cudaMallocPitch( (void**)&data_, &step_, colsBytes_, rows_) );        

        //refcount = (int*)cv::fastMalloc(sizeof(*refcount));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void kf::cuda::Memory2D::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }

    colsBytes_ = 0;
    rows_ = 0;    
    data_ = 0;    
    step_ = 0;
    refcount_ = 0;
}

void kf::cuda::Memory2D::copyTo(Memory2D& other) const
{
    if (empty())
        other.release();
    else
    {
        other.create(rows_, colsBytes_);    
        cudaSafeCall( cudaMemcpy2D(other.data_, other.step_, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToDevice) );
    }
}

void kf::cuda::Memory2D::upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg)
{
    create(rows_arg, colsBytes_arg);
    cudaSafeCall( cudaMemcpy2D(data_, step_, host_ptr_arg, host_step_arg, colsBytes_, rows_, cudaMemcpyHostToDevice) );        
}

void kf::cuda::Memory2D::download(void *host_ptr_arg, size_t host_step_arg) const
{    
    cudaSafeCall( cudaMemcpy2D(host_ptr_arg, host_step_arg, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );
}      

void kf::cuda::Memory2D::swap(Memory2D& other_arg)
{    
    std::swap(data_, other_arg.data_);
    std::swap(step_, other_arg.step_);

    std::swap(colsBytes_, other_arg.colsBytes_);
    std::swap(rows_, other_arg.rows_);
    std::swap(refcount_, other_arg.refcount_);                 
}

bool kf::cuda::Memory2D::empty() const { return !data_; }
int kf::cuda::Memory2D::colsBytes() const { return colsBytes_; }
int kf::cuda::Memory2D::rows() const { return rows_; }
size_t kf::cuda::Memory2D::step() const { return step_; }
