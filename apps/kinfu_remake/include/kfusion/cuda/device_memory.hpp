#pragma once

#include <opencv2/core/cuda/common.hpp>

namespace kf
{
    namespace cuda
    {
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Memory class
          *
          * \note This is a BLOB container class with reference counting for GPU memory.
          *
          * \author Anatoly Baksheev
          */

        class Memory
        {
        public:
            /** \brief Empty constructor. */
            Memory();

            /** \brief Destructor. */
            ~Memory();

            /** \brief Allocates internal buffer in GPU memory
              * \param sizeBytes_arg: amount of memory to allocate
              * */
            Memory(size_t sizeBytes_arg);

            /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
              * \param ptr_arg: pointer to buffer
              * \param sizeBytes_arg: buffer size
              * */
            Memory(void *ptr_arg, size_t sizeBytes_arg);

            /** \brief Copy constructor. Just increments reference counter. */
            Memory(const Memory& other_arg);

            /** \brief Assigment operator. Just increments reference counter. */
            Memory& operator=(const Memory& other_arg);

             /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
               * \param sizeBytes_arg: buffer size
               * */
            void create(size_t sizeBytes_arg);

            /** \brief Decrements reference counter and releases internal buffer if needed. */
            void release();

            /** \brief Performs data copying. If destination size differs it will be reallocated.
              * \param other_arg: destination container
              * */
            void copyTo(Memory& other) const;

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param host_ptr_arg: pointer to buffer to upload
              * \param sizeBytes_arg: buffer size
              * */
            void upload(const void *host_ptr_arg, size_t sizeBytes_arg);

            /** \brief Downloads data from internal buffer to CPU memory
              * \param host_ptr_arg: pointer to buffer to download
              * */
            void download(void *host_ptr_arg) const;

            /** \brief Performs swap of data pointed with another device memory.
              * \param other: device memory to swap with
              * */
            void swap(Memory& other_arg);

            /** \brief Returns pointer for internal buffer in GPU memory. */
            template<class T> T* ptr();

            /** \brief Returns constant pointer for internal buffer in GPU memory. */
            template<class T> const T* ptr() const;

            /** \brief Conversion to PtrSz for passing to kernel functions. */
            template <class U> operator cv::cuda::PtrSz<U>() const;

            /** \brief Returns true if unallocated otherwise false. */
            bool empty() const;

            size_t sizeBytes() const;

        private:
            /** \brief Device pointer. */
            void *data_;

            /** \brief Allocated size in bytes. */
            size_t sizeBytes_;

            /** \brief Pointer to reference counter in CPU memory. */
            int* refcount_;
        };

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Memory2D class
          *
          * \note This is a BLOB container class with reference counting for pitched GPU memory.
          *
          * \author Anatoly Baksheev
          */

        class Memory2D
        {
        public:
            /** \brief Empty constructor. */
            Memory2D();

            /** \brief Destructor. */
            ~Memory2D();

            /** \brief Allocates internal buffer in GPU memory
              * \param rows_arg: number of rows to allocate
              * \param colsBytes_arg: width of the buffer in bytes
              * */
            Memory2D(int rows_arg, int colsBytes_arg);


            /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
              * \param rows_arg: number of rows
              * \param colsBytes_arg: width of the buffer in bytes
              * \param data_arg: pointer to buffer
              * \param stepBytes_arg: stride between two consecutive rows in bytes
              * */
            Memory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg);

            /** \brief Copy constructor. Just increments reference counter. */
            Memory2D(const Memory2D& other_arg);

            /** \brief Assigment operator. Just increments reference counter. */
            Memory2D& operator=(const Memory2D& other_arg);

            /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
               * \param ptr_arg: number of rows to allocate
               * \param sizeBytes_arg: width of the buffer in bytes
               * */
            void create(int rows_arg, int colsBytes_arg);

            /** \brief Decrements reference counter and releases internal buffer if needed. */
            void release();

            /** \brief Performs data copying. If destination size differs it will be reallocated.
              * \param other_arg: destination container
              * */
            void copyTo(Memory2D& other) const;

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param host_ptr_arg: pointer to host buffer to upload
              * \param host_step_arg: stride between two consecutive rows in bytes for host buffer
              * \param rows_arg: number of rows to upload
              * \param sizeBytes_arg: width of host buffer in bytes
              * */
            void upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg);

            /** \brief Downloads data from internal buffer to CPU memory. User is resposible for correct host buffer size.
              * \param host_ptr_arg: pointer to host buffer to download
              * \param host_step_arg: stride between two consecutive rows in bytes for host buffer
              * */
            void download(void *host_ptr_arg, size_t host_step_arg) const;

            /** \brief Performs swap of data pointed with another device memory.
              * \param other: device memory to swap with
              * */
            void swap(Memory2D& other_arg);

            /** \brief Returns pointer to given row in internal buffer.
              * \param y_arg: row index
              * */
            template<class T> T* ptr(int y_arg = 0);

            /** \brief Returns constant pointer to given row in internal buffer.
              * \param y_arg: row index
              * */
            template<class T> const T* ptr(int y_arg = 0) const;

             /** \brief Conversion to PtrStep for passing to kernel functions. */
            template <class U> operator cv::cuda::PtrStep<U>() const;

            /** \brief Conversion to PtrStepSz for passing to kernel functions. */
            template <class U> operator cv::cuda::PtrStepSz<U>() const;

            /** \brief Returns true if unallocated otherwise false. */
            bool empty() const;

            /** \brief Returns number of bytes in each row. */
            int colsBytes() const;

            /** \brief Returns number of rows. */
            int rows() const;

            /** \brief Returns stride between two consecutive rows in bytes for internal buffer. Step is stored always and everywhere in bytes!!! */
            size_t step() const;
        private:
            /** \brief Device pointer. */
            void *data_;

            /** \brief Stride between two consecutive rows in bytes for internal buffer. Step is stored always and everywhere in bytes!!! */
            size_t step_;

            /** \brief Width of the buffer in bytes. */
            int colsBytes_;

            /** \brief Number of rows. */
            int rows_;

            /** \brief Pointer to reference counter in CPU memory. */
            int* refcount_;
        };
    }
}

/////////////////////  Inline implementations of Memory ////////////////////////////////////////////

template<class T> inline       T* kf::cuda::Memory::ptr()       { return (      T*)data_; }
template<class T> inline const T* kf::cuda::Memory::ptr() const { return (const T*)data_; }

template <class U> inline kf::cuda::Memory::operator cv::cuda::PtrSz<U>() const
{
    cv::cuda::PtrSz<U> result;
    result.data = (U*)ptr<U>();
    result.size = sizeBytes_/sizeof(U);
    return result;
}

/////////////////////  Inline implementations of Memory2D ////////////////////////////////////////////

template<class T>        T* kf::cuda::Memory2D::ptr(int y_arg)       { return (      T*)((      char*)data_ + y_arg * step_); }
template<class T>  const T* kf::cuda::Memory2D::ptr(int y_arg) const { return (const T*)((const char*)data_ + y_arg * step_); }

template <class U> kf::cuda::Memory2D::operator cv::cuda::PtrStep<U>() const
{
    cv::cuda::PtrStep<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    return result;
}

template <class U> kf::cuda::Memory2D::operator cv::cuda::PtrStepSz<U>() const
{
    cv::cuda::PtrStepSz<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    result.cols = colsBytes_/sizeof(U);
    result.rows = rows_;
    return result;
}
