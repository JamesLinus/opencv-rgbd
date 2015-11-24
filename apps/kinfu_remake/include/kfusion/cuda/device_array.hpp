#pragma once

#include <kfusion/exports.hpp>
#include <kfusion/cuda/device_memory.hpp>

#include <vector>

namespace kf
{
    namespace cuda
    {
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Array class
          *
          * \note Typed container for GPU memory with reference counting.
          *
          * \author Anatoly Baksheev
          */
        template<class T>
        class KF_EXPORTS Array : public Memory
        {
        public:
            /** \brief Element type. */
            typedef T type;

            /** \brief Element size. */
            enum { elem_size = sizeof(T) };

            /** \brief Empty constructor. */
            Array();

            /** \brief Allocates internal buffer in GPU memory
              * \param size_t: number of elements to allocate
              * */
            Array(size_t size);

            /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
              * \param ptr: pointer to buffer
              * \param size: elemens number
              * */
            Array(T *ptr, size_t size);

            /** \brief Copy constructor. Just increments reference counter. */
            Array(const Array& other);

            /** \brief Assigment operator. Just increments reference counter. */
            Array& operator = (const Array& other);

            /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
              * \param size: elemens number
              * */
            void create(size_t size);

            /** \brief Decrements reference counter and releases internal buffer if needed. */
            void release();

            /** \brief Performs data copying. If destination size differs it will be reallocated.
              * \param other_arg: destination container
              * */
            void copyTo(Array& other) const;

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param host_ptr_arg: pointer to buffer to upload
              * \param size: elemens number
              * */
            void upload(const T *host_ptr, size_t size);

            /** \brief Downloads data from internal buffer to CPU memory
              * \param host_ptr_arg: pointer to buffer to download
              * */
            void download(T *host_ptr) const;

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param data: host vector to upload from
              * */
            template<class A>
            void upload(const std::vector<T, A>& data);

             /** \brief Downloads data from internal buffer to CPU memory
               * \param data:  host vector to download to
               * */
            template<typename A>
            void download(std::vector<T, A>& data) const;

            /** \brief Performs swap of data pointed with another device array.
              * \param other: device array to swap with
              * */
            void swap(Array& other_arg);

            /** \brief Returns pointer for internal buffer in GPU memory. */
            T* ptr();

            /** \brief Returns const pointer for internal buffer in GPU memory. */
            const T* ptr() const;

            //using Memory::ptr;

            /** \brief Returns pointer for internal buffer in GPU memory. */
            operator T*();

            /** \brief Returns const pointer for internal buffer in GPU memory. */
            operator const T*() const;

            /** \brief Returns size in elements. */
            size_t size() const;
        };


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Array2D class
          *
          * \note Typed container for pitched GPU memory with reference counting.
          *
          * \author Anatoly Baksheev
          */
        template<class T>
        class KF_EXPORTS Array2D : public Memory2D
        {
        public:
            /** \brief Element type. */
            typedef T type;

            /** \brief Element size. */
            enum { elem_size = sizeof(T) };

            /** \brief Empty constructor. */
            Array2D();

            /** \brief Allocates internal buffer in GPU memory
              * \param rows: number of rows to allocate
              * \param cols: number of elements in each row
              * */
            Array2D(int rows, int cols);

             /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
              * \param rows: number of rows
              * \param cols: number of elements in each row
              * \param data: pointer to buffer
              * \param stepBytes: stride between two consecutive rows in bytes
              * */
            Array2D(int rows, int cols, void *data, size_t stepBytes);

            /** \brief Copy constructor. Just increments reference counter. */
            Array2D(const Array2D& other);

            /** \brief Assigment operator. Just increments reference counter. */
            Array2D& operator = (const Array2D& other);

            /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
               * \param rows: number of rows to allocate
               * \param cols: number of elements in each row
               * */
            void create(int rows, int cols);

            /** \brief Decrements reference counter and releases internal buffer if needed. */
            void release();

            /** \brief Performs data copying. If destination size differs it will be reallocated.
              * \param other: destination container
              * */
            void copyTo(Array2D& other) const;

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param host_ptr: pointer to host buffer to upload
              * \param host_step: stride between two consecutive rows in bytes for host buffer
              * \param rows: number of rows to upload
              * \param cols: number of elements in each row
              * */
            void upload(const void *host_ptr, size_t host_step, int rows, int cols);

            /** \brief Downloads data from internal buffer to CPU memory. User is resposible for correct host buffer size.
              * \param host_ptr: pointer to host buffer to download
              * \param host_step: stride between two consecutive rows in bytes for host buffer
              * */
            void download(void *host_ptr, size_t host_step) const;

            /** \brief Performs swap of data pointed with another device array.
              * \param other: device array to swap with
              * */
            void swap(Array2D& other_arg);

            /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
              * \param data: host vector to upload from
              * \param cols: stride in elements between two consecutive rows for host buffer
              * */
            template<class A>
            void upload(const std::vector<T, A>& data, int cols);

            /** \brief Downloads data from internal buffer to CPU memory
               * \param data: host vector to download to
               * \param cols: Output stride in elements between two consecutive rows for host vector.
               * */
            template<class A>
            void download(std::vector<T, A>& data, int& cols) const;

            /** \brief Returns pointer to given row in internal buffer.
              * \param y_arg: row index
              * */
            T* ptr(int y = 0);

            /** \brief Returns const pointer to given row in internal buffer.
              * \param y_arg: row index
              * */
            const T* ptr(int y = 0) const;

            //using Memory2D::ptr;

            /** \brief Returns pointer for internal buffer in GPU memory. */
            operator T*();

            /** \brief Returns const pointer for internal buffer in GPU memory. */
            operator const T*() const;

            /** \brief Returns number of elements in each row. */
            int cols() const;

            /** \brief Returns number of rows. */
            int rows() const;

            /** \brief Returns step in elements. */
            size_t elem_step() const;
        };
    }
}

/////////////////////  Inline implementations of Array ////////////////////////////////////////////

template<class T> inline kf::cuda::Array<T>::Array() {}
template<class T> inline kf::cuda::Array<T>::Array(size_t size) : Memory(size * elem_size) {}
template<class T> inline kf::cuda::Array<T>::Array(T *ptr, size_t size) : Memory(ptr, size * elem_size) {}
template<class T> inline kf::cuda::Array<T>::Array(const Array& other) : Memory(other) {}
template<class T> inline kf::cuda::Array<T>& kf::cuda::Array<T>::operator=(const Array& other)
{ Memory::operator=(other); return *this; }

template<class T> inline void kf::cuda::Array<T>::create(size_t size)
{ Memory::create(size * elem_size); }
template<class T> inline void kf::cuda::Array<T>::release()
{ Memory::release(); }

template<class T> inline void kf::cuda::Array<T>::copyTo(Array& other) const
{ Memory::copyTo(other); }
template<class T> inline void kf::cuda::Array<T>::upload(const T *host_ptr, size_t size)
{ Memory::upload(host_ptr, size * elem_size); }
template<class T> inline void kf::cuda::Array<T>::download(T *host_ptr) const
{ Memory::download( host_ptr ); }

template<class T> void kf::cuda::Array<T>::swap(Array& other_arg) { Memory::swap(other_arg); }

template<class T> inline kf::cuda::Array<T>::operator T*() { return ptr(); }
template<class T> inline kf::cuda::Array<T>::operator const T*() const { return ptr(); }
template<class T> inline size_t kf::cuda::Array<T>::size() const { return sizeBytes() / elem_size; }

template<class T> inline       T* kf::cuda::Array<T>::ptr()       { return Memory::ptr<T>(); }
template<class T> inline const T* kf::cuda::Array<T>::ptr() const { return Memory::ptr<T>(); }

template<class T> template<class A> inline void kf::cuda::Array<T>::upload(const std::vector<T, A>& data) { upload(&data[0], data.size()); }
template<class T> template<class A> inline void kf::cuda::Array<T>::download(std::vector<T, A>& data) const { data.resize(size()); if (!data.empty()) download(&data[0]); }

/////////////////////  Inline implementations of Array2D ////////////////////////////////////////////

template<class T> inline kf::cuda::Array2D<T>::Array2D() {}
template<class T> inline kf::cuda::Array2D<T>::Array2D(int rows, int cols) : Memory2D(rows, cols * elem_size) {}
template<class T> inline kf::cuda::Array2D<T>::Array2D(int rows, int cols, void *data, size_t stepBytes) : Memory2D(rows, cols * elem_size, data, stepBytes) {}
template<class T> inline kf::cuda::Array2D<T>::Array2D(const Array2D& other) : Memory2D(other) {}
template<class T> inline kf::cuda::Array2D<T>& kf::cuda::Array2D<T>::operator=(const Array2D& other)
{ Memory2D::operator=(other); return *this; }

template<class T> inline void kf::cuda::Array2D<T>::create(int rows, int cols)
{ Memory2D::create(rows, cols * elem_size); }
template<class T> inline void kf::cuda::Array2D<T>::release()
{ Memory2D::release(); }

template<class T> inline void kf::cuda::Array2D<T>::copyTo(Array2D& other) const
{ Memory2D::copyTo(other); }
template<class T> inline void kf::cuda::Array2D<T>::upload(const void *host_ptr, size_t host_step, int rows, int cols)
{ Memory2D::upload(host_ptr, host_step, rows, cols * elem_size); }
template<class T> inline void kf::cuda::Array2D<T>::download(void *host_ptr, size_t host_step) const
{ Memory2D::download( host_ptr, host_step ); }

template<class T> template<class A> inline void kf::cuda::Array2D<T>::upload(const std::vector<T, A>& data, int cols)
{ upload(&data[0], cols * elem_size, data.size()/cols, cols); }

template<class T> template<class A> inline void kf::cuda::Array2D<T>::download(std::vector<T, A>& data, int& elem_step) const
{ elem_step = cols(); data.resize(cols() * rows()); if (!data.empty()) download(&data[0], colsBytes());  }

template<class T> void  kf::cuda::Array2D<T>::swap(Array2D& other_arg) { Memory2D::swap(other_arg); }

template<class T> inline       T* kf::cuda::Array2D<T>::ptr(int y)       { return Memory2D::ptr<T>(y); }
template<class T> inline const T* kf::cuda::Array2D<T>::ptr(int y) const { return Memory2D::ptr<T>(y); }

template<class T> inline kf::cuda::Array2D<T>::operator T*() { return ptr(); }
template<class T> inline kf::cuda::Array2D<T>::operator const T*() const { return ptr(); }

template<class T> inline int kf::cuda::Array2D<T>::cols() const { return Memory2D::colsBytes()/elem_size; }
template<class T> inline int kf::cuda::Array2D<T>::rows() const { return Memory2D::rows(); }

template<class T> inline size_t kf::cuda::Array2D<T>::elem_step() const { return Memory2D::step()/elem_size; }
