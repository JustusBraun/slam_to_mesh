#pragma once

#include <lvr2/types/PointBuffer.hpp>
#include <lvr2/geometry/BaseVector.hpp>

namespace detail
{

class PointBufferIterator
{
    
    class ElementProxy
    {
    public:
        using Vector = lvr2::BaseVector<float>;

        ElementProxy(float* ptr)
        : ptr_(ptr)
        {
            assert(nullptr != ptr);
        }

        ElementProxy() = delete;
        ElementProxy(const ElementProxy& other) = delete;

        inline operator Vector() const
        {
            return Vector(ptr_[0], ptr_[1], ptr_[2]);
        }
        
        /// Assigning a Vector is transparent
        inline ElementProxy& operator=(const Vector& vec)
        {
            ptr_[0] = vec.x;
            ptr_[1] = vec.y;
            ptr_[2] = vec.z;
            return *this;
        }

        /// Assigning an element proxy to another should
        /// behave as direct assignment between the elements
        inline ElementProxy& operator=(const ElementProxy& other)
        {
            return (*this = (Vector) other);
        }
        
        float* ptr_;
    };

public:
    /// Iterator traits
    using difference_type = std::ptrdiff_t;
    using value_type = lvr2::BaseVector<float>;
    using pointer = void;
    using reference = ElementProxy;
    using iterator_category = std::forward_iterator_tag;

    PointBufferIterator(float* ptr)
    : ptr_(ptr)
    {}

    bool operator!=(const PointBufferIterator& other) const
    {
        return ptr_ != other.ptr_;
    }

    PointBufferIterator& operator++()
    {
        ptr_ += 3;
        return *this;
    }

    ElementProxy operator*()
    {
        return ElementProxy(ptr_);
    }

private:
    float* ptr_;
};

} // namespace detail
