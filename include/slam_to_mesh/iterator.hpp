#pragma once

#include <lvr2/types/PointBuffer.hpp>
#include <lvr2/geometry/BaseVector.hpp>

class PointBufferIterator
{
    
    class ElementProxy
    {
    public:
        ElementProxy(float* ptr)
        : ptr_(ptr)
        {}

        operator lvr2::BaseVector<float>() const
        {
            return lvr2::BaseVector<float>(ptr_[0], ptr_[1], ptr_[2]);
        }

        ElementProxy& operator=(const lvr2::BaseVector<float>& vec)
        {
            ptr_[0] = vec.x;
            ptr_[1] = vec.y;
            ptr_[2] = vec.z;
            return *this;
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


class PointBufferRange
{
public:
    PointBufferRange(lvr2::PointBuffer& buffer)
    : buf_(buffer.getPointArray())
    , size_(buffer.numPoints())
    {}

    PointBufferRange(lvr2::floatArr arr, size_t n)
    : buf_(arr)
    , size_(n)
    {}

    PointBufferIterator begin()
    {
        return PointBufferIterator(buf_.get());
    }

    PointBufferIterator end()
    {
        return PointBufferIterator(buf_.get() + size_ * 3);
    }

private:

    lvr2::floatArr buf_;
    size_t size_;
};
