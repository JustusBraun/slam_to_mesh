#pragma once

#include <lvr2/types/PointBuffer.hpp>
#include <lvr2/geometry/BaseVector.hpp>

#include <slam_to_mesh/detail/iterator.hpp>

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

    detail::PointBufferIterator begin()
    {
        return detail::PointBufferIterator(buf_.get());
    }

    detail::PointBufferIterator end()
    {
        return detail::PointBufferIterator(buf_.get() + size_ * 3);
    }

private:

    lvr2::floatArr buf_;
    size_t size_;
};
