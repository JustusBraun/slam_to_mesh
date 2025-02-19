#pragma once

#include <boost/variant.hpp>
#include <lvr2/types/VariantChannel.hpp>


/**
 *  @brief Creates a channel with the same type, width and name in a different buffer.
 *
 *  This is used to create the output buffers in \ref combine_pointclouds
 */
template <typename VariantChannelT>
struct CreateSameTypeChannelWithSize
: public boost::static_visitor<VariantChannelT>
{
    CreateSameTypeChannelWithSize(size_t size)
    : size_(size)
    {}

    template <typename T>
    VariantChannelT operator()(const lvr2::Channel<T>& channel) const
    {
        return lvr2::Channel<T>(size_, channel.width());
    }

private:
    size_t size_;
};

/**
 *  @brief Copy a channel to another.
 *
 *  Assumes that both channels are of the same type and that the target channel has enough space
 */
template<typename VariantChannelT>
struct CopyChannel
: public boost::static_visitor<size_t>
{
    CopyChannel(VariantChannelT& dest, size_t start_idx = 0)
    : target_(&dest)
    , begin_(start_idx)
    {}
    
    template <typename T>
    size_t operator()(const lvr2::Channel<T>& from) const
    {
        lvr2::Channel<T>& out = boost::get<lvr2::Channel<T>>(*target_);
        std::copy_n(
            from.dataPtr().get(),
            from.numElements() * from.width(),
            out.dataPtr().get() + begin_ * from.width()
        );

        return begin_ + from.numElements();
    }

private:
    VariantChannelT* target_;
    size_t begin_;
};


/**
 *  @brief Copy a channel to another removing elements for which the condition iterator returns true.
 *
 *  Assumes that both channels are of the same type and that the target channel has enough space.
 *
 */
template<typename VariantChannelT, typename ForwardIteratorT>
struct RemoveCopyIf
: public boost::static_visitor<size_t>
{
    RemoveCopyIf(VariantChannelT& dest, const ForwardIteratorT& condition)
    : target_(&dest)
    , cond_(condition)
    {}
    
    template <typename T>
    size_t operator()(const lvr2::Channel<T>& from) const
    {
        lvr2::Channel<T>& out = boost::get<lvr2::Channel<T>>(*target_);
        size_t out_i = 0;
        auto cond = cond_; // Copy because this function is const
        for (size_t src_i = 0; src_i < from.numElements(); src_i++, cond++)
        {
            if (*cond)
            {
                continue;
            }
            
            std::copy_n(
                from.dataPtr().get() + from.width() * src_i,
                from.width(),
                out.dataPtr().get() + from.width() * out_i
            );
            out_i++;
        }

        return out_i;
    }

private:
    VariantChannelT* target_;
    ForwardIteratorT cond_;
};
