#ifndef DISTANCE_MATRIX_H
#define DISTANCE_MATRIX_H

#include <functional>
#include <Eigen/Core>

template<class T>
class DistanceMatrix final
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data_;
    int length_ = 0;

public:
    DistanceMatrix(int buffer_length)
        : data_(buffer_length, buffer_length)
    {
    }

    //returns the actual size of the matrix
    int length() const { return length_; }

    T get(int x, int y) const { return data_(x, y); }

    template<class CONTAINER, class POINT>
    void update(const CONTAINER& container, std::function<T(const POINT&, const POINT&)> distance_calculator)
    {
        length_ = container.size();
        if(data_.rows() < length_)
        {
            ROS_WARN_STREAM("Distance matrix size buffer is smaller than input container. First " << data_.rows() << " elements will be used.");
            length_ = data_.rows();
        }

        for (int i = 0; i < length_; ++i)
            for (int j = i; j < length_; ++j)
            {
                data_(i, j) = data_(j, i) = distance_calculator(container[i], container[j]);
            }
    }
    
    //no copy
    DistanceMatrix(const DistanceMatrix&) = delete;
    DistanceMatrix& operator=(const DistanceMatrix&) = delete;

    //default move
    DistanceMatrix(DistanceMatrix&&) = default;
    DistanceMatrix& operator=(DistanceMatrix&&) = default;
};

#endif