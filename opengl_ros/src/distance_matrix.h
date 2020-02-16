#ifndef DISTANCE_MATRIX_H
#define DISTANCE_MATRIX_H

#include <functional>
#include <Eigen/Core>

template<class T>
class DistanceMatrix final
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data_;

public:
    DistanceMatrix(int length)
        : data_(length, length)
    {
    }

    int length() const { return length; }

    T get(int x, int y) const { return data_(x, y); }

    template<class CONTAINER, class POINT>
    void update(const CONTAINER& container, std::function<T(const POINT&, const POINT&)> distance_calculator)
    {
        for (int i = 0; i < container.size(); ++i)
            for (int j = i; j < container.size(); ++j)
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