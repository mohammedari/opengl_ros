#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <deque>
#include "distance_matrix.h"

class DBSCAN final
{
    double epsilon_;
    int min_points_;

    std::vector<int> find_points_in_epsilon(const DistanceMatrix& distances, int i) const
    {
        std::vector<int> points_in_epsilon;
        for (int j = 0; j < distances.length(); ++j)
        {
            if (distances.get(i, j) < epsilon_)
                points_in_epsilon.push_back(j)
        }

        return points_in_epsilon;
    }

public: 
    DBSCAN(double epsilon, int min_points)
        : epsilon_(epsilon), min_points_(min_points)
    {
    }

    std::vector<std::vector<int>> cluster(const DistanceMatrix& distances) const
    {
        std::vector<bool> is_clustered(distances.length(), false);
        std::vector<std::vector<int> clusters;
        for (int i = 0; i < distances.length(); ++i)
        {
            if (is_clustered[i])
                continue;

            //find points in epsilon distance
            //this list includes the point itself
            auto first_points = find_points_in_epsilon(distances, i);
            if (first_points.size() < min_points_)
                continue;

            //create a first cluster
            clusters.emplace_back();
            auto& current_cluster = clusters.back();

            //start clustering
            std::deque<int> neighboring_points(first_points.cbegin(), first_points.cend());
            while (0 < neighboring_points.size())
            {
                auto front = neighboring_points.front();
                neighboring_points.pop_front();

                if (is_clustered[front])
                    continue;

                is_clustered[front] = true;
                current_cluster.push_back(front);

                //add new neighboring points to the queue
                auto new_neighbors = find_points_in_epsilon(distances, front);
                for (const auto& new_neighbor : new_neighbors)
                    neighboring_points.push_back(new_neighbor);
            }
        }

        return clusters;
    }

    //default copy
    DBSCAN(const DBSCAN&) = default;
    DBSCAN& operator=(const DBSCAN&) = default;

    //default move
    DBSCAN(DBSCAN&&) = default;
    DBSCAN& operator=(DBSCAN&&) = default;
};

#endif