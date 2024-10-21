#ifndef KDTREE_H
#define KDTREE_H

#include <array>
#include <vector>
#include "nanoflann.hpp"
#include "point.h"
#include "cluster.h"

namespace kdtree {

struct adaptor {
    std::vector<dbscan::Point> m_points;

    explicit adaptor(std::vector<dbscan::Point>& points);
    
    [[nodiscard]] size_t kdtree_get_point_count() const;
    [[nodiscard]] float kdtree_get_pt(const size_t index, const size_t dim) const;

    template <class BBOX> bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

std::array<float, 3> get_query_point(std::vector<dbscan::Point>& points, size_t index);

std::vector<dbscan::Point> collect_points_with_indices(std::vector<size_t>& indices, std::vector<dbscan::Point>& points);

std::vector<dbscan::Cluster> cluster_points(std::vector<dbscan::Point>& points, float eps, int min_pts);

} // namespace kdtree

#endif // KDTREE_H
