#include "kdtree.h"

namespace kdtree {

adaptor::adaptor(std::vector<dbscan::Point>& points)
    : m_points(points)
{
}

size_t adaptor::kdtree_get_point_count() const
{
    return m_points.size();
}

float adaptor::kdtree_get_pt(const size_t index, const size_t dim) const
{
    switch (dim) {
    case 0:
        return m_points[index].x();
    case 1:
        return m_points[index].y();
    default:
        return m_points[index].z();
    }
}

std::array<float, 3> get_query_point(std::vector<dbscan::Point>& points, size_t index)
{
    return std::array<float, 3>({ (float)points[index].x(),
        (float)points[index].y(), (float)points[index].z() });
}

std::vector<dbscan::Point> collect_points_with_indices(std::vector<size_t>& indices, std::vector<dbscan::Point>& points)
{
    std::vector<dbscan::Point> collected_points = {};
    for (size_t index : indices) {
        collected_points.push_back(points[index]);
    }
    return collected_points;
}

std::vector<dbscan::Cluster> cluster_points(
    std::vector<dbscan::Point>& points, float eps, int min_pts)
{
    eps *= eps;
    const auto adapt = adaptor(points);
    using index_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, decltype(adapt)>, decltype(adapt),
        3>;

    index_t indexAdaptor(
        3, adapt, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    indexAdaptor.buildIndex();

    auto visited = std::vector<bool>(points.size());
    std::vector<dbscan::Cluster> clusters;
    auto matches = std::vector<std::pair<uint32_t, float>>();
    auto sub_matches = std::vector<std::pair<uint32_t, float>>();

    for (size_t i = 0; i < points.size(); i++) {
        if (visited[i])
            continue;

        indexAdaptor.radiusSearch(get_query_point(points, i).data(), eps,
            matches, nanoflann::SearchParams(32, 0.f, false));
        if (matches.size() < static_cast<size_t>(min_pts))
            continue;
        visited[i] = true;

        std::vector<size_t> cluster = { i };

        while (!matches.empty()) {
            auto nb_idx = matches.back().first;
            matches.pop_back();
            if (visited[nb_idx])
                continue;
            visited[nb_idx] = true;

            indexAdaptor.radiusSearch(get_query_point(points, nb_idx).data(),
                eps, sub_matches, nanoflann::SearchParams(32, 0.f, false));

            if (sub_matches.size() >= static_cast<size_t>(min_pts)) {
                std::copy(sub_matches.begin(), sub_matches.end(),
                    std::back_inserter(matches));
            }
            cluster.push_back(nb_idx);
        }
        dbscan::Cluster new_cluster(collect_points_with_indices(cluster, points));
        clusters.push_back(new_cluster);
    }
    return clusters;
}

} // namespace kdtree
