#include "rix/tf/frame_graph.hpp"

#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace tf {

FrameGraph::FrameGraph(const std::string &root, const rix::util::Duration &duration)
    : frames_(1, Frame(root, duration)), graph_(1), duration_(duration) {
    name_to_index_[root] = 0;
}

FrameGraph::FrameGraph(const FrameGraph &other)
    : graph_(other.graph_), frames_(other.frames_), name_to_index_(other.name_to_index_) {}

FrameGraph &FrameGraph::operator=(const FrameGraph &other) {
    if (this != &other) {
        graph_ = other.graph_;
        frames_ = other.frames_;
        name_to_index_ = other.name_to_index_;
        duration_ = other.duration_;
    }
    return *this;
}

bool FrameGraph::exists(const std::string &name) const { return name_to_index_.find(name) != name_to_index_.end(); }

FrameGraph::Iterator FrameGraph::get_root() const { return Iterator(*this, 0); }

/**< TODO: Implement the get_leaves method. */
std::vector<std::string> FrameGraph::get_leaves() const {
    std::vector<std::string> leaves;
    for (size_t i = 1; i < graph_.size(); ++i) {
        if (graph_[i].size() <= 1) {
            leaves.push_back(frames_[i].name);
        }
    }
    return leaves;
}

bool FrameGraph::update(const rix::msg::geometry::TF &tf) {
    for (const auto &transform : tf.transforms) {
        if (!update(transform)) return false;
    }
    return true;
}

/**< TODO: Implement the update method. */
bool FrameGraph::update(const rix::msg::geometry::TransformStamped &transform) {
    auto parent_it = name_to_index_.find(transform.header.frame_id);
    if (parent_it == name_to_index_.end()) {
        return false;
    }

    int parent_index = parent_it->second;
    if (graph_[parent_index].empty()) {
        int parent_parent = -1;
        if (parent_index != 0) {
            for (size_t i = 0; i < graph_.size(); ++i) {
                for (size_t j = 1; j < graph_[i].size(); ++j) {
                    if (graph_[i][j] == parent_index) {
                        parent_parent = static_cast<int>(i);
                        break;
                    }
                }
                if (parent_parent != -1) {
                    break;
                }
            }
        }
        graph_[parent_index].push_back(parent_parent);
    }

    int child_index = -1;
    auto child_it = name_to_index_.find(transform.child_frame_id);
    if (child_it == name_to_index_.end()) {
        child_index = static_cast<int>(frames_.size());
        frames_.emplace_back(transform.child_frame_id, duration_);
        graph_.emplace_back();
        graph_[child_index].push_back(parent_index);
        name_to_index_[transform.child_frame_id] = child_index;
    } else {
        child_index = child_it->second;
        int current_parent = graph_[child_index].empty() ? -1 : graph_[child_index][0];
        if (current_parent != parent_index) {
            return false;
        }
    }

    bool found_child = false;
    for (size_t i = 1; i < graph_[parent_index].size(); ++i) {
        if (graph_[parent_index][i] == child_index) {
            found_child = true;
            break;
        }
    }
    if (!found_child) {
        graph_[parent_index].push_back(child_index);
    }

    rix::util::Time time(transform.header.stamp);
    frames_[child_index].buffer.insert(time, transform.transform);
    return true;
}

/**< TODO: Implement the get_transform method. */
bool FrameGraph::get_transform(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                               rix::msg::geometry::TransformStamped &transform) const {
    if (target_frame == source_frame) {
        transform.header.frame_id = source_frame;
        transform.child_frame_id = target_frame;
        transform.header.stamp = time.to_msg();
        transform.transform = rix::robot::eigen_to_msg(Eigen::Affine3d::Identity());
        return true;
    }

    auto target_it = find(target_frame);
    if (target_it == end()) {
        return false;
    }
    auto source_it = find(source_frame);
    if (source_it == end()) {
        return false;
    }

    auto ancestor_it = find_nearest_ancestor(target_it, source_it);
    if (ancestor_it == end()) {
        return false;
    }

    auto build_path = [this](int node_index, int ancestor_index) {
        std::vector<int> path;
        if (node_index < 0 || ancestor_index < 0) {
            return path;
        }
        path.push_back(node_index);
        while (!path.empty() && path.back() != ancestor_index) {
            int current = path.back();
            int parent = graph_[current].empty() ? -1 : graph_[current][0];
            if (parent < 0) {
                path.clear();
                break;
            }
            path.push_back(parent);
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    auto target_path = build_path(target_it.index_, ancestor_it.index_);
    if (target_path.empty()) {
        return false;
    }
    auto source_path = build_path(source_it.index_, ancestor_it.index_);
    if (source_path.empty()) {
        return false;
    }

    auto compute_transform = [&](const std::vector<int> &path, Eigen::Affine3d &result) {
        result = Eigen::Affine3d::Identity();
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            int parent = path[i];
            int child = path[i + 1];
            rix::msg::geometry::Transform edge_tf;
            if (!frames_[child].buffer.get(time, edge_tf)) {
                return false;
            }
            result = result * rix::robot::msg_to_eigen(edge_tf);
        }
        return true;
    };

    Eigen::Affine3d target_tf;
    if (!compute_transform(target_path, target_tf)) {
        return false;
    }
    Eigen::Affine3d source_tf;
    if (!compute_transform(source_path, source_tf)) {
        return false;
    }

    Eigen::Affine3d final_tf = target_tf * source_tf.inverse();
    transform.header.frame_id = source_frame;
    transform.child_frame_id = target_frame;
    transform.header.stamp = time.to_msg();
    transform.transform = rix::robot::eigen_to_msg(final_tf);
    return true;
}

FrameGraph::Iterator FrameGraph::find(const std::string &name) const {
    auto it = name_to_index_.find(name);
    if (it != name_to_index_.end()) {
        return Iterator(*this, it->second);
    }
    return end();
}

/**< TODO: Implement the find_nearest_ancestor method. */
FrameGraph::Iterator FrameGraph::find_nearest_ancestor(Iterator frame_a, Iterator frame_b) const {
    if (frame_a == end() || frame_b == end()) {
        return end();
    }

    std::vector<bool> visited(frames_.size(), false);

    auto get_parent = [this](int index) {
        if (index < 0) {
            return -1;
        }
        if (index >= static_cast<int>(graph_.size()) || graph_[index].empty()) {
            return -1;
        }
        return graph_[index][0];
    };

    int idx = frame_a.index_;
    while (idx >= 0) {
        visited[idx] = true;
        idx = get_parent(idx);
    }

    idx = frame_b.index_;
    while (idx >= 0) {
        if (visited[idx]) {
            return Iterator(*this, idx);
        }
        idx = get_parent(idx);
    }

    return end();
}

FrameGraph::Iterator FrameGraph::find_nearest_ancestor(const std::string &frame_a, const std::string &frame_b) const {
    auto it_a = find(frame_a);
    if (it_a == end()) {
        return end();
    }
    auto it_b = find(frame_b);
    if (it_b == end()) {
        return end();
    }
    return find_nearest_ancestor(it_a, it_b);
}

FrameGraph::Iterator::Iterator(const Iterator &other) : graph_(other.graph_), index_(other.index_) {}

FrameGraph::Iterator &FrameGraph::Iterator::operator=(const Iterator &other) {
    if (this != &other) {
        index_ = other.index_;
    }
    return *this;
}

FrameGraph::Iterator FrameGraph::end() const { return Iterator(*this, -1); }

FrameGraph::Iterator::Iterator(const FrameGraph &graph, int index) : graph_(graph), index_(index) {}

FrameGraph::Iterator &FrameGraph::Iterator::operator--() {
    if (index_ > 0) {
        index_ = graph_.graph_[index_][0];
    }
    return *this;
}

FrameGraph::Iterator FrameGraph::Iterator::operator--(int) {
    FrameGraph::Iterator tmp = *this;
    --(*this);
    return tmp;
}

bool FrameGraph::Iterator::operator==(const Iterator &other) const {
    return index_ == other.index_ && &graph_ == &other.graph_;
}
bool FrameGraph::Iterator::operator!=(const Iterator &other) const { return !(*this == other); }
const Frame &FrameGraph::Iterator::operator*() const { return graph_.frames_[index_]; }
const Frame *FrameGraph::Iterator::operator->() const { return &graph_.frames_[index_]; }
bool FrameGraph::Iterator::operator<(const Iterator &other) const { return index_ < other.index_; }
bool FrameGraph::Iterator::operator>(const Iterator &other) const { return index_ > other.index_; }
bool FrameGraph::Iterator::operator<=(const Iterator &other) const { return index_ <= other.index_; }
bool FrameGraph::Iterator::operator>=(const Iterator &other) const { return index_ >= other.index_; }

}  // namespace tf
}  // namespace rix