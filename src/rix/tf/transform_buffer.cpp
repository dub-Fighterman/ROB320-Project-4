#include "rix/tf/transform_buffer.hpp"

#include <Eigen/Geometry>
#include <algorithm>

#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace tf {

TransformBuffer::TransformBuffer() {}

TransformBuffer::TransformBuffer(const rix::util::Duration &duration) : duration_(duration) {}

TransformBuffer::TransformBuffer(const TransformBuffer &other) : duration_(other.duration_), buffer_(other.buffer_) {}

TransformBuffer &TransformBuffer::operator=(const TransformBuffer &other) {
    if (this != &other) {
        duration_ = other.duration_;
        buffer_ = other.buffer_;
    }
    return *this;
}

size_t TransformBuffer::size() const { return buffer_.size(); }

bool TransformBuffer::empty() const { return buffer_.empty(); }

void TransformBuffer::clear() { buffer_.clear(); }

/*< TODO: Implement the insert method. */
void TransformBuffer::insert(const rix::util::Time &time, const rix::msg::geometry::Transform &transform) {
    auto insert_pos = std::lower_bound(
        buffer_.begin(), buffer_.end(), time,
        [](const std::pair<rix::util::Time, rix::msg::geometry::Transform> &entry, const rix::util::Time &t) {
            return entry.first < t;
        });

    if (insert_pos != buffer_.end() && insert_pos->first == time) {
        insert_pos->second = transform;
    } 
    else {
        buffer_.insert(insert_pos, std::make_pair(time, transform));
    }

    if (buffer_.empty()) { 
        return; 
    }

    if (duration_.to_nanoseconds() <= 0) { 
        return; 
    }

    const auto latest_time = buffer_.back().first;
    const auto min_time = latest_time - duration_;

    while (!buffer_.empty() && buffer_.front().first < min_time) {
        buffer_.pop_front();
    }
}

/*< TODO: Implement the get method. */
bool TransformBuffer::get(const rix::util::Time &time, rix::msg::geometry::Transform &transform) const {
    if (buffer_.empty()) { return false; }

    if (time.to_nanoseconds() == 0) {
        transform = buffer_.back().second;
        return true;
    }

    if (time <= buffer_.front().first) {
        transform = buffer_.front().second;
        return true;
    }

    if (time >= buffer_.back().first) {
        transform = buffer_.back().second;
        return true;
    }

    auto upper = std::lower_bound(
        buffer_.begin(), buffer_.end(), time,
        [](const std::pair<rix::util::Time, rix::msg::geometry::Transform> &entry, const rix::util::Time &t) {
            return entry.first < t;
        });

    if (upper != buffer_.end() && upper->first == time) {
        transform = upper->second;
        return true;
    }

    if (upper == buffer_.begin()) {
        transform = upper->second;
        return true;
    }

    auto lower = upper;
    --lower;

    rix::util::Duration total_duration = upper->first - lower->first;
    if (total_duration.to_nanoseconds() <= 0) {
        transform = lower->second;
        return true;
    }

    rix::util::Duration elapsed = time - lower->first;
    double ratio = static_cast<double>(elapsed.to_nanoseconds()) /
                   static_cast<double>(total_duration.to_nanoseconds());
    ratio = std::clamp(ratio, 0.0, 1.0);

    transform = rix::robot::interpolate(lower->second, upper->second, ratio);
    return true;
}

const std::deque<std::pair<rix::util::Time, rix::msg::geometry::Transform>> &TransformBuffer::data() const {
    return buffer_;
}

rix::util::Duration TransformBuffer::duration() const { return duration_; }

void TransformBuffer::set_duration(const rix::util::Duration &duration) { duration_ = duration; }

}  // namespace tf
}  // namespace rix