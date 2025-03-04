#pragma once

#include <chrono>
#include <iostream>
#include <Eigen>

#include "TimeUtils.h"
#include "MatrixHelpers.h"
#include "Logger.h"

inline int STALE_TAG_MS = 50; // If a tag hasn't been seen in this amount of time, then assume we no longer see the tag and clear values
inline size_t NUM_TAG_IDS = 25; // Total number of tag IDs expected

/**
 * @brief A single Pose with a translation vector and rotation matrix. Includes arithmetic and ostream operator overloads.
 */
struct Pose_single {
    // Translation (x,y,z) and rotation (rotation matrix RPY)
    Eigen::Vector3d T = Eigen::Vector3d::Constant(0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Constant(0);

    Pose_single operator-(const Pose_single &o){
        Pose_single diff;

        diff.T = T - o.T;
        diff.R = R * o.R;

        return diff;
    }
    Pose_single operator+(const Pose_single &o){
        Pose_single sum;

        sum.T = T + o.T;
        sum.R = o.R * R;

        return sum;
    }
    template<typename T>
    Pose_single operator*(T val) {
        Pose_single prod;

        prod.T = this->T * val;
        prod.R = this->R * val;

        return prod;
    }

    template<typename T>
    Pose_single operator/=(T val) {
        this->T /= val;
        this->R /= val;
        return *this;
    }

    Pose_single &operator+=(const Pose_single &o) {
        this->T += o.T;
        this->R += o.R;

        return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const Pose_single& p) {
        os << "XYZ: " << std::fixed << std::setprecision(4) << p.T
           << " RPY: " << RotationMatrixToRPY(p.R);

        return os;
    }
};

/**
 * @brief A Pose object is created from a single Apriltag detection. Each Pose objecet contains four separate Pose_single
 * objects, one for each frame of reference between the Apriltag and the robot. We prefer using extra memory
 * instead of recomputing each of the four frames when we need them. The associated camera and tag ID is recorded, as well
 * as the calculated error for the tag detection.
 */
struct Pose{
    // The tag this pose originated from
    int tag_id = -1;
    // The camera that detected this tag
    int cam_id = -1;
    // The time this pose object was created at
    ulong time = CurrentTime();

    // The center of the detection in image pixel coordinates.
    double c[2];

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    double p[4][2];

    // The pose of the object relative to the AprilTag, camera, robot, and the global field coordinates
    Pose_single tag;
    Pose_single camera;
    Pose_single robot;
    Pose_single global;

    // The error of the tag detection
    double err;

    friend std::ostream& operator<<(std::ostream& os, const Pose& o_p) {
        os << "tag_id: " << o_p.tag_id << ", " << "cam_id: " << o_p.cam_id << ", err: " << std::fixed << std::setprecision(4) << o_p.err
           << ", GLOBAL_FRAME: " << o_p.global << ", ROBOT_FRAME: " << o_p.robot << ", CAM_FRAME: " << o_p.camera;
        return os;
    }

    Pose &operator+=(const Pose &o) {
        this->tag += o.tag;
        this->camera += o.camera;
        this->robot += o.robot;
        this->global += o.global;
        this->err += o.err;
        return *this;
    }

    template<typename T>
    Pose operator/=(T val) {
        this->tag /= val;
        this->camera /= val;
        this->robot /= val;
        this->global /= val;
        this->err /= val;
        return *this;
    }

};

/**
 * @brief A data structure that organizes Poses based on tag_ids. Includes methods to add tags, clear stale tags and remove
 * all tags. An ordered vector of NUM_TAG_IDS represents each of the possible tag_ids that can be detected, then for
 * each tag_id, an inner vector contains all of the Poses that were detected for this tag_id. The size of the outer vector
 * data is NUM_TAG_IDS, so to get the Poses for a tag with ID i, we must access the vector at index i-1 (data[i-1]).
 */
struct TagArray{
    /**
     * @brief Create an empty TagArray of size NUM_TAG_IDS.
     */
    explicit TagArray(){
        _num_tags = 0;
        data = std::vector<std::vector<Pose>>{NUM_TAG_IDS};
    }
    /**
     * @brief Remove all tags in the TagArray object.
     */
    void ClearAll(){
        _num_tags = 0;
        data = std::vector<std::vector<Pose>>{NUM_TAG_IDS};
    }
    /**
     * @brief Add a tag to the data structure at the tag_id-1 index in data.
     * @param tag The tag to be added
     */
    void AddTag(const Pose& tag){
        data[tag.tag_id-1].push_back(tag);
        _num_tags++;
    }

    /**
     * @brief Get the number of tags in the data structure.
     * @return The number of tags.
     */
    const int GetNumTags(){
        return _num_tags;
    }

    /**
     * @brief Remove all tags that are older than 150ms.
     * @return The number of stale tags removed.
     */
    int ClearStale(){
        int stale_tags = 0;
        for (std::vector<Pose>& v: data){
            for (auto it=v.begin(); it!=v.end();){
                if ((CurrentTime() -  (*it).time) > STALE_TAG_MS * 1.0e6){
                    it = v.erase(it);
                    stale_tags++;
                } else{
                    it++;
                }
            }
        }
        if (stale_tags > _num_tags){
            AppLogger::Logger::Log("ERROR, more stale tags than total number of tags. stale_tags=" + std::to_string(stale_tags) + ", _num_tags=" + std::to_string(_num_tags), AppLogger::SEVERITY::ERROR);
            exit(1);

        }
        _num_tags -= stale_tags;
        return stale_tags;
    }

    std::vector<std::vector<Pose>> data;
    size_t _num_tags = 0;
};

/**
 * @brief A Pose object that describes the robots location in the global frame. Only the global Pose_single object has
 * valid pose data.
 */
struct RobotPose: public Pose {
    RobotPose() = default;

    TagArray RelativeTags;

    explicit RobotPose(const Pose_single& global_pose) {
        this->global = global_pose;
    }

    friend std::ostream& operator<<(std::ostream& os, const RobotPose& o_p) {
        os << o_p.global;
        return os;
    }

};