#pragma once

#include <map>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen>

#include "Pose.h"
#include "Logger.h"

using json = nlohmann::json;

/**
 * @brief The Apriltag layout .fmap parser. Read the global tag positions from the .fmap file to create a std::map<int, Pose_single>
 * object, which maps Apriltag IDs to their global position on field. All tag IDs in the .fmap file should be unique.
 *
 * All poses are in degrees and meters, with the origin 0,0,0 being at the center of the field. From the red alliance, the
 * front of the tag is visible, with the X axis towards the red alliance, Y axis right, and Z axis up.
 *
 * Use the Limelight tools map builder tool to create
 * a .fmap file from scratch - https://tools.limelightvision.io/map-builder, or download an existing file.
 */
class TagLayoutParser{
public:
    /**
     * @brief No instantiation of this class, only used for its ParseConfig function.
     */
    TagLayoutParser() = delete;
    /**
     * @brief No instantiation of this class, only used for its ParseConfig function.
     */
    void operator=(TagLayoutParser const&) = delete;
    /**
     * @brief Parse the configuration file for tags. All tags should have unique ids, maximum of 25 tags.
     * @param cfg_file The path to the .fmap configuration file relative to the TagLayoutParser.h file.
     * @return The mapping between tag IDs and their location on the field in the global field frame.
     */
    static std::map<int, Pose_single> ParseConfig(const std::string& cfg_file){
        std::map<int, Pose_single> tag_layout;

        try {
            std::ifstream f(cfg_file);

            json json_tags = json::parse(f);

            for (const auto& item: json_tags["fiducials"]){
                std::string family = item["family"];
                int id = item["id"].get<int>();
                double size = item["size"].get<double>();
                int unique = item["unique"].get<int>();
                std::vector<double> transform = item["transform"].get<std::vector<double>>();

                // 4x4 transform: interpreted as row-major [R|t]; rows 0-2 are rotation, col 3 is translation.
                // If your .fmap uses column-major (e.g. OpenGL-style), swap to R_AG(i,j)=transform[j*4+i], T_AG=transform[12,13,14].
                Eigen::Vector3d T_AG{transform[3], transform[7], transform[11]};
                Eigen::Matrix3d R_AG{{transform[0], transform[1], transform[2]},
                                     {transform[4], transform[5], transform[6]},
                                     {transform[8], transform[9], transform[10]}};

                Pose_single new_pose;
//                new_pose.R = CreateRotationMatrix(RotationMatrixToRPY(R_AG) + Eigen::Vector3d{0,0,180});
                new_pose.R = R_AG;
                new_pose.T = T_AG;

                tag_layout.insert({id, new_pose});

                AppLogger::Logger::Log("Adding tag " + to_string(new_pose) + " from .fmap file", AppLogger::SEVERITY::INFO);
            }

        } catch (const std::exception& e) {
            AppLogger::Logger::Log("Error parsing JSON file: " + std::string(e.what()), AppLogger::SEVERITY::ERROR);
            throw(e);
        }

        return tag_layout;
    }


};

