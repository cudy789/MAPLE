#pragma once

#include "Worker.h"
#include "Pose.h"

#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

/**
 * @brief The threaded NetworkTables worker periodically retrieves the latest robot pose from the LocalizationWorker
 * and publishes the global frame pose data to NetworkTables. By default, the worker will attempt to connect to the
 * NetworkTables server at 10.TE.AM.2, where TEAM is the FRC team number. You may also specify a hostname to
 * connect to instead, i.e. 127.0.0.1, 192.168.1.213, etc..
 *
 * Other clients connected to the same NetworkTables server will see data being published at:
 * * MAPLE/location: double[pos_x, pos_y, pos_z] in meters
 * * MAPLE/orientation: double[roll, pitch, yaw] in degrees
 * * MAPLE/robot_relative_tagid: int[tagid1, tagid2, ...]
 * * MAPLE/robot_relative_x: double[tag1_dist_x, tag2_dist_x, ...]
 * * MAPLE/robot_relative_y: double[tag1_dist_y, tag2_dist_y, ...]
 * * MAPLE/robot_relative_yaw: double[tag1_angle_yaw, tag2_angle_yaw, ...]
 *
 * Compatible with NetworkTables 4.
 */
class NTWorker: public Worker{

public:
    /**
     * @brief Create the worker, default NetworkTables client connection to 127.0.0.1. Setup the publishers to position
     * and orientation.
     */
    NTWorker();
    /**
     * @brief Create the worker, connect to NetworkTables via FRC team number.
     * @param team_num The FRC team number.
     */
    NTWorker(int team_num);
    /**
     * @brief Create the worker, connect to NetworkTables via hostname string.
     * @param hostname The hostname with the NetworkTables server running.
     */
    NTWorker(std::string hostname);

    /**
     * @brief Create the worker, connect to NetworkTables via hostname string.
     * @param hostname The hostname with the NetworkTables server running.
     * @param ntport4 The port which the NetworkTables 4 server is running on.
     */
    NTWorker(const std::string& hostname, int ntport4);

    /**
     * @brief Register the pose callback function that is periodically called to retrieve the latest pose data to publish
     * to NetworkTables.
     * @param pose_callback The callback function which returns a RobotPose object to be parsed and published to NetworkTables.
     */
    void RegisterPoseCallback(const std::function<RobotPose()>& pose_callback);
    /**
     * @brief Check if the worker is connected to the NetworkTables server.
     * @return true if connected to a NetworkTables server, false otherwise.
     */
    bool IsConnected();

private:
    /**
     * @brief Start the NT4 client using either the team number if present, otherwise hostname. If the client fails to
     * connect, wait 10 seconds, then restart the thread.
     */
    void Init() override;
    /**
     * @brief Execute pose callback function to obtain the latest robot pose estimate, parse into the two NetworkTables
     * topics, then publish the values.
     */
    void Execute() override;

    int _team_num = -1;
    std::string _hostname = "127.0.0.1";
    int _ntport4 = NT_DEFAULT_PORT4;

    nt::NetworkTableInstance _nt_instance;
    std::shared_ptr<nt::NetworkTable> _nt_table;

    nt::DoubleArrayPublisher _position;
    nt::DoubleArrayPublisher _orientation;

    nt::IntegerArrayPublisher _robot_relative_tagid;
    nt::DoubleArrayPublisher _robot_relative_x;
    nt::DoubleArrayPublisher _robot_relative_y;
    nt::DoubleArrayPublisher _robot_relative_z;
    nt::DoubleArrayPublisher _robot_relative_yaw;
    nt::IntegerArrayPublisher _camera_relative_tagid;
    nt::DoubleArrayPublisher _camera_relative_x;
    nt::DoubleArrayPublisher _camera_relative_y;
    nt::DoubleArrayPublisher _camera_relative_z;
    nt::DoubleArrayPublisher _camera_relative_yaw;

    std::function<RobotPose()> _pose_callback;
};