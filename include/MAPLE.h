#pragma once

// Disable debug stdout for Apriltag library
#define NODEBUG 1

#include <string>
#include <vector>
#include <csignal>

#include "LocalizationWorker.h"
#include "WebServerWorker.h"
#include "TDCamWorker.h"
#include "NTWorker.h"
#include "ConfigParser.h"
#include "Logger.h"


/**
 * @brief The Multicamera Apriltag Pose Localization and Estimation (MAPLE) class is a singleton object with with individual Worker
 * threads for each camera, the localization computation, the webserver, the NetworkTables client, and the logger.
 *
 * First, the config.yml configuration file is read to determine MAPLEParams and setup the TDCamWorker Workers.
 * Next, the .fmap file is parsed to configure global locations of the Apriltags.
 * All Worker threads are spun up and process data in realtime. Most Worker threads are marked as 'stay alive', which
 * will attempt to restart the Worker in case of an exception. MAPLE will only exit on a CTRL-C interrupt.
 *
 * The latest robot pose in the global frame is available using GetRobotPose(), and is measured in position x,y,z in
 * meters, and orientation roll, pitch, yaw in degrees.
 *
 */
class MAPLE{

public:
    /**
     * @brief No copy constructor allowed for the singleton.
     */
    MAPLE (MAPLE const&) = delete;

    /**
     * @brief Return the singleton instance of the MAPLE object. Create a new static instance if none exists.
     */
    static MAPLE& GetInstance();

    /**
     * @brief Read configuration parameters from the specified config yaml file. Create all Worker threads after
     * files are loaded.
     * @param config_file The .yaml configuration file path, relative to MAPLE.cpp
     */
    void Setup(const std::string& config_file);

    /**
     * @brief Start all Worker threads, which starts the MAPLE system.
     */
    void Start();

    /**
     * @brief Calibrate all cameras by finding the distortion coefficients using a 7x7 checkerboard image. Distortion
     * coefficients are not directly applied to the cameras, the user must copy the parameters from stdout or the logfile.
     * Block until calibration is complete.
     */
    void Calibrate();

    /**
     * @brief Get the latest RobotPose estimate. The estimate depends on which localization strategy is used.
     * @return A RobotPose object with the global pose position estimate.
     */
    RobotPose GetRobotPose();

    /**
     * @brief Wait for all Workers to finish. Blocks until CTRL-C is received or Stop() is called.
     */
    void Join();

    /**
     * @brief Stop all Workers gracefully.
     */
    void Stop();

    /**
     * @brief Register sigint callback to Workers
     * @param signum The signal value caught
     */
    static void StaticSignalCallback(int signum);

    /**
     * @brief The signal callback within the context of the singleton instance which registers the callback function
     * to each thread.
     * @param signum The signal value caught
     */
    void SignalCallback(int signum);

    /**
     * @brief Restart MAPLE.
     */
    void Restart();

    /**
     * @brief Get MAPLE version.
     * @return The version.
     */
    std::string GetVersion();

private:
    MAPLE() = default;

    LocalizationWorker* _l_w;
    std::vector<Worker*> _workers_t;
    std::vector<Worker*> _cam_workers_t;

    Worker* _w_w;

    MAPLEParams _params;

    std::string _config_file;

    std::string _version;

    bool _is_finished = false;
    std::binary_semaphore _is_finished_sem{1};

    void SetupHelper(const std::string &config_file);

    bool _restart_requested = false;

};