#pragma once

#include <thread>
#include <bits/stdc++.h>

#include "TDCam.h"
#include "Worker.h"

/**
 * @brief The threaded tag detector camera worker periodically polls its capture device for new camera frames, detects
 * Apriltags in the frame, then passes the Apriltag pose detections to the LocalizationWorker class via a callback function.
 * This worker also provides access to an annotated image with bounding boxes and tag IDs displayed around detected Apriltags
 * in the latest camera frame.
 */
class TDCamWorker: public Worker, public TDCam {
public:
    /**
     * @brief No default constructor.
     */
    TDCamWorker() = delete;

    /**
     * @brief Call the Worker and TDCam constructors, set execution frequency to 50hz and enable stay_alive. Register
     * the tag detection callback function, and set flag to enable/disable debug cv ImShow.
     * @param c_params The camera configuration to use during setup and computations.
     * @param tag_layout The Apriltag field layout to use during computations.
     * @param queue_tags_callback Pass all Apriltag pose detections in the frame through this callback function.
     * @param record_video Flag to enable/disable video recording.
     */
    TDCamWorker(CamParams& c_params, const std::map<int, Pose_single>& tag_layout, std::function<bool(TagArray&)> queue_tags_callback, bool record_video);
    /**
     * @brief Get the latest camera image with Apriltag detections highlighted with bounding boxes and tagg IDs.
     * @return The latest annotated image.
     */
    cv::Mat GetAnnotatedIm();


protected:
    /**
     * @brief Setup the capture device and start the detector. If the capture device cannot be opened, restart the thread.
     */
    void Init() override;
    /**
     * @brief Get an image from the capture device, compute the tag poses in the image, annotate the image with the
     * tag bounding boxes, the camera name, and current FPS. Run callback to send tag pose data, and display image locally
     * if flag is set.
     */
    void Execute() override;
    /**
     * @brief Close the capture device.
     */
    void Finish() override;

    std::function<bool(TagArray&)> _queue_tags_callback;

    std::binary_semaphore _annotated_im_sem{1};
    cv::Mat _annotated_im;

    cv::Mat _last_img;
    // Use for both capture video saving and capture video playback
    double _ns_per_frame = 1.0e9/_c_params.fps;
    int _period_frames = 0;
    ulong _start_time = 0;


    std::vector<double> _possible_angles{0, -90, 180, 90};
};