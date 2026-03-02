#pragma once

#include "MatrixHelpers.h"
#include "Pose.h"
#include "opencv2/opencv.hpp"
#include "Logger.h"
#include "ConfigParser.h"
#include "TagLayoutParser.h"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/homography.h"
}

/**
 * @brief A tag detector camera class which opens a webcam capture and performs Apriltag detection on the image frame. One
 * TDCam object per attached webcam.
 */
class TDCam{
public:
    /**
     * @brief No default constructor.
     */
    TDCam() = delete;
    /**
     * @brief Assign param structs to local member variables.
     * @param c_params The camera configuration to use during setup and computations.
     * @param tag_layout The Apriltag field layout to use during computations.
     */
    explicit TDCam(CamParams& c_params, const std::map<int, Pose_single>& tag_layout, bool enable_video_writer=false);
    /**
     * @brief Ensure all Apriltag dynamic memory is cleaned up in child classes.
     */
    virtual ~TDCam();

    /**
     * @brief Create the cv::VideoCapture object to pull frames from. Try to start capture at the desired resolution,
     * exposure, and FPS specified in the _c_params struct. Disable autoexposure. Note: OpenCV doesn't force the cameras to run at the
     * specified properties, the camera will choose whichever parameters are closest to its available features. Camera
     * exposure setting is not supported on some cameras (Logitech C505e for example).
     */
    void InitCap();
    /**
     * @brief Create the cv::VideoCapture from a prerecorded .avi file for evaluation. Opens the file at the specified
     * frame rate and resolution as configured in the .yml file.
     */
    void InitRecordedCap();
    /**
     * @brief Create the Apriltag detector with the specified detector parameters.
     */
    virtual void InitDetector();
    /**
     * @brief Gracefully close the capture device.
     */
    virtual void CloseCap();
    /**
     * @brief Get an image frame from the opened capture device. Will block until the capture returns an image.
     * @return An image frame from the capture device.
     */
    cv::Mat GetImage();
    /**
     * @brief Save the image to a video file. Blocks until image is written.
     */
    void SaveImage(const cv::Mat& img);
    /**
     * @brief Undistort an image using the provided camera intrinsic parameters and distortion coefficients. Retains original
     * image resolution.
     * @param img [input:output] The image to undistort.
     */
    void Undistort(cv::Mat &img);

    /**
     * @brief Run the Apriltag detector on an input image and find all possible tag poses for all tags in the image. Calculates
     * the pose for each tag in four frames: tag, tag w.r.t. camera, tag w.r.t. robot, and robot w.r.t. world. Organizes
     * these detections by tag ID into the TagArray object.
     * @param img The image to run the Apriltag detector on.
     * @return All possible tag detections found in the image.
     */

    /**
     * @brief TODO
     * @param T_ct
     * @param R_ct
     * @return
     */
    Pose_single CameraPoseToGlobal(const Eigen::Vector3d& T_ct, const Eigen::Matrix3d& R_ct, const Eigen::Vector3d& T_wt, const Eigen::Matrix3d& R_wt);

    TagArray GetTagsFromImage(const cv::Mat& img);
    /**
     * @brief Given an image and its corresponding tag detections, draw a bounding box around the detected tags in the frame
     * and label the tag with its ID.
     * @param tags All tags detected in this image.
     * @param img The image to draw on.
     * @return The annotated image with bounding boxes and labels.
     */
    cv::Mat DrawTagBoxesOnImage(const TagArray& tags, const cv::Mat& img);

    /**
     * @brief Use OpenCVs imshow function to display an image.
     * @param title Title of the image window.
     * @param timeout How long to wait before closing the image window, in milliseconds.
     * @param img The image to display in the window.
     */
    void ImShow(const std::string& title, int timeout, const cv::Mat& img);

protected:
    CamParams _c_params;

    cv::Mat _dist_coeffs;
    cv::Mat _camera_matrix;

    std::map<int, Pose_single> _tag_layout;

    cv::VideoCapture _cap;

    bool _enable_video_writer;
    cv::VideoWriter* _writer;

    apriltag_detector_t* _tag_detector;
    apriltag_family_t* _tf;

//    Eigen::Matrix3d _R_translation_fix = CreateRotationMatrix({-90, 0, 0}); // pitch and yaw works, not roll
////    Eigen::Matrix3d _R_rotation_fix = CreateRotationMatrix({-90, 0, 90}); // this worked with _R_translation_fix = Identity
//    Eigen::Matrix3d _R_rotation_fix = CreateRotationMatrix({0, 0, 90}); // pitch and yaw works, not roll

    Eigen::Matrix3d _R_translation_fix = CreateRotationMatrix({0, 0, 0});
    Eigen::Matrix3d _R_rotation_fix = CreateRotationMatrix({-90, 0, 90}); // pitch and yaw works, not roll

public:
    const Eigen::Matrix3d &GetRotRotationFix() const;

    const Eigen::Matrix3d &GetRotTranslationFix() const;

protected:

    void SetRotRotationFix(const Eigen::Matrix3d& R_fix);
    void SetTransRotationFix(const Eigen::Matrix3d& T_fix);
};