#include "TDCam.h"
#include "TimeUtils.h"

TDCam::TDCam(CamParams& c_params, const std::map<int, Pose_single>& tag_layout, bool enable_video_writer):
        _c_params(c_params), _tag_layout(tag_layout), _enable_video_writer(enable_video_writer){
    if (_enable_video_writer){
        _writer = new cv::VideoWriter(_c_params.name + "_" + datetime_ms() + ".mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                  30.0, cv::Size(_c_params.rx, _c_params.ry));
        if (!_writer->isOpened()){
            AppLogger::Logger::Log("Error starting video recording on camera " + _c_params.name, AppLogger::SEVERITY::ERROR);
        } else {
            AppLogger::Logger::Log("Enabling video recording on camera " + _c_params.name);
        }
    }

    if (_c_params.dist_coeffs.size() == 5){
        _dist_coeffs = cv::Mat(_c_params.dist_coeffs).reshape(1,1);
    if (_c_params.fx !=0 && _c_params.fy !=0 && _c_params.cx !=0 && _c_params.cy !=0)
        _camera_matrix = (cv::Mat_<double>(3,3) <<
                _c_params.fx, 0.0,          _c_params.cx,
                0.0,          _c_params.fy, _c_params.cy,
                0.0,          0.0,          1.0
        );
    }

}

void TDCam::InitCap() {
    AppLogger::Logger::Log("Enabling video capture on camera " + _c_params.name);

    ulong start_ns = CurrentTime();

    // Initialize camera
    _cap = cv::VideoCapture(_c_params.camera_id, cv::CAP_V4L);
    if (!_cap.isOpened()) {
        AppLogger::Logger::Log("Error enabling video capture on camera " +
                               _c_params.name, AppLogger::SEVERITY::ERROR);
    }

    _cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // Turn off autoexposure = 1, on = 3
    _cap.set(cv::CAP_PROP_EXPOSURE, _c_params.exposure); // set exposure value, do not use auto exposure

    sleep(2);

    _cap.set(cv::CAP_PROP_FPS, _c_params.fps); // Frame rate
    _cap.set(cv::CAP_PROP_FRAME_WIDTH, _c_params.rx); // Width
    _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _c_params.ry); // Height

    _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    ulong end_ns = CurrentTime();
    AppLogger::Logger::Log("Camera capture detector " + _c_params.name +
                           " initialized in " + std::to_string((end_ns - start_ns) / 1.0e9) + " seconds" );

    AppLogger::Logger::Log(std::to_string(_cap.get(cv::CAP_PROP_FRAME_WIDTH)) + "x" +
                           std::to_string(_cap.get(cv::CAP_PROP_FRAME_HEIGHT)) + " @" +
                           std::to_string(_cap.get(cv::CAP_PROP_FPS)) + "FPS");
}

void TDCam::InitRecordedCap() {
    AppLogger::Logger::Log("Playing back video from file " + _c_params.camera_playback_file);

    ulong start_ns = CurrentTime();

    // Initialize camera
    _cap = cv::VideoCapture (_c_params.camera_playback_file, cv::CAP_FFMPEG);
    if (!_cap.isOpened()) {
        AppLogger::Logger::Log("Error opening video file " +
                               _c_params.camera_playback_file, AppLogger::SEVERITY::ERROR);
    }

    _cap.set(cv::CAP_PROP_FPS, _c_params.fps); // Frame rate
    _cap.set(cv::CAP_PROP_FRAME_WIDTH, _c_params.rx); // Width
    _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _c_params.ry); // Height

    sleep(2);

    ulong end_ns = CurrentTime();
    AppLogger::Logger::Log("Video playback detector " + _c_params.name +
                           " initialized in " + std::to_string((end_ns - start_ns) / 1.0e9) + " seconds" );

    AppLogger::Logger::Log(std::to_string(_cap.get(cv::CAP_PROP_FRAME_WIDTH)) + "x" +
                           std::to_string(_cap.get(cv::CAP_PROP_FRAME_HEIGHT)) + " @" +
                           std::to_string(_cap.get(cv::CAP_PROP_FPS)) + "FPS");
}

void TDCam::InitDetector(){
    // Initialize tag detector with options
    _tf = tag36h11_create();

    // Create tag detector for specific tag family
    _tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(_tag_detector, _tf);

    if (errno == ENOMEM) {
        AppLogger::Logger::Log("Unable to add family to detector due to insufficient memory to allocate the "
                               "tag-family decoder with the default maximum hamming value of 2. "
                               "Try choosing an alternative tag family.", AppLogger::SEVERITY::ERROR);
        exit(-1);
    }

    // Set detector options
    _tag_detector->quad_decimate = _c_params.tag_detector.quad_decimate;
    _tag_detector->quad_sigma = _c_params.tag_detector.quad_sigma;
    _tag_detector->nthreads = _c_params.tag_detector.nthreads;
    _tag_detector->debug = _c_params.tag_detector.debug;
    _tag_detector->refine_edges = _c_params.tag_detector.refine_edges;
}

void TDCam::CloseCap(){
    AppLogger::Logger::Log("Closing video capture on camera " + _c_params.name);
    _cap.release();
}

TDCam::~TDCam() {
    apriltag_detector_destroy(_tag_detector);
    tag36h11_destroy(_tf);

    if (_enable_video_writer) {
        _writer->release();
        delete _writer;
    }
}

cv::Mat TDCam::GetImage() {
    cv::Mat img;
    if (!_cap.read(img)){
        AppLogger::Logger::Log("End of " + _c_params.camera_playback_file + " video file");
        return {};
    }

    if (errno == EAGAIN) {
        do {
            AppLogger::Logger::Log("Error getting frame from " + _c_params.camera_playback_file + " cap, trying again", AppLogger::SEVERITY::DEBUG);
            errno=0;
            if (!_cap.read(img)){
                AppLogger::Logger::Log("End of " + _c_params.camera_playback_file + " video file");
                return {};
            }
        } while (errno == EAGAIN);
    }

    cv::Mat brightened;
    img.convertTo(brightened, -1, 1.0, 50);

    return brightened;
}

void TDCam::SaveImage(const cv::Mat &img) {
    _writer->write(img);
}

void TDCam::Undistort(cv::Mat &img){
    cv::Mat input = img.clone();
    cv::undistort(input, img, _camera_matrix, _dist_coeffs);
}

TagArray TDCam::GetTagsFromImage(const cv::Mat &img) {
    TagArray detected_tags;
    // Convert img to grayscale
    cv::Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);


    // Make an image_u8_t header for the Mat data
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

    // Detect tags in image
    zarray_t *detections = apriltag_detector_detect(_tag_detector, &im);
    if (errno == EAGAIN) {
        AppLogger::Logger::Log("Unable to create the " + std::to_string(_tag_detector->nthreads) +
            " threads requested", AppLogger::SEVERITY::ERROR);
        AppLogger::Logger::Log("Number of threads in workerpool:" + to_string(workerpool_get_nthreads(_tag_detector->wp)));
        errno=0;
        return detected_tags;
    }

    // Iterate through each detection in the frame
    for (int i = 0; i < zarray_size(detections); i++) {
        // ==================== Calculate the pose of the tag ====================

        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // Check that tagID is within our expected tag range
        if (det->id < 0 || det->id >= NUM_TAG_IDS){
            continue;
        }

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.1651; // in meters
        // Camera params from https://horus.readthedocs.io/en/release-0.2/source/scanner-components/camera.html

        info.fx = _c_params.fx;
        info.fy = _c_params.fy;

        info.cx = _c_params.cx;
        info.cy = _c_params.cy;

        // Calculate the tag's pose, return an error value as well
        double err1, err2;
        apriltag_pose_t pose1, pose2;
        // Use underlying method of estimate_tag_pose so we can use both possible pose locations
        estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 50);

        std::vector<apriltag_pose_t*> calculated_poses;
        std::vector<double> calculated_errors;
        if (pose2.R){
            calculated_poses = {&pose1, &pose2};
            calculated_errors = {err1, err2};
        } else{
            calculated_poses = {&pose1};
            calculated_errors = {err1};
        }

        for (int j=0; j < calculated_poses.size(); j++) {
            apriltag_pose_t* pose = calculated_poses[j]; // get the pose and corresponding error
            double err = calculated_errors[j];

            Eigen::Matrix3d R_tag_local_fix = CreateRotationMatrix({0,0,0}) * CreateRotationMatrix({-90,0,0});
//            Eigen::Matrix3d T_tag_global_fix = CreateRotationMatrix({-90, 90, 0}); // working without any tag rotation
            Eigen::Matrix3d T_tag_global_fix = GetRotTranslationFix();
            Eigen::Matrix3d R_tag_global_fix = GetRotRotationFix();
//            Eigen::Matrix3d T_tag_global_fix_2 = GetRotTranslationFix();


            Eigen::Vector3d T_tag_camera_raw = Array2EM<double, 3, 1>(pose->t->data);
            Eigen::Matrix3d R_tag_camera_raw = Array2EM<double, 3, 3>(pose->R->data);
//            Eigen::Matrix3d R_tag_camera = R_tag_local_fix * R_tag_camera_raw;


            // Get the location of the apriltag in the world frame
            Pose_single Pose_AG; // the field transformation from apriltag frame to global field frame as specified in the .fmap file
            if (_tag_layout.find(det->id) != _tag_layout.end()) {
                Pose_AG = _tag_layout[det->id];
            } else {
                AppLogger::Logger::Log("Cannot find tag ID " + to_string(det->id) + " in .fmap file", AppLogger::SEVERITY::WARNING);
                if (pose->R) matd_destroy(pose->R);
                if (pose->t) matd_destroy(pose->t);
                continue;
            }

            // ============= Global Frame =============
            Eigen::Matrix3d R_robot_global_unordered = Pose_AG.R * ((R_tag_global_fix * R_tag_camera_raw.transpose()));
            Eigen::Vector3d T_robot_global_unordered = Pose_AG.T - _c_params.R_camera_robot.transpose() * (T_tag_global_fix * (
                       _c_params.R_camera_robot * Pose_AG.R * ((R_tag_global_fix * R_tag_camera_raw.transpose()))
                    ) * T_tag_camera_raw + _c_params.R_camera_robot * _c_params.T_camera_robot);

            Eigen::Vector3d T_robot_global = {T_robot_global_unordered[0], T_robot_global_unordered[1], T_robot_global_unordered[2]};

            Eigen::Vector3d R_robot_ordered_vec = RotationMatrixToRPY(R_robot_global_unordered);
            R_robot_ordered_vec[0] += 90;
            if (R_robot_ordered_vec[0] >=180) R_robot_ordered_vec[0] -= 180;
            Eigen::Vector3d c_extrinsic_rotation = RotationMatrixToRPY(_c_params.R_camera_robot);
            Eigen::Matrix3d R_robot_global = CreateRotationMatrix({R_robot_ordered_vec[1]-c_extrinsic_rotation[0], R_robot_ordered_vec[0]-c_extrinsic_rotation[1], R_robot_ordered_vec[2]-c_extrinsic_rotation[2]});

            // ============= Robot Frame =============

            Eigen::Matrix3d R_robot_unordered = ((R_tag_global_fix * R_tag_camera_raw.transpose()));


//            Eigen::Vector3d T_robot_unordered = (_c_params.R_camera_robot.transpose() * T_tag_camera_raw) + _c_params.T_camera_robot;
//
//            Eigen::Vector3d T_robot = {-T_robot_unordered[1], T_robot_unordered[2], T_robot_unordered[0]};
            Eigen::Vector3d cam_rpy_raw = RotationMatrixToRPY(_c_params.R_camera_robot);
            cam_rpy_raw[2] += 90;
            cam_rpy_raw[2] *= -1;
            //                                base dist (x)             offset center of robot  (yaw * y dist)                                 account for dependent axis (pitch * y dist)
            Eigen::Vector3d T_robot = {T_tag_camera_raw[0] - sin(Deg2Rad(cam_rpy_raw[2])) * T_tag_camera_raw[2] + sin(Deg2Rad(cam_rpy_raw[1])) * T_tag_camera_raw[2],
            //                                base dist (y)             offset center of robot  (pitch * z dist)                               account for dependent axis (yaw * x dist)
                                       T_tag_camera_raw[2] - sin(Deg2Rad(cam_rpy_raw[1])) * T_tag_camera_raw[1] + sin(Deg2Rad(cam_rpy_raw[2])) * T_tag_camera_raw[0],
                                       T_tag_camera_raw[1]
            };

            T_robot[0] += -_c_params.T_camera_robot[1];
            T_robot[1] += _c_params.T_camera_robot[0];
            T_robot[2] += _c_params.T_camera_robot[2];

            AppLogger::Logger::Log("cam_rpy_raw: " + to_string(cam_rpy_raw));
            AppLogger::Logger::Log("T_tag_camera_raw: " + to_string(T_tag_camera_raw));

            Eigen::Vector3d R_robot_robot_ordered_vec = RotationMatrixToRPY(R_robot_unordered);
            R_robot_robot_ordered_vec[0] += 90;
            if (R_robot_robot_ordered_vec[0] >=180) R_robot_robot_ordered_vec[0] -= 180;

            R_robot_robot_ordered_vec[2] += 180;
            if (R_robot_robot_ordered_vec[0] >=180) R_robot_robot_ordered_vec[0] -= 180;
            //Eigen::Vector3d c_extrinsic_rotation = RotationMatrixToRPY(_c_params.R_camera_robot);
            Eigen::Matrix3d R_robot = CreateRotationMatrix({R_robot_robot_ordered_vec[1]-c_extrinsic_rotation[0], R_robot_robot_ordered_vec[0]-c_extrinsic_rotation[1], R_robot_robot_ordered_vec[2]-c_extrinsic_rotation[2]});



            // ==================== Now make the Pose_t object ====================

            Pose new_tag;
            new_tag.cam_id = _c_params.camera_id;
            new_tag.tag_id = det->id;
            new_tag.err = err * 1e5;
            // Tag frame
            new_tag.tag.R = Eigen::Matrix3d::Constant(0);
            new_tag.tag.T = Eigen::Vector3d::Constant(0);
            // Camera frame
            new_tag.camera.R = R_tag_camera_raw;
            new_tag.camera.T = T_tag_camera_raw;
            // Robot frame
            new_tag.robot.R = R_robot;
            new_tag.robot.T = T_robot;
            // Global frame
            new_tag.global.R = R_robot_global;
            new_tag.global.T = T_robot_global;

            // CV pixel coords for outlining tag on image
            new_tag.c[0] = det->c[0];
            new_tag.c[1] = det->c[1];
            for (int  k = 0; k < 4; k++) {
                for (int l = 0; l < 2; l++) {
                    new_tag.p[k][l] = det->p[k][l];
                }
            }

            AppLogger::Logger::Log("Processed tag " + to_string(new_tag), AppLogger::SEVERITY::DEBUG);
            AppLogger::Logger::Log("Tag " + to_string(det->id) + " known global location: " + to_string(Pose_AG.T), AppLogger::SEVERITY::DEBUG);

            // Add tag to detected TagArray object
            detected_tags.AddTag(new_tag);

            if (pose->R) matd_destroy(pose->R);
            if (pose->t) matd_destroy(pose->t);

        }
    }
    apriltag_detections_destroy(detections);
    return detected_tags;
}

cv::Mat TDCam::DrawTagBoxesOnImage(const TagArray &tags, const cv::Mat &img) {
    cv::Mat annotated_img = img;
    for (const std::vector<Pose>& v: tags.data){
        for (const Pose& p: v){

            // Draw detection outlines
            line(annotated_img, cv::Point(p.p[0][0], p.p[0][1]),
                 cv::Point(p.p[1][0], p.p[1][1]),
                 cv::Scalar(0, 0xff, 0), 2);
            line(annotated_img, cv::Point(p.p[0][0], p.p[0][1]),
                 cv::Point(p.p[3][0], p.p[3][1]),
                 cv::Scalar(0, 0, 0xff), 2);
            line(annotated_img, cv::Point(p.p[1][0], p.p[1][1]),
                 cv::Point(p.p[2][0], p.p[2][1]),
                 cv::Scalar(0xff, 0, 0), 2);
            line(annotated_img, cv::Point(p.p[2][0], p.p[2][1]),
                 cv::Point(p.p[3][0], p.p[3][1]),
                 cv::Scalar(0xff, 0, 0), 2);

            std::stringstream ss;
            ss << p.tag_id;
            cv::String text = ss.str();
            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
            putText(annotated_img, text, cv::Point(p.c[0]-textsize.width/2,
                                       p.c[1]+textsize.height/2),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        }
    }
    return annotated_img;
}

void TDCam::ImShow(const std::string& title, int timeout, const cv::Mat& img) {
    imshow(title, img);
    if (cv::waitKey(timeout) >= 0) return;
}

void TDCam::SetRotRotationFix(const Eigen::Matrix3d& R_fix) {
    _R_rotation_fix = R_fix;
}

void TDCam::SetTransRotationFix(const Eigen::Matrix3d& T_fix) {
    _R_translation_fix = T_fix;
}

const Eigen::Matrix3d &TDCam::GetRotRotationFix() const {
    return _R_rotation_fix;
}

const Eigen::Matrix3d &TDCam::GetRotTranslationFix() const {
    return _R_translation_fix;
}
