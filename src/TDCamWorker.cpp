#include "TDCamWorker.h"

TDCamWorker::TDCamWorker(CamParams& c_params, const std::map<int, Pose_single>& tag_layout,
                         std::function<bool(TagArray&)> queue_tags_callback, bool record_video)
        : Worker{"TDCamWorker " + c_params.name, true, 100, AppLogger::SEVERITY::DEBUG},
          TDCam{c_params, tag_layout, record_video},
          _queue_tags_callback{std::move(queue_tags_callback)}
          {}

void TDCamWorker::Init() {
    if (!_c_params.camera_playback_file.empty()){
        InitRecordedCap();
        SetExecutionFreq(_c_params.fps);
    } else{
        InitCap();
    }
    InitDetector();

    if (!_cap.isOpened()){
        AppLogger::Logger::Log("Camera " + std::to_string(_c_params.camera_id) + " cannot be opened", AppLogger::SEVERITY::ERROR);
        Stop(false);
        sleep(5);

    } else{
        AppLogger::Logger::Log("Starting tag detector for cam " + std::to_string(_c_params.camera_id));
        if (_camera_matrix.empty()){
            AppLogger::Logger::Log("No distortion matrix was found, not undistorting images before processing", AppLogger::SEVERITY::WARNING);
        }
    }
}

void TDCamWorker::Finish() {
    AppLogger::Logger::Log("in the tdcamworker finish function");
    CloseCap();
    Worker::Finish();
}

cv::Mat TDCamWorker::GetAnnotatedIm() {
    cv::Mat ret_mat;
    if (_annotated_im_sem.try_acquire_for(std::chrono::duration<ulong, std::milli>(500))){
        ret_mat = _annotated_im.clone();
        _annotated_im_sem.release();
    }
    return ret_mat;
}

void TDCamWorker::Execute() {
    cv::Mat img = GetImage();

    if (!img.empty()) {
        try {
            cv::Mat annotated_im;

            // Save image to video file
            if (_enable_video_writer || !_c_params.camera_playback_file.empty()){
                if (_start_time == 0) _start_time = CurrentTime();

                if (_enable_video_writer) SaveImage(img);

                auto period_time = [this]() -> long {return (CurrentTime() - _start_time) % (ulong)1.0e9;};
                _period_frames++;

                // Check if we need to save another frame this period or read more frames from the video capture, the
                // camera might be running at a lower FPS than what is specified or we are not reading at a constant rate.
                // We must maintain a constant framerate when readin/writing to a video file.
                if (_period_frames * _ns_per_frame < period_time()){

                    int extra_frames = std::ceil((period_time() - (_period_frames * _ns_per_frame)) / _ns_per_frame);
                    AppLogger::Logger::Log(_c_params.name + " needs " + to_string(extra_frames) + " extra frames", AppLogger::SEVERITY::DEBUG);
                    for (int i=0; i<extra_frames; i++){
                        // If writing to video file, write image. If reading, read image.
                        if (_enable_video_writer) SaveImage(img);
                        else img = GetImage();
                        _period_frames++;
                    }
                }
                if (_period_frames > _c_params.fps) _period_frames=0;
                annotated_im = img;

            }
            if (!_enable_video_writer){
                // Undistort the image
                if (!_camera_matrix.empty()) {
                    Undistort(img);
                }

                // Find tags in image
                TagArray raw_tags = GetTagsFromImage(img);

//                bool found_fix = false;
//                if (raw_tags.GetNumTags() > 0){
//                    for (double tr_angle: _possible_angles){
//                        if (found_fix) break;
//                        for(double tp_angle: _possible_angles){
//                            if (found_fix) break;
//                            for (double ty_angle: _possible_angles){
//                                if (found_fix) break;
//                                for (double rr_angle: _possible_angles){
//                                    if (found_fix) break;
//                                    for (double rp_angle: _possible_angles){
//                                        if (found_fix) break;
//                                        for (double ry_angle: _possible_angles){
//                                            if (found_fix) break;
//                                            Eigen::Matrix3d trot_fix = CreateRotationMatrix({tr_angle, tp_angle, ty_angle});
//                                            Eigen::Matrix3d rrot_fix = CreateRotationMatrix({rr_angle, rp_angle, ry_angle});
//                                            SetTransRotationFix(trot_fix);
//                                            SetRotRotationFix(rrot_fix);
//                                            TagArray fixed_tags = GetTagsFromImage(img);
//                                            Pose_single lerr_tag;
//                                            Pose_single desired_pose{{1, 0.5, 0.5}, CreateRotationMatrix({0, 0, -70})};
//
//                                            if (fixed_tags.data.at(12).at(0).err > fixed_tags.data.at(12).at(1).err){
//                                                lerr_tag = fixed_tags.data.at(12).at(1).global;
//                                            } else{
//                                                lerr_tag = fixed_tags.data.at(12).at(0).global;
//                                            }
//
////                                            lerr_tag.R = CreateRotationMatrix({(RotationMatrixToRPY(lerr_tag.R)[0] + 90), RotationMatrixToRPY(lerr_tag.R)[1], RotationMatrixToRPY(lerr_tag.R)[2]});
//
//
//// TODO try this one next, add 90 degrees to roll
//// [2025-01-16_03:12:10.340] INFO:         ######### FOUND CORRECT TRANSLATION ROTATION FIX #########
////[2025-01-16_03:12:10.340] INFO:         troll: 0 tpitch: 0 tyaw: 0
////[2025-01-16_03:12:10.340] INFO:         rroll: -90 rpitch: 0 ryaw: 90
////[2025-01-16_03:12:10.340] INFO:                 New pose: XYZ: [[-0.5173], [0.4789], [0.4905]] RPY: [[-89.7297], [-14.9583], [-30.1566]]
//
//
//                                            if (EigenEquals(RotationMatrixToRPY(lerr_tag.R), RotationMatrixToRPY(desired_pose.R), 5)){
////                                            AppLogger::Logger::Log("lerr_tag: " + to_string(lerr_tag));
////                                            if (EigenEquals(lerr_tag.T, desired_pose.T, 0.5) && EigenEquals(RotationMatrixToRPY(lerr_tag.R), RotationMatrixToRPY(desired_pose.R), 8)){
////                                            if (EigenEquals(lerr_tag.T, desired_pose.T, 0.3) ){
////                                            if (EigenEquals(RotationMatrixToRPY(lerr_tag.R), RotationMatrixToRPY(desired_pose.R), 8)){
////                                                found_fix=true;
//                                                AppLogger::Logger::Log("\t######### FOUND CORRECT TRANSLATION ROTATION FIX #########");
//                                                AppLogger::Logger::Log("\ttroll: " + to_string(tr_angle) + " tpitch: " + to_string(tp_angle) + " tyaw: " + to_string(ty_angle));
//                                                AppLogger::Logger::Log("\trroll: " + to_string(rr_angle) + " rpitch: " + to_string(rp_angle) + " ryaw: " + to_string(ry_angle));
//                                                AppLogger::Logger::Log("\t\tNew pose: " + to_string(lerr_tag));
//                                            }
//
//                                        }
//                                    }
//                                }
//
//
//
//                            }
//                        }
//                    }
//                }
//                if (!found_fix) AppLogger::Logger::Log("Did not find a translation rotation fix", AppLogger::SEVERITY::ERROR);
//                exit(0);


                bool success = _queue_tags_callback(raw_tags);
                if (!success) {
                    AppLogger::Logger::Log("Camera " + std::to_string(_c_params.camera_id) +
                                           " error acquiring lock to add tags to processing queue",
                                           AppLogger::SEVERITY::WARNING);
                }
                // Draw tags onto image
                annotated_im = DrawTagBoxesOnImage(raw_tags, img);
            }


            // Put fps in top right corner
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << GetExecutionFreq();
            cv::putText(annotated_im, ss.str(), cv::Point(annotated_im.size().width - 50, 0 + 20),
                        cv::FONT_HERSHEY_DUPLEX, 0.65, cv::Scalar(0, 255, 0), 2);

            // Put camera name in top left corner
            cv::putText(annotated_im, _c_params.name, cv::Point(10, 0 + 20),
                        cv::FONT_HERSHEY_DUPLEX, 0.65, cv::Scalar(0, 255, 0), 2);

            // Update latest annotated image
            if (_annotated_im_sem.try_acquire_for(std::chrono::duration<ulong, std::milli>(100))) {
                _annotated_im = annotated_im;
                _annotated_im_sem.release();
            }

        } catch(cv::Exception& e)
        {
            AppLogger::Logger::Log(e.what(), AppLogger::SEVERITY::ERROR);
        }
    } else {
        if (!_c_params.camera_playback_file.empty()){
            if (errno != EAGAIN){
                AppLogger::Logger::Log("Reached the end of the video file " + _c_params.camera_playback_file + ", stopping thread.");
                Stop(false);
            }
        } else{
            AppLogger::Logger::Log("Error getting img from camera " + _c_params.name, AppLogger::SEVERITY::WARNING);
            Stop(false);
            sleep(5);
        }
    }

}