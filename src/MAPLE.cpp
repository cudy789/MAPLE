#include "MAPLE.h"
#include "CalibrationCamWorker.h"
#include "Updater.h"

void MAPLE::Setup(const std::string& config_file) {
    AppLogger::Logger::SetVerbosity(AppLogger::INFO);
    AppLogger::Logger::Log("Starting Multicamera Apriltag Pose Localization and Estimation (MAPLE)", AppLogger::INFO);

    // Register signal handler
    signal(SIGINT, MAPLE::MAPLE::StaticSignalCallback);
//    signal(27, MAPLE::MAPLE::StaticSignalCallback);

    _config_file = config_file;

    SetupHelper(config_file);

}

std::string MAPLE::GetFMAPFilename() {
    return _params.fmap_file;
}

void MAPLE::SetupHelper(const std::string& config_file){
    // Parse the version string
    std::ifstream v_file("version.txt");
    if (v_file.is_open()){
        std::string version;
        getline(v_file, version);
        _version = version;
    } else{
        AppLogger::Logger::Log("Unable to get MAPLE version", AppLogger::SEVERITY::ERROR);
        _version = "Unknown";
    }
    v_file.close();

    // Parse map and configuration files
    _params = ConfigParser::ParseConfig(config_file);
    std::vector<CamParams>& c_params = _params.cam_params;
    std::map<int, Pose_single> tag_layout = TagLayoutParser::ParseConfig(_params.fmap_file);

    // Create localization worker
    _l_w = new LocalizationWorker(_params.pose_logging);
    _workers_t.emplace_back(_l_w);
    _l_w->LogStats(true);

    // Create webserver worker
    if (!_restart_requested){
        _w_w = new WebServerWorker(8080);
    }
    dynamic_cast<WebServerWorker*>(_w_w)->RegisterRobotPoseFunc([this]() -> RobotPose {return _l_w->GetRobotPose();});
//    _workers_t.emplace_back(w_w);

    // Create NetworkTables worker
    if (_params.team_num > 0){
        NTWorker* w_nt = new NTWorker(_params.team_num);
        _workers_t.emplace_back(w_nt);
        w_nt->RegisterPoseCallback([this]() -> RobotPose {return _l_w->GetRobotPose();});
    } else{
        AppLogger::Logger::Log("Not starting NetworkTables, invalid team number provided", AppLogger::SEVERITY::WARNING);
    }

    // Create camera workers
    for (CamParams& p: c_params){
        if (p.calibrate){
            CalibrationCamWorker* this_cam_worker = new CalibrationCamWorker(p);
            dynamic_cast<WebServerWorker*>(_w_w)->RegisterMatFunc([this_cam_worker]() -> cv::Mat {return this_cam_worker->GetAnnotatedIm();});
            _cam_workers_t.emplace_back(this_cam_worker);
        } else{
            TDCamWorker* this_cam_worker = new TDCamWorker(p, tag_layout, [this](TagArray& raw_tags) -> bool {return _l_w->QueueTags(raw_tags);},
                                                           _params.video_recording);
            dynamic_cast<WebServerWorker*>(_w_w)->RegisterMatFunc([this_cam_worker]() -> cv::Mat {return this_cam_worker->GetAnnotatedIm();});
            _cam_workers_t.emplace_back(this_cam_worker);
        }
    }

}

MAPLE &MAPLE::GetInstance() {
    static MAPLE instance; // instantiated on first call, guaranteed to be destroyed
    return instance;
}

void MAPLE::Start(){
    // Start all workers
    for (Worker* w: _workers_t){
        w->Start();
    }
    for (Worker* w: _cam_workers_t){
        w->Start();
    }
    if (!_restart_requested){
        _w_w->Start();
    }

    AppLogger::Logger::Log("All workers have been started");
}

void MAPLE::Calibrate() {

}

RobotPose MAPLE::GetRobotPose() {
    return _l_w->GetRobotPose();
}

void MAPLE::Join(){
    // Wait until the tag detection threads are finished
    bool is_finished = false;
    do{
        _is_finished_sem.acquire();
        is_finished = _is_finished;
        _is_finished_sem.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } while(!is_finished);
}

std::string MAPLE::GetVersion() {
    return _version;
}

void MAPLE::Stop(){
    if (!_restart_requested){
        _w_w->Stop();
        delete _w_w;
    } else{
        dynamic_cast<WebServerWorker*>(_w_w)->ClearRobotPoseFuncRegistrations();
        dynamic_cast<WebServerWorker*>(_w_w)->ClearMatFuncRegistrations();
    }
    for (Worker* t: _cam_workers_t){
        t->Stop();
        delete t;
    }
    _cam_workers_t.clear();
    for (Worker* t: _workers_t){
        t->Stop();
        delete t;
    }
    _workers_t.clear();
    AppLogger::Logger::Log("Stopped all worker threads");
}

void MAPLE::Restart(){
    AppLogger::Logger::Log("Restarting MAPLE", AppLogger::SEVERITY::WARNING);
    _restart_requested = true;
    Stop();
    AppLogger::Logger::Log("Reloading config file", AppLogger::SEVERITY::WARNING);
//    exit(1);
    SetupHelper(_config_file);
    Start();
    _restart_requested = false;

}

void MAPLE::StaticSignalCallback(int signum) {
    GetInstance().SignalCallback(signum);
}

void MAPLE::SignalCallback(int signum) {
    AppLogger::Logger::Log("Caught signal interrupt, handling...", AppLogger::SEVERITY::WARNING);
    Stop();

    if (signum == 27){
        AppLogger::Logger::Log("Restarting MAPLE", AppLogger::SEVERITY::WARNING);
        SetupHelper(_config_file);
        Start();
    } else {
        AppLogger::Logger::Log("Caught CTRL-C, exiting...", AppLogger::SEVERITY::WARNING);
        exit(signum);
    }
}