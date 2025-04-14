#include "WebServerWorker.h"
#include "nlohmann/json.hpp"

WebServerWorker::WebServerWorker(unsigned short port) :
        Worker{"Webserver worker", true, 30.0},  // Call Worker constructor
        _port(port),
        _mgr(mg_mgr())
{}

WebServerWorker::~WebServerWorker() {
    mg_mgr_free(&_mgr);  // This will close all connections and free ports
    _connection = nullptr;
    _viewer_connection = nullptr;
    _ws_connection = nullptr;
}

bool WebServerWorker::RegisterMatFunc(const std::function<cv::Mat()>& mat_func) {
    _mat_funcs_sem.acquire();
    _mat_funcs.emplace_back(mat_func);
    _mat_funcs_sem.release();
    return true;
}

void WebServerWorker::ClearMatFuncRegistrations() {
    _mat_funcs_sem.acquire();
    _mat_funcs.clear();
    _mat_funcs_sem.release();
}

bool WebServerWorker::RegisterRobotPoseFunc(const std::function<RobotPose()>& pose_func) {
    _robot_pose_func_sem.acquire();
    _robot_pose_func = pose_func;
    _robot_pose_func_sem.release();
    return true;
}

void WebServerWorker::ClearRobotPoseFuncRegistrations() {
    _robot_pose_func_sem.acquire();
    _robot_pose_func = []() -> RobotPose {return {};};
    _robot_pose_func_sem.release();

}

void WebServerWorker::Init() {
    mg_mgr_init(&_mgr);
    _mgr.userdata = this;

    std::string address = "http://0.0.0.0:" + std::to_string(_port+1);
    _connection =
        mg_http_listen(&_mgr, address.c_str(),
           [](mg_connection *conn, int ev, void *ev_data) {
             if (ev == MG_EV_HTTP_MSG) {
                 mg_printf(conn,
                           "HTTP/1.0 200 OK\r\n"
                           "Cache-Control: no-cache\r\n"
                           "Pragma: no-cache\r\nExpires: Thu, 01 Dec 1994 16:00:00 GMT\r\n"
                           "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n");
                 conn->data[0] = 'S';
                 AppLogger::Logger::Log("Client connected to webserver");
             }
         },
         this);
    if (_connection == nullptr) {
        AppLogger::Logger::Log("Failed to start Mongoose server on port " + std::to_string(_port), AppLogger::SEVERITY::WARNING);
        sleep(10);
        Stop(false);
    }

    std::string viewer_address = "http://0.0.0.0:" + std::to_string(_port);

    _viewer_connection =
        mg_http_listen(&_mgr, viewer_address.c_str(),
           [](mg_connection *conn, int ev, void *ev_data) {
               if (ev == MG_EV_HTTP_MSG) {
                   struct mg_http_message *hm = (struct mg_http_message *) ev_data;
                   if (mg_match(hm->uri, mg_str("/api/get-config"), NULL)) {
                       AppLogger::Logger::Log("Trying to get config for webpage editor");
                       std::ifstream file("../config.yml");
                       if (!file) {
                           mg_http_reply(conn, 500, "Content-Type: text/plain\r\n", "Failed to read config file");
                           return;
                       }
                       std::stringstream buffer;
                       buffer << file.rdbuf();
                       mg_http_reply(conn, 200, "Content-Type: text/plain\r\n", "%s", buffer.str().c_str());
                   } else if (mg_match(hm->uri, mg_str("/api/save-config"), NULL)) {
                       AppLogger::Logger::Log("Trying save get config from webpage editor");

                       // Parse the incoming JSON data
                       nlohmann::json requestData = nlohmann::json::parse(std::string(hm->body.buf, hm->body.len));

                       // Extract the configuration data and decode it correctly
                       std::string config_data = requestData.at("config").get<std::string>();

                       std::ofstream temp_outfile("../config.yml.tmp");
                       if (!temp_outfile) {
                           mg_http_reply(conn, 500, "Content-Type: text/plain\r\n", "Failed to save config file");
                           AppLogger::Logger::Log("Failed to save config file", AppLogger::SEVERITY::ERROR);
                           return;
                       }
                       temp_outfile << config_data;
                       temp_outfile.close();
                       try{
                           ConfigParser::ParseConfig("../config.yml.tmp");
                       } catch (const std::exception& e) {
                           mg_http_reply(conn, 501, "Content-Type: text/plain\r\n", "Error: invalid .yml file");
                           AppLogger::Logger::Log("Error: Configuration cannot be parsed correctly.", AppLogger::SEVERITY::ERROR);
                           std::remove("../config.yml.tmp");
                           return;
                       }
                       std::remove("../config.yml.tmp");

                       std::ofstream outfile("../config.yml");
                       outfile << config_data;
                       outfile.close();

                       AppLogger::Logger::Log("Received new config data: " + config_data);

                       mg_http_reply(conn, 200, "Content-Type: text/plain\r\n", "Configuration saved successfully");
                       AppLogger::Logger::Log("Successfully saved config");

//                       exit(1);
                       auto* server = static_cast<WebServerWorker*>(conn->mgr->userdata);
                       server->_restart_requested = true;
//                       std::raise(27);
                   }
                   else {
                       struct mg_http_serve_opts opts = {.root_dir = "./web/"};

                       mg_http_serve_dir(conn, hm, &opts);
                       AppLogger::Logger::Log("Client connected to viewer");
                   }
               }
           },
           this);

    std::string ws_address = "http://0.0.0.0:" + std::to_string(_port+2);

    _ws_connection =
            mg_http_listen(&_mgr, ws_address.c_str(),
                           [](mg_connection *conn, int ev, void *ev_data) {
                               if (ev == MG_EV_HTTP_MSG) {
                                   AppLogger::Logger::Log("Get http request on websocket port, upgrading to ws");
                                   struct mg_http_message *hm = (struct mg_http_message *) ev_data;
                                   mg_ws_upgrade(conn, hm, NULL);
                               } else if (ev == MG_EV_WS_OPEN){
                                   AppLogger::Logger::Log("Websocket connected");
                                   conn->data[0] = 'W';
                               }
                           },
                           this);

    AppLogger::Logger::Log("Starting webserver");
}

void WebServerWorker::Execute() {
    _mat_funcs_sem.acquire();
    if (!_mat_funcs.empty()){
        cv::Mat merged_frame = _mat_funcs[0]();
        for (int i=1; i<_mat_funcs.size(); i++){
            const cv::Mat& new_frame = _mat_funcs[i]();
            if (merged_frame.empty()) merged_frame = new_frame;
            if (new_frame.empty()) continue;
            if (new_frame.cols != merged_frame.cols){// TODO allow multiple camera image sizes
                continue;
            }
            cv::vconcat(merged_frame, new_frame, merged_frame);
        }
        _mat_funcs_sem.release();
        if (!merged_frame.empty()) {
            std::vector<uchar> buf;
            cv::imencode(".jpg", merged_frame, buf, std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 25});

            mg_connection *conn;
            for (conn=_mgr.conns; conn !=nullptr; conn=conn->next){
                if (conn->data[0] == 'S'){
                    mg_printf(conn,
                              "--frame\r\n"
                              "Content-Type: image/jpeg\r\n"
                              "Content-Length: %lu\r\n\r\n",
                              buf.size());
                    mg_send(conn, buf.data(), buf.size());
                    mg_printf(conn, "\r\n", 2);
                } else if (conn->data[0] == 'W'){
                    _robot_pose_func_sem.acquire();
                    RobotPose latest_pose = _robot_pose_func();
                    _robot_pose_func_sem.release();
                    double x = latest_pose.global.T[0];
                    double y = latest_pose.global.T[1];
                    double z = latest_pose.global.T[2];
                    Eigen::Vector3d rpy = RotationMatrixToRPY(latest_pose.global.R);

                    std::string test_data =
                            "{\"x\": " + to_string(x) +
                            ",\"y\": " + to_string(y) +
                            ",\"z\": " + to_string(z) +
                            ",\"roll\": " + to_string(rpy[0]) +
                            ",\"pitch\": " + to_string(rpy[1]) +
                            ",\"yaw\": " + to_string(rpy[2]) +
                            "}";

                    mg_ws_send(conn, test_data.c_str(), test_data.size(), WEBSOCKET_OP_TEXT);
                }
            }
        } else {
            AppLogger::Logger::Log("Webserver merged frame is empty", AppLogger::SEVERITY::DEBUG);
        }

    } else {
//        AppLogger::Logger::Log("Webserver has no camera streams", AppLogger::SEVERITY::WARNING);
        _mat_funcs_sem.release();
    }


    if (_restart_requested){
        AppLogger::Logger::Log("Webserver restart requested", AppLogger::SEVERITY::WARNING);

        std::thread([this]() {
            MAPLE::GetInstance().Restart();
        }).detach();
        _restart_requested = false;

    }
    mg_mgr_poll(&_mgr, 0);

}