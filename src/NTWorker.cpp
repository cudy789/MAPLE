#include "NTWorker.h"

NTWorker::NTWorker() :
        Worker{"NetworkTables worker"},
        _nt_instance{nt::NetworkTableInstance::GetDefault()} {

    _nt_table = _nt_instance.GetTable("MAPLE");

    _position = _nt_table->GetDoubleArrayTopic("position").Publish();
    _orientation = _nt_table->GetDoubleArrayTopic("orientation").Publish();

    _robot_relative_tagid = _nt_table->GetIntegerArrayTopic("robot_relative_tagid").Publish();
    _robot_relative_x = _nt_table->GetDoubleArrayTopic("robot_relative_x").Publish();
    _robot_relative_y = _nt_table->GetDoubleArrayTopic("robot_relative_y").Publish();
    _robot_relative_yaw = _nt_table->GetDoubleArrayTopic("robot_relative_yaw").Publish();
}

NTWorker::NTWorker(int team_num) :  NTWorker() { _team_num=team_num; }

NTWorker::NTWorker(std::string hostname) :  NTWorker() { _hostname=hostname; }


NTWorker::NTWorker(const std::string& hostname, int ntport4) {
    _hostname = hostname;
    _ntport4 = ntport4;
}

void NTWorker::RegisterPoseCallback(const std::function<RobotPose()>& pose_callback) {
    _pose_callback = pose_callback;
}

bool NTWorker::IsConnected(){
    return _nt_instance.IsConnected();
}

void NTWorker::Init() {
    _nt_instance.StartClient4("MAPLE client");

    if (_team_num != -1){
        AppLogger::Logger::Log("NTWorker connecting using team number " + to_string(_team_num));
        _nt_instance.SetServerTeam(_team_num);
    } else {
        AppLogger::Logger::Log("NTWorker connecting to " + _hostname + ":" + to_string(_ntport4));
        _nt_instance.SetServer(_hostname, _ntport4);
    }

    sleep(2);

    if (_nt_instance.IsConnected()){
        AppLogger::Logger::Log("NTWorker successfully connected");
    } else{
        AppLogger::Logger::Log("NTWorker could not connect to NetworkTables server, retrying...", AppLogger::SEVERITY::WARNING);
        Stop(false);
        sleep(10);
    }
}

void NTWorker::Execute() {
    RobotPose new_pose = _pose_callback();

    // Publish global orientation
    std::vector<double> new_position = {new_pose.global.T[0], new_pose.global.T[1], new_pose.global.T[2]};
    Eigen::Vector3d eig_new_orientation = RotationMatrixToRPY(new_pose.global.R);
    std::vector<double> new_orientation = {eig_new_orientation[0], eig_new_orientation[1], eig_new_orientation[2]};
    _position.Set(new_position);
    _orientation.Set(new_orientation);

    // Publish robot relative tags
    std::vector<long> rr_tagid;
    std::vector<double> rr_x;
    std::vector<double> rr_y;
    std::vector<double> rr_yaw;

    for (const auto& tag_vec: new_pose.RelativeTags.data){
        if (!tag_vec.empty()){
            for (const auto& t: tag_vec){
                rr_tagid.push_back(t.tag_id);
                rr_x.push_back(t.robot.T[0]);
                rr_y.push_back(t.robot.T[1]);
                rr_yaw.push_back(RotationMatrixToRPY(t.robot.R)[2]);
            }
        }
    }

    _robot_relative_tagid.Set(rr_tagid);
    _robot_relative_x.Set(rr_x);
    _robot_relative_y.Set(rr_y);
    _robot_relative_yaw.Set(rr_yaw);

}