#include <gtest/gtest.h>

#include "common.h"
#include "NTWorker.h"

std::function<RobotPose()> CreateCallback(const std::vector<double>& pos_vals){
    return [pos_vals]() -> RobotPose {
        RobotPose ret_val;
        ret_val.global.T = {pos_vals[0], pos_vals[1], pos_vals[2]};
        return ret_val;
    };
}

std::function<RobotPose()> CreateCallback(const std::vector<double>& pos_vals,
                                                           const std::vector<long>& tag_ids,
                                                           const std::vector<double>& tag_x,
                                                           const std::vector<double>& tag_y,
                                                           const std::vector<double>& tag_yaw){
    return [pos_vals, tag_ids, tag_x, tag_y, tag_yaw]() -> RobotPose {
        RobotPose ret_val;
        ret_val.global.T = {pos_vals[0], pos_vals[1], pos_vals[2]};
//        ret_val.RelativeTags.AddTag(Pose());

        for (int i=0; i < tag_ids.size(); i++){
            Pose new_tag;
            new_tag.tag_id = (int)tag_ids[i];
            new_tag.robot.T[0] = tag_x[i];
            new_tag.robot.T[1] = tag_y[i];
            new_tag.robot.R = CreateRotationMatrix({0, 0, tag_yaw[i]});

            ret_val.RelativeTags.AddTag(new_tag);
        }
//        AppLogger::Logger::Log("finished add tags callback");
        return ret_val;
    };
}

TEST(NetworkTablesWorker, NTNoServer){
    SetupLogger("NetworkTablesWorker_NTNoServer");

    NTWorker nt_worker(2987);
    nt_worker.RegisterPoseCallback(CreateCallback({1, 2, 3}));
    nt_worker.Start();

    sleep(5);

    // Verify the worker never connected to the server
    ASSERT_FALSE(nt_worker.IsConnected());


    ASSERT_TRUE(nt_worker.Stop());
    AppLogger::Logger::Flush();
}

TEST(NetworkTablesWorker, NTServerRobotRelativeTags){
    SetupLogger("NetworkTablesWorker_NTServerRobotRelativeTags");

    // Create server
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::Create();
    inst.StartServer("../test/networktables.json", "127.0.0.1", NT_DEFAULT_PORT3, NT_DEFAULT_PORT4);
    AppLogger::Logger::Log("Started NetworkTables test server");

    // Verify that nothing has been published to the topic yet
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("MAPLE");
    nt::DoubleArraySubscriber position_sub = table->GetDoubleArrayTopic("position").Subscribe({});
    nt::IntegerArraySubscriber tag_id_sub = table->GetIntegerArrayTopic("robot_relative_tagid").Subscribe({});
    nt::DoubleArraySubscriber pos_x_sub = table->GetDoubleArrayTopic("robot_relative_x").Subscribe({});
    nt::DoubleArraySubscriber pos_y_sub = table->GetDoubleArrayTopic("robot_relative_y").Subscribe({});
    nt::DoubleArraySubscriber pos_yaw_sub = table->GetDoubleArrayTopic("robot_relative_yaw").Subscribe({});


    // Test that the worker can publish at least one message to the NetworkTables server
    std::vector<double> desired_pos = {500, -500, 200};
    std::vector<long> desired_tag_id = {1,2,3};
    std::vector<double> desired_tag_x = {5,5,5};
    std::vector<double> desired_tag_y = {7,8,9};
    std::vector<double> desired_tag_yaw = {0,90,180};

    NTWorker nt_worker("127.0.0.1");
    nt_worker.RegisterPoseCallback(CreateCallback(
            desired_pos,
            desired_tag_id,
            desired_tag_x,
            desired_tag_y,
            desired_tag_yaw));

    nt_worker.Start();

    sleep(5);
    // Verify the server has one connection (the one worker)
    EXPECT_TRUE(inst.GetConnections().size() == 1);
    // Verify the worker thinks it is connected to the server
    EXPECT_TRUE(nt_worker.IsConnected());
    AppLogger::Logger::Log("Server and NTWorker agree the worker is connected to NetworkTables test server");

    sleep(2);

    // Verify that a message was published to the server
    std::vector<double> actual_pos = position_sub.Get(); // get the latest value
    std::vector<long> actual_tag_id = tag_id_sub.Get(); // get the latest value
    std::vector<double> actual_tag_x = pos_x_sub.Get(); // get the latest value
    std::vector<double> actual_tag_y = pos_y_sub.Get(); // get the latest value
    std::vector<double> actual_tag_yaw = pos_yaw_sub.Get(); // get the latest value
    ASSERT_TRUE(actual_pos.size() == 3);
    for (int i=0; i < actual_pos.size(); i++){
        ASSERT_TRUE(actual_pos[i] == desired_pos[i]);
    }

    ASSERT_TRUE(actual_tag_id.size() == 3);
    for (int i=0; i < 3; i++){
        ASSERT_TRUE(actual_tag_id[i] == desired_tag_id[i]);
        ASSERT_TRUE(actual_tag_x[i] == desired_tag_x[i]);
        ASSERT_TRUE(actual_tag_y[i] == desired_tag_y[i]);
        ASSERT_TRUE(actual_tag_yaw[i] == desired_tag_yaw[i]);
    }
    AppLogger::Logger::Log("Server received correct robot relative tag data from NTWorker");

    ASSERT_TRUE(nt_worker.Stop());

    AppLogger::Logger::Flush();
}