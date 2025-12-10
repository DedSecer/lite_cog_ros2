//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>
#include <chrono>

#include "laser_mapping.h"

using namespace std::chrono_literals;

/// run the lidar mapping in online mode

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    std::cout << "catch sig " << sig << std::endl;
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    // google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS();

    signal(SIGINT, SigHandle);
    rclcpp::WallRate loop_rate(1ms);

    LOG(INFO) << "starting loop";

    // online, almost same with offline, just receive the messages from ros
    while (rclcpp::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        rclcpp::spin_some(laser_mapping);
        laser_mapping->Run();
        loop_rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
