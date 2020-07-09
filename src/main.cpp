
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include <ros/ros.h>
#include <fusion_data.h>
#include <source_data.h>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;



int main(int argc, char* argv[]) {
  // ROS节点初始化
  ros::init(argc, argv, "ukf_fusion_publisher");
    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/odom_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher fusion_info_pub = n.advertise<sensor_fusion::fusion_data>("/ukf_fusion_info", 10);
    ros::Publisher source_info_pub = n.advertise<sensor_fusion::source_data>("/ukf_orgin_data", 10);
    // 设置循环的频率
    ros::Rate loop_rate(10);

 // 设置毫米波雷达/激光雷达输入数据的路径
    // Set radar & lidar data file path
    std::string input_file_name = "../data/sample-laser-radar-measurement-data-1.txt";

    // 打开数据，若失败则输出失败信息，返回-1，并终止程序
    // Open file. if failed return -1 & end program
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if (!input_file.is_open()) {
        std::cout << "Failed to open file named : " << input_file_name << std::endl;
        return -1;
    }

    // 分配内存
    // measurement_pack_list：毫米波雷达/激光雷达实际测得的数据。数据包含测量值和时间戳，即融合算法的输入。
    // groundtruth_pack_list：每次测量时，障碍物位置的真值。对比融合算法输出和真值的差别，用于评估融合算法结果的好坏。
    //std::vector<MeasurementPackage> measurement_pack_list;
    //std::vector<GroundTruthPackage> groundtruth_pack_list;

    // 通过while循环将雷达测量值和真值全部读入内存，存入measurement_pack_list和groundtruth_pack_list中
    // Store radar & lidar data into memory

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(input_file, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values_ = VectorXd(4);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
  }
      //开始部署ukf融合算法
      // Create a UKF instance
      UKF ukf;
      // used to compute the RMSE later
      vector<VectorXd> estimations;
      vector<VectorXd> ground_truth;
      // start filtering from the second frame (the speed is unknown in the first
      // frame)
       sensor_fusion::fusion_data msg;
       sensor_fusion::source_data msg2;

  while (ros::ok())
   {
      
      size_t number_of_measurements = measurement_pack_list.size();
      for (size_t k = 0; k < number_of_measurements; ++k) 
      {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // output the estimation
       // cout<<"x："<< ukf.x_(0) << endl; // pos1 - est
        //cout<<"y:"<< ukf.x_(1) << endl; // pos2 - est
        //cout << "vel:"<<ukf.x_(2) <<endl; // vel_abs -est
       // cout<< "yaw_angle:"<<ukf.x_(3) << endl; // yaw_angle -est
       //cout <<"yaw_rate:"<< ukf.x_(4) << endl; // yaw_rate -est

        VectorXd ukf_x_cartesian_ = VectorXd(4);

        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
       msg.x=x_estimate_;
       msg.y=y_estimate_;
       msg.vx=vx_estimate_;
       msg.vy=vy_estimate_;
       //发送fusion data
       fusion_info_pub.publish(msg);
        // output the ground truth packages
       // cout <<"x:"<< gt_pack_list[k].gt_values_(0) << "\t";
       // cout <<"y:"<< gt_pack_list[k].gt_values_(1) << "\t";
       // cout<< "vx:"<<gt_pack_list[k].gt_values_(2) << "\t";
       // cout <<"vy："<< gt_pack_list[k].gt_values_(3) << "\t";
        //原始数据发送
       msg2.x=gt_pack_list[k].gt_values_(0);
       msg2.y=gt_pack_list[k].gt_values_(1);
       msg2.vx=gt_pack_list[k].gt_values_(2);
       msg2.vy=gt_pack_list[k].gt_values_(3);
       //发送source data
       source_info_pub.publish(msg2);
       ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
        
        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
        loop_rate.sleep();
  }
  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
  cout << "Done!" << endl;
  }
  //return 0;
}
