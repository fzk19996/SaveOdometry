// This Programme aims at changing odom topic form ros to Kitti ground-truth format
// Created by hyx on 2020/9/9.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

int flag;
fstream file;

void callback(const nav_msgs::OdometryConstPtr &odom) {
    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Matrix3d E(Eigen::MatrixXd::Identity(3, 3));
    if (flag == 0) R_transform << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    else R_transform << -1, 0, 0, 0, -1, 0, 0, 0, 1;

    Eigen::Quaterniond q_transform(R_transform);

    Eigen::Quaterniond quaternion4;
    quaternion4.w() = odom->pose.pose.orientation.w;
    quaternion4.x() = odom->pose.pose.orientation.x;
    quaternion4.y() = odom->pose.pose.orientation.y;
    quaternion4.z() = odom->pose.pose.orientation.z;
    Eigen::Quaterniond q = q_transform * quaternion4;

    Eigen::Vector3d poses;
    poses[0] = odom->pose.pose.position.x;
    poses[1] = odom->pose.pose.position.y;
    poses[2] = odom->pose.pose.position.z;
    Eigen::Vector3d t = q_transform * poses;

    Eigen::Matrix3d rotation_M3d;
    rotation_M3d = q.toRotationMatrix();

    Eigen::Matrix<double, 3, 4> p;
    p.topRightCorner(3, 3) = rotation_M3d;
    p.col(3) = t;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++)
            if (i == 2 && j == 3) file << p(i, j) << endl;
            else file << p(i, j) << " ";
    }
    cout << "p:" << endl << p << endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "GetOdom");
    cout << "Start..." << endl;
    ros::NodeHandle nh("~");

    string topic, savePath;
    nh.getParam("SavePath", savePath);
    nh.getParam("OdomTopic", topic);
    nh.getParam("DifferentTransformMatrix", flag);

    string Path = "/home/hyx/" + savePath;
    cout<<Path<<endl;
    file.open(Path, ios::out);

    ros::Subscriber odom = nh.subscribe(topic, 100, callback);

    ros::spin();

    file.close();
    cout<<"Done! \n";

    return 0;
}