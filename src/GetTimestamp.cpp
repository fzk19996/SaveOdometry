//
// Created by hyx on 2020/9/20.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen3/Eigen/Eigen>

using namespace std;

int frame_id = 0;
fstream file;
long first_sec, first_nsec;

void OdomHandler(const nav_msgs::OdometryConstPtr &odom) {
    if (frame_id == 0) {
        first_sec = odom->header.stamp.sec;
        first_nsec = odom->header.stamp.nsec;
        frame_id = 1;
    }

    int sec, nsec;
    sec = odom->header.stamp.sec - first_sec;
    nsec = odom->header.stamp.nsec - first_nsec;
    if (nsec < 0) {
        nsec += 10e9;
        sec--;
    }

    Eigen::VectorXd p;

    Eigen::Vector3d pose;

    pose[0] = odom->pose.pose.position.x;
    pose[1] = odom->pose.pose.position.y;
    pose[2] = odom->pose.pose.position.z;

    Eigen::Quaterniond q;
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q.z() = odom->pose.pose.orientation.w;

    cout << sec << '.' << nsec << " " << pose << endl;

//    file<<sec<<'.'<<nsec<<" "<<p<<endl;



}

int main(int argc, char **argv) {
    ros::init(argc, argv, "GetTimestamp");
    cout << "Start recording..." << endl;
    ros::NodeHandle nh("~");

    string topic, savePath;
    nh.getParam("SavePath", savePath);
    nh.getParam("OdomTopic", topic);
    string Path = "/home/hyx/" + savePath;

    file.open(Path, ios::out);
    ros::Subscriber odom = nh.subscribe(topic, 100, OdomHandler);
    ros::Subscriber gt = nh.subscribe(topic, 100, OdomHandler);


    ros::spin();
    file.close();
    return 0;
}