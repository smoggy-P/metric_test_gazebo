#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <string.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <numeric>
#include <visualization_msgs/Marker.h>
using namespace std;

ros::Subscriber sub;
ros::Publisher pub, pub_pedestrain;
ros::Time time0;
bool msg_received = false;

vector<geometry_msgs::Pose> drone_poses;
vector<float> map_bounds;
vector<float> time_survived_list;

float d_sample, t_max, r_robot;

bool isActor(string name) {
  if (char(name[0]) == 'a' && char(name[1]) == 'c') {
    return true;
  }
  return false;
}

void getParams(ros::NodeHandle n) {
  if (n.getParam("/metric_test/map_bounds", map_bounds)) {
    ROS_INFO("get param map_bounds: %f, %f, %f, %f", map_bounds[0], map_bounds[1], map_bounds[2], map_bounds[3]);
  }
  else {
    ROS_INFO("fail to get param map_bounds, set to default");
    map_bounds = {-2, 23, -7, 7};
  }

  if (n.getParam("/metric_test/d_sample", d_sample)) {
    ROS_INFO("get param d_sample: %f", d_sample);
  }
  else {
    ROS_INFO("fail to get param d_sample, set to default");
    d_sample = 6;
  }

  if (n.getParam("/metric_test/t_max", t_max)) {
    ROS_INFO("get param t_max: %f", t_max);
  }
  else {
    ROS_INFO("fail to get param t_max, set to default");
    t_max = 10;
  }

  if (n.getParam("/metric_test/r_robot", r_robot)) {
    ROS_INFO("get param r_robot: %f", r_robot);
  }
  else {
    ROS_INFO("fail to get param r_robot, set to default");
    r_robot = 1.0;
  }
}

void stateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
  visualization_msgs::Marker marker_pedestrian;
  marker_pedestrian.header.frame_id = "map";
  marker_pedestrian.header.stamp = ros::Time();
  marker_pedestrian.type = visualization_msgs::Marker::SPHERE_LIST;
  geometry_msgs::Vector3 scale;
  scale.x = 0.2;
  scale.y = 0.2;
  scale.z = 0.2;
  marker_pedestrian.scale = scale;

  for (auto itr=msg->name.begin(); itr!=msg->name.end(); itr++) {
    if (isActor(*itr)) {
      geometry_msgs::Point point;
      point.x = msg->pose[itr - msg->name.begin()].position.x;
      point.y = msg->pose[itr - msg->name.begin()].position.y;
      point.z = 0;
      marker_pedestrian.points.push_back(point);
      std_msgs::ColorRGBA color;
      color.r = 1;
      color.g = 0;
      color.b = 0;
      color.a = 1;
      marker_pedestrian.colors.push_back(color);
    }
  }
  pub_pedestrain.publish(marker_pedestrian);

  
  if (!msg_received) {
    msg_received = true;
    ROS_INFO("msg received");
    time0 = ros::Time::now();
  }
  else{
    if (ros::Time::now().toSec() - time0.toSec() > t_max) {
      ROS_INFO("time up");
      ROS_INFO("average time_survived: %f", std::accumulate(time_survived_list.begin(), time_survived_list.end(), 0.0) / time_survived_list.size());
      ros::shutdown();
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    geometry_msgs::Vector3 scale;
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;
    marker.scale = scale;
    vector<std_msgs::ColorRGBA> colors;
    for (auto itr = time_survived_list.begin(); itr != time_survived_list.end(); itr++) {
      std_msgs::ColorRGBA color;
      color.r = 0;
      color.g = 0;
      color.b = 1;
      color.a = 1;
      colors.push_back(color);
    }

    for (auto itr = msg->name.begin(); itr != msg->name.end(); itr++) {
      if (isActor(*itr)) {
        for (auto itr2 = drone_poses.begin(); itr2 != drone_poses.end(); itr2++) {
          float x_robot = itr2->position.x;
          float y_robot = itr2->position.y;
          float x_actor = msg->pose[itr - msg->name.begin()].position.x;
          float y_actor = msg->pose[itr - msg->name.begin()].position.y;
          float dist = sqrt(pow(x_robot - x_actor, 2) + pow(y_robot - y_actor, 2));
          if (dist < r_robot) {
            float time_survived = ros::Time::now().toSec() - time0.toSec();
            if (time_survived < time_survived_list[itr2 - drone_poses.begin()]) {
              time_survived_list[itr2 - drone_poses.begin()] = time_survived;
            }
            colors[itr2 - drone_poses.begin()].r = 1;
            colors[itr2 - drone_poses.begin()].b = 0;
          }
        }
      }
    }
    for (auto itr = drone_poses.begin(); itr != drone_poses.end(); itr++) {
      geometry_msgs::Point point;
      point.x = itr->position.x;
      point.y = itr->position.y;
      point.z = 0;
      marker.points.push_back(point);
      marker.colors.push_back(colors[itr - drone_poses.begin()]);
    }
    pub.publish(marker);
  }

  
  // for (auto itr = time_survived_list.begin(); itr != time_survived_list.end(); itr++) {
  //     ROS_INFO("time_survived: %f", *itr);
  //   }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "survivability_calculator");
  ros::NodeHandle n;
  getParams(n);

  for (float x_robot=map_bounds[0]; x_robot<=map_bounds[1]; x_robot+=d_sample) {
    for (float y_robot=map_bounds[2]; y_robot<=map_bounds[3]; y_robot+=d_sample) {
      geometry_msgs::Pose drone_pose;
      drone_pose.position.x = x_robot;
      drone_pose.position.y = y_robot;
      drone_poses.push_back(drone_pose);
      time_survived_list.push_back(t_max);
    }
  }

  sub = n.subscribe("/gazebo/model_states", 100, stateCB);
  pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pub_pedestrain = n.advertise<visualization_msgs::Marker>("pedestrain_marker", 10);

  ros::spin();
  return 0;
}