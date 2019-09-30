#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "graph_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward
{
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;

// useful global variables
bool _has_map = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher _grid_path_vis_pub, _debug_nodes_vis_pub, _closed_nodes_vis_pub, _open_nodes_vis_pub, _close_nodes_sequence_vis_pub, _grid_map_vis_pub;

gridPathFinder *_path_finder = new gridPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

void visDebugNodes(vector<Vector3d> nodes);
void visGridPath(vector<Vector3d> &nodes, bool is_use_jps);
void visCloseNode(vector<Vector3d> nodes);
void visOpenNode(vector<Vector3d> nodes);
void visCloseNodeSequence(vector<Vector3d> nodes);
void displaySphereList(vector<Vector3d> & list, double resolution, Vector4d color, int id);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt); //待补充

void rcvWaypointsCallback(const nav_msgs::Path &wp)
{
    if (wp.poses[0].pose.position.z < 0.0 || _has_map == false)
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
        wp.poses[0].pose.position.y,
        wp.poses[0].pose.position.z;

    ROS_INFO("[jps_node] receive the way-points");
    _path_finder->graphSearch(_start_pt, target_pt, true);
    vector<Vector3d> path_jps = _path_finder->getPath();
    // visGridPath(path_jps, true);
    displaySphereList(path_jps, _resolution, Eigen::Vector4d(1, 0, 0, 1.0), 1);
    _path_finder->resetUsedGrids();

    _path_finder->graphSearch(_start_pt, target_pt, false);
    vector<Vector3d> path_astar = _path_finder->getPath();
    // visGridPath(path_astar, false);
    displaySphereList(path_astar, _resolution, Eigen::Vector4d(0, 1, 0, 1.0), 2);
    _path_finder->resetUsedGrids();
}

// 收到了点云地图
//　转换成grid　发布出去
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
    static sensor_msgs::PointCloud2 map_vis;

    if (_has_map) //　说明此例中地图是静态的，只订阅一次s
    {
        _grid_map_vis_pub.publish(map_vis);
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    if ((int)cloud.points.size() == 0)
        return;

    pcl::PointXYZ pt, pt_inf;
    int inf_step = round(_cloud_margin * _inv_resolution);
    int inf_step_z = max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];
        for (int x = -inf_step; x <= inf_step; x++)
        {
            for (int y = -inf_step; y <= inf_step; y++)
            {
                for (int z = -inf_step_z; z <= inf_step_z; z++)
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;
                    _path_finder->setObs(inf_x, inf_y, inf_z);

                    Vector3d cor_inf = _path_finder->coordRounding(Vector3d(inf_x, inf_y, inf_z));

                    pt_inf.x = cor_inf(0);
                    pt_inf.y = cor_inf(1);
                    pt_inf.z = cor_inf(2);
                    cloud_vis.points.push_back(pt_inf);
                }
            }
        }
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

/*
* demo node
*   -订阅　map 
*   -订阅　waypoints 目标点
*   
*   - 发布：
    点云　grid_map_vis
    visual path: grid_path_vis
    open_node
    close_node
    debug_node
    close_nodes_sequence_vis
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
    _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

    _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _debug_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("debug_nodes_vis", 1);
    _closed_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("closed_nodes_vis", 1);
    _open_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("open_nodes_vis", 1);
    _close_nodes_sequence_vis_pub = nh.advertise<visualization_msgs::Marker>("close_nodes_sequence_vis", 10);

    nh.param("map/cloud_margin", _cloud_margin, 0.0);
    nh.param("map/resolution", _resolution, 0.2); // gridmap 分辨率

    nh.param("map/x_size", _x_size, 50.0); //　grid map　size　米
    nh.param("map/y_size", _y_size, 50.0);
    nh.param("map/z_size", _z_size, 5.0);

    nh.param("planning/start_x", _start_pt(0), 0.0); //   起点的坐标
    nh.param("planning/start_y", _start_pt(1), 0.0);
    nh.param("planning/start_z", _start_pt(2), 0.0);

    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0; //　地图size的最小值和最大值 米
    _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;

    _inv_resolution = 1.0 / _resolution;

    _max_x_id = (int)(_x_size * _inv_resolution); //地图 gridmap cell　的最大值
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    //实例化一个路径规划的类
    _path_finder = new gridPathFinder();
    _path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    delete _path_finder;
    return 0;
}

void visDebugNodes(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/debug_info";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _debug_nodes_vis_pub.publish(node_vis);
}

void visGridPath(vector<Vector3d> &nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    if (is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;//
    node_vis.action = visualization_msgs::Marker::ADD;//
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if (is_use_jps)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}
void displaySphereList(vector<Vector3d> & list, double resolution, Vector4d color, int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  _grid_path_vis_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  _grid_path_vis_pub.publish(mk);
}

void visCloseNode(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/closed_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _closed_nodes_vis_pub.publish(node_vis);
}

void visOpenNode(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/visited_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.3;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _open_nodes_vis_pub.publish(node_vis);
}

void visCloseNodeSequence(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/animation_of_close_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        usleep(50000);
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
        _close_nodes_sequence_vis_pub.publish(node_vis);
    }
}
