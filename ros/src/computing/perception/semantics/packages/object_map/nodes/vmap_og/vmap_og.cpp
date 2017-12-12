/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector_map/vector_map.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

using namespace vector_map;

namespace
{

std::vector<std::vector<geometry_msgs::Point>> g_area_points;

tf::TransformListener* g_tflistenerp;
tf::TransformBroadcaster* g_tfbroadcasterp;


geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  // Transform pose
  tf_pose = tf * tf_pose;

  // Convert TF pose to ROS pose
  geometry_msgs::Pose ros_pose;
  tf::poseTFToMsg(tf_pose, ros_pose);

  return ros_pose;
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Point tf_point;
  tf::pointMsgToTF(point, tf_point);

  // Transform pose
  tf_point = tf * tf_point;

  // Convert TF pose to ROS pose
  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}


void publishMap(const grid_map::GridMap& map, const ros::Publisher& pub)
{
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());  
}

tf::StampedTransform findTransform(const std::string& frame1, const std::string& frame2)
{
  tf::StampedTransform transform;

  try
  {
    // What time should we use?
    g_tflistenerp->lookupTransform(frame1, frame2, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return transform;
  }

  return transform;
}

void createAreaPoints(const vector_map_msgs::Area& area, const vector_map::VectorMap& vmap)
{
  std::vector<geometry_msgs::Point> area_points;

  if (area.aid == 0)
    return;

  vector_map_msgs::Line line = vmap.findByKey(Key<Line>(area.slid));
  // must set beginning line
  if (line.lid == 0 || line.blid != 0)
    return;

  // Search all lines in area
  while (line.flid != 0)
  {
    vector_map_msgs::Point bp = vmap.findByKey(Key<Point>(line.bpid));
    if (bp.pid == 0)
      return;

    Point fp = vmap.findByKey(Key<Point>(line.fpid));
    if (fp.pid == 0)
      return;

    // 2 points of line
    area_points.push_back(convertPointToGeomPoint(bp));
    area_points.push_back(convertPointToGeomPoint(fp));

    line = vmap.findByKey(Key<Line>(line.flid));
    if (line.lid == 0)
      return;
  }

  Point bp = vmap.findByKey(Key<Point>(line.bpid));
  Point fp = vmap.findByKey(Key<Point>(line.fpid));
  if (bp.pid == 0 || fp.pid == 0)
    return;

  area_points.push_back(convertPointToGeomPoint(bp));
  area_points.push_back(convertPointToGeomPoint(fp));

  g_area_points.push_back(area_points);

  return;
}


void applyVmapPolygon(grid_map::GridMap& map)
{
  map["wayarea"].setConstant(100);

  tf::StampedTransform tf = findTransform(map.getFrameId(), "map");

  for (const auto& points : g_area_points)
  {
    // create polygon
    grid_map::Polygon polygon;
    polygon.setFrameId(map.getFrameId());
    for (const auto& p : points)
    {
      // transform to GridMap coordinate
      geometry_msgs::Point tf_point = transformPoint(p, tf);
      polygon.addVertex(grid_map::Position(tf_point.x, tf_point.y));
    }

    // apply polygon...
    for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator)
    {
      map.at("wayarea", *iterator) = 0;
    }
  }

}

void applyVmapCV(grid_map::GridMap& map, const std::vector<std::vector<geometry_msgs::Point>>& area_points_vec)
{
  map["wayarea"].setConstant(100);

  cv::Mat original_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "wayarea", CV_8UC1, 0, 100, original_image);

  // deep copy
  cv::Mat filled_image = original_image.clone();

  // for obtaining vector map coordinate in velodyne frame
  tf::StampedTransform tf = findTransform(map.getFrameId(), "map");

  // calculate map position
  grid_map::Position map_pos = map.getPosition();
  double origin_x_offset = map.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = map.getLength().y() / 2.0 - map_pos.y();

  for (const auto& points : area_points_vec)
  {
    std::vector<cv::Point> cv_points;

    for (const auto& p : points)
    {
      // transform to GridMap coordinate
      geometry_msgs::Point tf_point = transformPoint(p, tf);

      // coordinate conversion for cv image
      double cv_x = (map.getLength().y() - origin_y_offset - tf_point.y) / map.getResolution();
      double cv_y = (map.getLength().x() - origin_x_offset - tf_point.x) / map.getResolution();
      cv_points.emplace_back(cv::Point(cv_x, cv_y));
    }

    // fill polygon
    // NOTE: cv::Scalar(255, 255, 255) is white, and white means 100 in Occupancy Grid of ROS
    //       cv::Scalar(  0,   0,   0) -> 0
    //       cv::Scalar(0) is Okay (1 channel)
    cv::fillConvexPoly(filled_image, cv_points.data(), cv_points.size(), cv::Scalar(0));
  }

  // visualize the image
  /*
  cv::namedWindow("Original", 0);
  cv::namedWindow("Filled"  , 0);

  cv::imshow("Original", original_image);
  cv::imshow("Filled"  , filled_image);

  cv::waitKey(1);
  */

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(filled_image, "wayarea", map, 0, 100);
}


// Callback
void convertOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg, const ros::Publisher& pub)
{
  // timer start
  auto start = std::chrono::system_clock::now();

  ROS_INFO("Subscribed Occupancy Grid Map");

  // convert ROS OccupancyGrid to GridMap

  static grid_map::GridMap map({"original", "wayarea", "points_wayarea"});
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "original", map);

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  // vmap polygon
  /*
  if (!g_polygons.empty())
  {
    ROS_INFO("Use wayarea");
    applyVmapPolygon(map);
    map["dist_vmap"] = map["distance_transform"] + map["wayarea"];
  }
  */

  // vmap polygon CV
  bool use_wayarea = false;
  if (!g_area_points.empty())
  {
    ROS_INFO_STREAM("Use wayarea");
    applyVmapCV(map, g_area_points);
    map["points_wayarea"] = map["original"] + map["wayarea"];

    use_wayarea = true;
  }

  // publish grid map as ROS message
  publishMap(map, pub);

  // timer end
  auto end = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "ogm_vmap");
  ros::NodeHandle nh("~");
  ros::NodeHandle private_nh("~");

  // TODO: define in class
  tf::TransformListener tf_listener;
  g_tflistenerp = &tf_listener;
  tf::TransformBroadcaster tf_broadcaster;
  g_tfbroadcasterp = &tf_broadcaster;
  

  std::string map_topic;
  private_nh.param<std::string>("/ogm_vmap/map_topic", map_topic, "/realtime_cost_map");

  ros::Publisher grid_map_pub    = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher another_map_pub = nh.advertise<grid_map_msgs::GridMap>("another_grid_map", 1, true);
  
  ros::Subscriber ogm_sub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, std::bind(convertOccupancyGrid, std::placeholders::_1, grid_map_pub));


  // vector map
  // enable Non blocking?
  vector_map::VectorMap vmap;
  vmap.subscribe(nh, Category::POINT | Category::LINE | Category::AREA | Category::WAY_AREA);

  // all true -> all data
  std::vector<vector_map_msgs::WayArea> way_areas = vmap.findByFilter([](const vector_map_msgs::WayArea& way_area){return true;});

  if (way_areas.empty())
  {
    ROS_WARN_STREAM("ogm_vmap: No WayArea!");
  }

  for (const auto& way_area : way_areas)
  {
    vector_map_msgs::Area area = vmap.findByKey(Key<Area>(way_area.aid));
    createAreaPoints(area, vmap);
  }


  ros::spin();

  return 0;
}
