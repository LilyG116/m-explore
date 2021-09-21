#ifndef LILY_EXPLORE_H_
#define LILY_EXPLORE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/explore.h>

#include <lily_explore/my_msg.h>
struct Point {
    float x;
    float y;
};

namespace explore
{
class assigner
{
private:
//tf::TransformListener listener;

public:


double dist(geometry_msgs::Point& frontier, geometry_msgs::Point& pos);

//		
void add_ftr_list(std::vector<geometry_msgs::Point> new_ftrs, int size);

//Gets frontier lists from the individual robots and saves them, sets frontier flags to true
void clbk_r1(const lily_explore::my_msg::ConstPtr& msg_r1);
void clbk_r2(const lily_explore::my_msg::ConstPtr& msg_r2);
void clbk_r3(const lily_explore::my_msg::ConstPtr& msg_r3);

//Method checks against known maps for already known frontier information
bool unknown(double x, double y);

//Call backs for map updates, update flags to true and save updated maps
void clbk_map1(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
void clbk_map2(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
void clbk_map3(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

//Initialize nodehandles, subscribers to frontierlists, and map topics, and publishing to goal topics, intialize flags to false 
void initialize();
Point point; 

//new


};
}
#endif
