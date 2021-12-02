#ifndef LILY_EXPLORE_H_
#define LILY_EXPLORE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/explore.h>

#include <lily_explore/my_msg.h>
#include <lily_explore/robotlist.h>
struct Point {
    float x;
    float y;
};
struct kPoint {
    double x, y;     // coordinates
    int cluster;     // no default cluster
    double minDist, maxDist;  // default infinite dist to nearest cluster

    kPoint() : 
        x(0.0), 
        y(0.0),
        cluster(-1),
        maxDist(0),
        minDist(__DBL_MAX__) {}
        
    kPoint(double x, double y) : 
        x(x), 
        y(y),
        cluster(-1),
        maxDist(0),
        minDist(__DBL_MAX__) {}

    double distance(kPoint p) {
        return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
    }
};
namespace explore
{
class assigner
{
private:
//tf::TransformListener listener;

public:



//		
void add_ftr_list(std::vector<geometry_msgs::Point> new_ftrs, int size);

//Gets frontier lists from the individual robots and saves them, sets frontier flags to true
void clbk_r1(const lily_explore::my_msg::ConstPtr& msg_r1);
void clbk_r2(const lily_explore::my_msg::ConstPtr& msg_r2);
void clbk_r3(const lily_explore::my_msg::ConstPtr& msg_r3);

//Method checks against known maps for already known frontier information
bool unknown(double x, double y);

//Call backs for map updates, update flags to true and save updated maps
static void clbk_map1(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
static void clbk_map2(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
static void clbk_map3(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

//Initialize nodehandles, subscribers to frontierlists, and map topics, and publishing to goal topics, intialize flags to false 
void initialize();
Point point; 

//new


};
}
#endif
