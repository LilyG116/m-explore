//Lilys method for subscribing to all of the frontier lists and then assigning them to the nearest robots, greedy assignment

//#include <explore/Lily_Assigner.h>
#include <tgmath.h>
#include <thread>
#include <sstream>
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <lily_explore/my_msg.h>
#include "explore/Lily_Assigner.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <tf/transform_listener.h>
#include <explore/frontier_search.h>
#include <explore/costmap_client.h>
#include <mutex>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <explore/explore.h>
#include <explore/costmap_tools.h>
#include <visualization_msgs/MarkerArray.h>
#include <explore/costmap_2d_ros.h>
#include <vector>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>



std::vector<geometry_msgs::Point> ftr_r1;
std::vector<geometry_msgs::Point> ftr_r2;
std::vector<geometry_msgs::Point> ftr_r3;
std::vector<geometry_msgs::Point> ftr_list_new;
std::vector<geometry_msgs::Point> ftr_list;
bool flagr1, flagr2, flagr3, flag4, flag5, flag6, uknown;
int msr1_size, msr2_size, msr3_size;

ros::Publisher marker_array_publisher_;
ros::Publisher goals_r1;
ros::Publisher goals_r2;
ros::Publisher goals_r3;
ros::Subscriber sub_r1;
ros::Subscriber sub_r2;
ros::Subscriber sub_r3;
ros::Subscriber map_r1;
ros::Subscriber map_r2;
ros::Subscriber map_r3;
geometry_msgs::Point pos_r1;
geometry_msgs::Point pos_r2;
geometry_msgs::Point pos_r3;
lily_explore::my_msg msg_r1;
lily_explore::my_msg msg_r2;
lily_explore::my_msg msg_r3;
nav_msgs::OccupancyGrid map1;
nav_msgs::OccupancyGrid map2;
nav_msgs::OccupancyGrid map3;



namespace explore
{


double dist(geometry_msgs::Point& frontier, geometry_msgs::Point& pos)
{
    

    double dx = frontier.x - pos.x;
    double dy = frontier.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);
    return d;
}

bool unknown(double x, double y){

   int grid_x2 = (x - map2.info.origin.position.x)/map2.info.resolution ;//
   int grid_y2 = (y - map2.info.origin.position.y)/map2.info.resolution;//
   int ind1 = grid_x2 + grid_y2*map1.info.width;

   ROS_INFO("conversion %f, %f -> %d, %d", x, y, grid_x2, grid_y2); 
   ROS_INFO("Unknown point %d, %d, %d", map1.data[ind1], map2.data[ind1], map3.data[ind1]);

   int ind, count1, count2, count3;
   count1 = 0; 
   count2 = 0;
   count3 = 0;
   for (int i = grid_x2 -2; i <= grid_x2 +2; i++)
   {
     for (int j = grid_y2 -2; j <= grid_y2 +2; j++)
     {
      ind = i + j*map1.info.width;
      if(map1.data[ind]== -1){
        count1 = count1 +1;
      }  
      if(map2.data[ind]== -1){
        count2 = count2 +1;
      }
      if(map3.data[ind]== -1){
        count3 = count3 +1;
      } 
     }
  }
  ROS_INFO("Unknown point %d, %d, %d", count1, count2, count3);
  if(count1>9  && count2>9 && count3>9){
       ROS_INFO("Added");
       return false;
  }   
  else{
       return true;
  } 
}

void add_ftr_list(std::vector<geometry_msgs::Point> new_ftrs, int size)
{
  
  //initialize some variable destroyed every call
  tf::TransformListener listener;
  //ROS_INFO("new_ftrs: %lx", new_ftrs.size());
  //ROS_INFO("ftrs_list: %lx", ftr_list.size());
  int init_list_size = ftr_list.size();
  bool flag = false;
  
  if (ftr_list.size() == 0)
  {
     for (int i=0; i < size; i++)
     {
     ftr_list.push_back(new_ftrs[i]);
     ftr_list_new.push_back(new_ftrs[i]);
     //ROS_INFO("ftr list x:%f y: %f", ftr_list[i].x, ftr_list[i].y);
     }
  ROS_INFO("initialize list");
  }
  else{
  //Filter for frontiers within +/-0.2 m different in x or y direction to add to global list only
 
  //ROS_INFO("ftr_new size: %lx", ftr_list_new.size());
  for (int h = 0; h < size; h++)
  {
  flag = false;
    for (int j= 0; j< init_list_size; j++)
    {
     //if (ftr_list_new[j].x-0.1 <=new_ftrs[h].x <=ftr_list_new[j].x+0.1 && ftr_list_new[j].y-0.1 <=new_ftrs[h].y <=ftr_list_new[j].y+0.1 )
      if (ftr_list[j].x ==new_ftrs[h].x && ftr_list[j].y ==new_ftrs[h].y)
      {
       flag = true;
      }
      }
     if (flag == false)
      {
        ftr_list.push_back(new_ftrs[h]);
        ftr_list_new.push_back(new_ftrs[h]);
        //ROS_INFO("ftr list x:%f y: %f", ftr_list[h].x, ftr_list[h].y);
      }
    }
   }
   if (flag4 && flag5 &&flag6){
     ROS_INFO("ftr_list size before removing known: %lx", ftr_list.size());
     int count = ftr_list.size();
     int ii = 0;
    for (int j= 0; j< count; j++){
        //bool ffa = unknown(ftr_list[ii].x, ftr_list[ii].y) ;
        //printf("Unknown = %d\n", ffa);
        if (unknown(ftr_list[ii].x, ftr_list[ii].y)){
         ftr_list.erase(ftr_list.begin() + ii);
          
        }
        else {
          ii++;
        }
      }
     ROS_INFO("ftr_list size after removing known: %lx", ftr_list.size());
    }
  
    //Print the number of global frontiers
    //ROS_INFO("Number of frontiers on global list and new frontiers: %lx, %lx", ftr_list.size(), ftr_list_new.size());
   
   //sort to lowest cost choices for goal allocation
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;   
  
  for (int kk = 0; kk<ftr_list.size(); kk++)
  {
     marker.header.frame_id = "robot_1/map";
     marker.header.stamp = ros::Time();
     marker.ns = "explore";
     marker.lifetime = ros::Duration(5.0);
     marker.id = kk;
     marker.type = visualization_msgs::Marker::SPHERE;
     marker.action = visualization_msgs::Marker::ADD;
     marker.pose.position.x = ftr_list[kk].x;
     marker.pose.position.y = ftr_list[kk].y;
     marker.pose.position.z = ftr_list[kk].z;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     marker.scale.x = 0.1;
     marker.scale.y = 0.1;
     marker.scale.z = 0.1;
     marker.color.a = 1.0; // Don't forget to set the alpha!
     marker.color.r = 0.0;
     marker.color.g = 1.0;
     marker.color.b = 0.0;
     markers.push_back(marker);
  }
 
  marker_array_publisher_.publish(markers_msg);
  markers_msg.markers.clear();

  tf::StampedTransform posr1;
  tf::StampedTransform posr2;
  tf::StampedTransform posr3;
  std::vector<Point> gr1_Points;
  Point gr1_point; 
  lily_explore::my_msg msg_gr1;
  std::vector<Point> gr2_Points;
  Point gr2_point; 
  lily_explore::my_msg msg_gr2;
  std::vector<Point> gr3_Points;
  Point gr3_point; 
  gr1_Points.clear();
  gr2_Points.clear();
  gr3_Points.clear();

  if(flagr1 && flagr2 && flagr3 ){
    int ftr_size = ftr_list.size();
    listener.waitForTransform("/map", "/robot_1/base_link", ros::Time::now(), ros::Duration(4.0));
    listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), posr1);
    listener.waitForTransform("/map", "/robot_2/base_link", ros::Time::now(),  ros::Duration(4.0));
    listener.lookupTransform("/map", "/robot_2/base_link", ros::Time(0), posr2);
    listener.waitForTransform("/map", "/robot_3/base_link", ros::Time::now(), ros::Duration(4.0));
    listener.lookupTransform("/map", "/robot_3/base_link", ros::Time(0), posr3);
    double distances[3][ftr_size];
    for (int p=0; p < ftr_size; p++){

    double dex = ftr_list[p].x-posr1.getOrigin().x();
    double dey = ftr_list[p].y-posr1.getOrigin().y();
    distances[0][p] = (sqrt(dex*dex +dey*dey));
    dex = ftr_list[p].x-posr2.getOrigin().x();
    dey = ftr_list[p].y-posr2.getOrigin().y();
    distances[1][p] = (sqrt(dex*dex +dey*dey));
    dex = ftr_list[p].x-posr3.getOrigin().x();
    dey = ftr_list[p].y-posr3.getOrigin().y();
    distances[2][p] = (sqrt(dex*dex +dey*dey));

   }
 
    //ROS_INFO("Distances %f, %f, %f,", distances[0],distances[1], distances[2]);
   //for(int yy =0; yy<ftr_size; ){
   if (ftr_list.size()>= 3){
   for (int g= 0; g<3; g++){
    int index = 10;
    double value= 100000;
    
    if (g == 0){
     for(int jk= 0; jk<ftr_list.size(); jk++){
      if(distances[0][jk]<value &&distances[0][jk]!= -1){
        index = jk;
        value = distances[0][jk];
        ROS_INFO("%f", distances[0][jk]);
      }}
      gr1_point.x = ftr_list[index].x; 
      gr1_point.y = ftr_list[index].y; 
      gr1_Points.push_back(gr1_point);
      distances[0][index]= -1;
      distances[1][index]= -1;
      distances[2][index]= -1;
      //yy++;
      //if (yy>= ftr_size) {break;}
      ROS_INFO("index with smallest dist: %d, gr1 size %lx, point %f, %f ", index,     gr1_Points.size(), gr1_point.x, gr1_point.y);
    }
    if (g ==1){
      for(int jk= 0; jk<ftr_list.size(); jk++){
      if(distances[1][jk]<value &&distances[1][jk]!= -1){
        index = jk;
        value = distances[1][jk];
        ROS_INFO("%f", distances[1][jk]);
      }}
      gr2_point.x = ftr_list[index].x; 
      gr2_point.y = ftr_list[index].y; 
      gr2_Points.push_back(gr2_point);
      distances[0][index]= -1;
      distances[1][index]= -1;
      distances[2][index]= -1;
      //yy++;
      //if (yy>= ftr_size) {break;}
      ROS_INFO("index with smallest dist: %d, gr2 size %lx, point %f, %f ", index,     gr2_Points.size(), gr2_point.x, gr2_point.y);
    }
    if (g ==2 ){
     for(int jk= 0; jk<ftr_list.size(); jk++){
      if(distances[2][jk]<value &&distances[2][jk]!= -1){
        index = jk;
        value = distances[2][jk];
        ROS_INFO("%f", distances[2][jk]);
      }}
      gr3_point.x = ftr_list[index].x; 
      gr3_point.y = ftr_list[index].y; 
      gr3_Points.push_back(gr3_point);
      distances[0][index]= -1;
      distances[1][index]= -1;
      distances[2][index]= -1;
      //yy++;
      //if (yy>= ftr_size) {break;}
      ROS_INFO("index with smallest dist: %d, gr3 size %lx, point %f, %f ", index,     gr3_Points.size(), gr3_point.x, gr3_point.y);
    }
     
  }
  }
  int index = 0;
  double value = 1000;
  for (int yy= 3; yy<ftr_size; yy++){
     for(int jk= 0; jk<3; jk++){
      if(distances[jk][yy]<value &&distances[jk][yy]!= -1){
        index = jk;
        value = distances[jk][yy];
      }}
  if (index ==0){
      gr1_point.x = ftr_list[yy].x; 
      gr1_point.y = ftr_list[yy].y; 
      gr1_Points.push_back(gr1_point);  
  }
  if (index ==1){
      gr2_point.x = ftr_list[yy].x; 
      gr2_point.y = ftr_list[yy].y; 
      gr2_Points.push_back(gr2_point);
  }
    if (index ==2){
      gr3_point.x = ftr_list[yy].x; 
      gr3_point.y = ftr_list[yy].y; 
      gr3_Points.push_back(gr3_point);
  }
     ROS_INFO("index with smallest dist: %d, point %f, %f ", index, ftr_list[yy].x, ftr_list[yy].y);
  }
 // }
  
  
  // loop control
  ftr_list_new.clear();
  //ftr_list.clear();
  msg_r1.points.clear();
  msg_r2.points.clear();
  msg_r3.points.clear();
  
  ROS_INFO("Fronteirs for robot 1, 2, 3: %lx, %lx, %lx", gr1_Points.size(), gr2_Points.size(), gr3_Points.size());
 if (gr1_Points.size() != 0){
  int size = gr1_Points.size();
  Point points[size];
  Point point;
  for (int i = 0; i< size; i++){
    point.x = gr1_Points[i].x;
    point.y = gr1_Points[i].y;
    points[i] = point;
   //ROS_INFO("robot 1 x:%f y: %f", point.x, point.y);
  }

  int num = 0;
  for (int i = 0; i<gr1_Points.size(); i++) {
        geometry_msgs::Point point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = 0;
        msg_r1.points.push_back(point);
        //ROS_INFO("robot 1 x:%f y: %f", point.x, point.y);
        num++;
  }
  msg_r1.size = num;
  goals_r1.publish(msg_r1);
  num = 0;
  }

  if (gr2_Points.size() != 0){
  int size = gr2_Points.size();
  Point points[size];
  Point point;
  for (int i = 0; i< size; i++){
    point.x = gr2_Points[i].x;
    point.y = gr2_Points[i].y;
    points[i] = point;
    //ROS_INFO("robot 2 x:%f y: %f", point.x, point.y);
  }
  //std::vector<Point> my_vector(points, points + sizeof(points) / sizeof(points[0]));
//ROS_INFO("vector size %lx, points size %lx", my_vector.size(), sizeof(points));
  int num = 0;
  for (int i = 0; i<gr2_Points.size(); i++) {
        geometry_msgs::Point point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = 0;
        msg_r2.points.push_back(point);
        //ROS_INFO("robot 2 x:%f y: %f", point.x, point.y);
        num++;
  }
  msg_r2.size = num;
  goals_r2.publish(msg_r2);
  num = 0;
  }

  if (gr3_Points.size() != 0){
  int size = gr3_Points.size();
  Point points[size];
  Point point;
  for (int i = 0; i< size; i++){
    point.x = gr3_Points[i].x;
    point.y = gr3_Points[i].y;

    points[i] = point;
    //ROS_INFO("robot 3 x:%f y: %f", point.x, point.y);
  }
  //std::vector<Point> my_vector(points, points + sizeof(points) / sizeof(points[0]));
  int num = 0;
//ROS_INFO("vector size %lx, points size %lx", my_vector.size(), sizeof(points));
   for (int i = 0; i<gr3_Points.size(); i++) {
        geometry_msgs::Point point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = 0;
        msg_r3.points.push_back(point);
        //ROS_INFO("robot 3 x:%f y: %f", point.x, point.y);
        num++;
  }
  msg_r3.size = num;
  goals_r3.publish(msg_r3);
  num = 0;
}
}
}

void clbk_r1(const lily_explore::my_msg::ConstPtr& msg_r1){
  ROS_INFO("New information for Robot_1\n Frontiers size: %d", msg_r1->size);
  flagr1= true;
  pos_r1 = msg_r1->location;
  Point points[msg_r1->size];
  Point point;
  for (int i=0; i < msg_r1->size; i++){
    
    point.x = msg_r1->points[i].x;
    point.y = msg_r1->points[i].y;
    points[i] = point;
  } 
  
  std::vector<Point> my_vector (points, points + sizeof(points) / sizeof(Point));
  
  // loop control

  ftr_r1.clear();

  int i = 0;
  for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        ftr_r1.push_back(point);
        //ROS_INFO("robot 1 clbk x: %f, y: %f", point.x, point.y);
        i++;
  }
  add_ftr_list(ftr_r1, msg_r1->size);
  msr1_size = msg_r1->size;
}

void clbk_r2(const lily_explore::my_msg::ConstPtr& msg_r2){
  ROS_INFO("New information for Robot_2\n Frontiers size: %d", msg_r2->size);
  flagr2 = true;
  pos_r2 = msg_r2->location;
  Point points[msg_r2->size];
  Point point;
  for (int i=0; i < msg_r2->size; i++){
    
    point.x = msg_r2->points[i].x;
    point.y = msg_r2->points[i].y;
    points[i] = point;
  } 
  
  std::vector<Point> my_vector (points, points + sizeof(points) / sizeof(Point));
  
  // loop control

  ftr_r2.clear();

  int i = 0;
  for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        ftr_r2.push_back(point);
        //ROS_INFO("robot 2 clbk x: %f, y: %f", point.x, point.y);
        i++;
  }
  add_ftr_list(ftr_r2, msg_r2->size);
  msr2_size = msg_r2->size;
}

void clbk_r3(const lily_explore::my_msg::ConstPtr& msg_r3){
  ROS_INFO("New information for Robot_3\n Frontiers size: %d", msg_r3->size);
  flagr3=true;
  pos_r3 = msg_r3->location;
  Point points[msg_r3->size];
  Point point;
  for (int i=0; i < msg_r3->size; i++){
    
    point.x = msg_r3->points[i].x;
    point.y = msg_r3->points[i].y;
    points[i] = point;
  } 
  
  std::vector<Point> my_vector (points, points + sizeof(points) / sizeof(Point));
  
  // loop control

  ftr_r3.clear();

  int i = 0;
  for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        ftr_r3.push_back(point);
        //ROS_INFO("robot 3 clbk x: %f, y: %f", point.x, point.y);
        i++;
  }
  add_ftr_list(ftr_r3, msg_r3->size);
  msr3_size = msg_r3->size;
}

void clbk_map1(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
   map1 = *map_msg;
   ROS_INFO("Occupancy grid");
   flag4 = true;
}

void clbk_map2(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
   map2 = *map_msg;
   ROS_INFO("Occupancy grid");
   flag5 = true;
}

void clbk_map3(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
   map3 = *map_msg;
   ROS_INFO("Occupancy grid");
   flag6 = true;
}


 
void initialize()
{
  ROS_INFO("node works change");
  ros::NodeHandle n;
  sub_r1 = n.subscribe("/robot_1/explore/frontierlist", 1, clbk_r1);
  sub_r2 = n.subscribe("/robot_2/explore/frontierlist", 1, clbk_r2);
  sub_r3 = n.subscribe("/robot_3/explore/frontierlist", 1, clbk_r3);
  map_r1 = n.subscribe("/robot_1/map", 1, clbk_map1);
  map_r2 = n.subscribe("/robot_2/map", 1, clbk_map2);
  map_r3 = n.subscribe("/robot_3/map", 1, clbk_map3);
  goals_r1 = n.advertise<lily_explore::my_msg>("/robot_1/goals", 1);
  goals_r2 = n.advertise<lily_explore::my_msg>("/robot_2/goals", 1);
  goals_r3 = n.advertise<lily_explore::my_msg>("/robot_3/goals", 1);
  marker_array_publisher_ = n.advertise<visualization_msgs::MarkerArray>("ass_frontiers", 0);

  double timeout;
  double min_frontier_size;
  
  flagr1 = false;
  flagr2 = false;
  flagr3 = false;
  flag4 = false;
  flag5 = false;
  flag6 = false;
  uknown = true;
 
}

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "Lily_Assigner");
  
  explore::initialize();

  ros::Rate rate(100);

  while(ros::ok() ){
  ros::spinOnce();
  }

return 0;
}


