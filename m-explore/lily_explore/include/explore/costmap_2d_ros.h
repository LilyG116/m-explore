 /*********************************************************************
  *
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2008, 2013, Willow Garage, Inc.
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of Willow Garage, Inc. nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  * Author: Eitan Marder-Eppstein
  *         David V. Lu!!
  *********************************************************************/
 #ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
 #define COSTMAP_2D_COSTMAP_2D_ROS_H_
  
 #include <costmap_2d/layered_costmap.h>
 #include <costmap_2d/layer.h>
 #include <costmap_2d/costmap_2d_publisher.h>
 #include <costmap_2d/Costmap2DConfig.h>
 #include <costmap_2d/footprint.h>
 #include <geometry_msgs/Polygon.h>
 #include <geometry_msgs/PolygonStamped.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <dynamic_reconfigure/server.h>
 #include <pluginlib/class_loader.hpp>
 #include <tf2/LinearMath/Transform.h>
  
 class SuperValue : public XmlRpc::XmlRpcValue
 {
 public:
   void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
   {
     _type = TypeStruct;
     _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
   }
   void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
   {
     _type = TypeArray;
     _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
   }
 };
  
 namespace costmap_2d
 {
  
 class Costmap2DROS
 {
 public:
   Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
   ~Costmap2DROS();
  
   void start();
  
   void stop();
  
   void pause();
  
   void resume();
  
   void updateMap();
  
   void resetLayers();
  
   bool isCurrent() const
     {
       return layered_costmap_->isCurrent();
     }
  
   bool isStopped() const
     {
       return stopped_;
     }
  
   bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;
  
   inline const std::string& getName() const noexcept
     {
       return name_;
     }
  
   double getTransformTolerance() const
     {
       return transform_tolerance_;
     }
  
   Costmap2D* getCostmap() const
     {
       return layered_costmap_->getCostmap();
     }
  
   inline const std::string& getGlobalFrameID() const noexcept
     {
       return global_frame_;
     }
  
   inline const std::string& getBaseFrameID() const noexcept
     {
       return robot_base_frame_;
     }
   LayeredCostmap* getLayeredCostmap() const
     {
       return layered_costmap_;
     }
  
   geometry_msgs::Polygon getRobotFootprintPolygon() const
   {
     return costmap_2d::toPolygon(padded_footprint_);
   }
  
   inline const std::vector<geometry_msgs::Point>& getRobotFootprint() const noexcept
   {
     return padded_footprint_;
   }
  
   inline const std::vector<geometry_msgs::Point>& getUnpaddedRobotFootprint() const noexcept
   {
     return unpadded_footprint_;
   }
  
   void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;
  
   void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);
  
   void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);
  
 protected:
   LayeredCostmap* layered_costmap_;
   std::string name_;
   tf2_ros::Buffer& tf_;  
   std::string global_frame_;  
   std::string robot_base_frame_;  
   double transform_tolerance_;  
  
 private:
   void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                                const costmap_2d::Costmap2DConfig &old_config);
  
   void loadOldParameters(ros::NodeHandle& nh);
   void warnForOldParameters(ros::NodeHandle& nh);
   void checkOldParam(ros::NodeHandle& nh, const std::string &param_name);
   void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
   void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
   void movementCB(const ros::TimerEvent &event);
   void mapUpdateLoop(double frequency);
   bool map_update_thread_shutdown_;
   bool stop_updates_, initialized_, stopped_, robot_stopped_;
   boost::thread* map_update_thread_;  
   ros::Timer timer_;
   ros::Time last_publish_;
   ros::Duration publish_cycle;
   pluginlib::ClassLoader<Layer> plugin_loader_;
   geometry_msgs::PoseStamped old_pose_;
   Costmap2DPublisher* publisher_;
   dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;
  
   boost::recursive_mutex configuration_mutex_;
  
   ros::Subscriber footprint_sub_;
   ros::Publisher footprint_pub_;
   std::vector<geometry_msgs::Point> unpadded_footprint_;
   std::vector<geometry_msgs::Point> padded_footprint_;
   float footprint_padding_;
   costmap_2d::Costmap2DConfig old_config_;
 };
 // class Costmap2DROS
 }  // namespace costmap_2d
  
 #endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
