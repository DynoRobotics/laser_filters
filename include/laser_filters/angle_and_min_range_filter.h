/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, JSK (The University of Tokyo).
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#ifndef LASER_SCAN_ANGLE_AND_MIN_RANGE_FILTER_H
#define LASER_SCAN_ANGLE_AND_MIN_RANGE_FILTER_H
/**
\author Christoffer Johannesson (Dyno Robotics)
@b ScanRangeFilter takes input scans and filters minimum range between specific angle bounds. 
    Basically and:ing a range and angle filter
**/

#include "filters/filter_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{

class LaserScanAngleAndMinRangeFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:

  double range_threshold_ ;
  double lower_angle_threshold_;
  double upper_angle_threshold_;

  bool use_message_range_limits_ ;
  float replacement_value_ ;

  bool configure()
  {
    range_threshold_ = 0.0;
    lower_angle_threshold_ = 0.0;
    upper_angle_threshold_ = 2 * 3.14159265; // TODO(Chris): Swap for math.PI
    getParam("range_threshold", range_threshold_);
    getParam("lower_angle_threshold", lower_angle_threshold_);
    getParam("upper_angle_threshold", upper_angle_threshold_);

    use_message_range_limits_ = false;
    getParam("use_message_range_limits", use_message_range_limits_);

    // work around the not implemented getParam(std::string name, float& value) method
    double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    getParam("lower_replacement_value", temp_replacement_value);
    replacement_value_ = static_cast<float>(temp_replacement_value);

    return true;
  }

  virtual ~LaserScanAngleAndMinRangeFilter()
  {

  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    unsigned int count = 0;
    filtered_scan = input_scan;
    float current_angle = input_scan.angle_min;

    if (use_message_range_limits_)
    {
      range_threshold_ = 0.99 * input_scan.range_min;
    }
    
    for (unsigned int i=0; i < input_scan.ranges.size(); i++) // Need to check ever reading in the current scan
    {
      if (current_angle < lower_angle_threshold_){
        // empty
      }else if(current_angle < upper_angle_threshold_ ){ // this is the angle range to filter
        if(filtered_scan.ranges[i] < range_threshold_){
          filtered_scan.ranges[i] = replacement_value_;
          count++;
        }
      }else if (current_angle > upper_angle_threshold_){
        break; // done with filter range, no need to keep looping
      }
      current_angle += input_scan.angle_increment;
    }

    //make sure to set all the needed fields on the filtered scan
    filtered_scan.header.frame_id = input_scan.header.frame_id;
    filtered_scan.header.stamp = input_scan.header.stamp;
    filtered_scan.angle_min = input_scan.angle_min;
    filtered_scan.angle_max = input_scan.angle_max;
    filtered_scan.angle_increment = input_scan.angle_increment;
    filtered_scan.time_increment = input_scan.time_increment;
    filtered_scan.scan_time = input_scan.scan_time;
    filtered_scan.range_min = input_scan.range_min;
    filtered_scan.range_max = input_scan.range_max;

    RCLCPP_DEBUG(logging_interface_->get_logger(), "Filtered out %d points from the laser scan.", (int)count);
    RCLCPP_INFO(logging_interface_->get_logger(), "Filtered out %d points from the laser scan.", (int)count);

    return true;
  }
} ;

}

#endif // LASER_SCAN_ANGLE_AND_MIN_RANGE_FILTER_H
