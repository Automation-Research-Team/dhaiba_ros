/*!
* \file		Detector.h
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <iostream>
#include <map>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace dhaiba_ros
{
/************************************************************************
*  class Bridge								*
************************************************************************/
class Bridge
{
  private:
    struct frame_props
    {
	std::string	mesh = "";
	bool		published = false;
    };
    
  public:
		Bridge(const std::string& name)		;

    double	rate()				const	{ return _rate; }
    void	run()					;
	
  private:
    void	tick()							;
    bool	send_armatures(const std::string& frame, ros::Time time);

  private:
    ros::NodeHandle			_nh;
    const tf::TransformListener		_listener;
    double				_rate;
    
    std::string				_root_frame;
    std::vector<std::string>		_leaf_frames;
    std::map<std::string, frame_props>	_armatures;
};

}	// namespace dhaiba_ros
