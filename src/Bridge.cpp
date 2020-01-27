/*!
* \file		Bridge.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <iostream>

#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
#include "Bridge.h"

namespace dhaiba_ros
{
/************************************************************************
*  global functions							*
************************************************************************/

/************************************************************************
*  class Bridge								*
************************************************************************/
Bridge::Bridge(const std::string& name)
    :_nh(name),
     _listener(),
     _rate(10.0),
     _root_frame("map")
{
    _nh.param("rate",	    _rate,	 10.0);
    _nh.param("root_frame", _root_frame, std::string("map"));
}

void
Bridge::run()
{
    ros::Rate	looprate(_rate);
    
    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}

void
Bridge::tick()
{
    std::cerr << "-------------" << std::endl;
    
  // Get all frames.
    std::vector<std::string>	frames;
    _listener.getFrameStrings(frames);
		 
    const auto	now = ros::Time::now();
    for (const auto& frame : frames)
	if (frame != _root_frame)
	{
	    try
	    {
		_listener.waitForTransform(_root_frame, frame, now,
					   ros::Duration(10));
		tf::StampedTransform	transform;
		_listener.lookupTransform(_root_frame, frame, now, transform);
		const auto&	R = transform.getBasis();
		const auto&	t = transform.getOrigin();

		std::cerr << "sent armature: " << frame << std::endl;
	    }
	    catch (const std::exception& err)
	    {
		ROS_ERROR_STREAM("failed to  send armature: " << err.what());
	    }
	}
}
    
}	// namespace dhaiba_ros
