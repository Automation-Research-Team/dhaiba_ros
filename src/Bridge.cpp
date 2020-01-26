/*!
* \file		Bridge.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <iostream>
#include <cstdint>

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
     _root_frame(""),
     _leaf_frames()
{
    _nh.param("rate", _rate, 10.0);

  // Set root_frame and leaf_frames from config file if specified.
    std::string	config;
    _nh.param("config",	config,	std::string(""));
    if (config != "")
    {
	try
	{
	    const auto	conf = YAML::LoadFile(
				ros::package::getPath("dhaiba_ros")
				+ "/config/" + config + ".yaml");
	    _root_frame  = conf["root_frame" ].template as<std::string>();
	    _leaf_frames = conf["leaf_frames"].template as<std::vector<std::string> >();
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM("(dhaiba_ros::Bridge) " << err.what());
	}
    }

    std::cerr << "Debug point 1" << std::endl;

  // Get all frames.
    std::vector<std::string>	frames;
    _listener.getFrameStrings(frames);

    std::for_each(frames.cbegin(), frames.cend(),
		  [](const auto& frame){ std::cerr << frame << std::endl; });
    std::cerr << "Debug point 2" << std::endl;

  // Validate and reset _root_frame if necessary.
    const auto	now = ros::Time::now();

    if (_root_frame == "" ||
	std::find(frames.cbegin(), frames.cend(), _root_frame)
	== frames.cend())
    {
	std::cerr << "Debug point 3" << std::endl;

	_root_frame = *std::find_if(
			frames.cbegin(), frames.cend(),
			[this, now](const auto& frame)
			{
			    std::string	parent;
			    return !_listener.getParent(frame, now, parent);
			});
    }
    
    ROS_INFO_STREAM("(dhaiba_ros::Bridge) Set root_frame to " << _root_frame);

  // Validate and reset _leaf_frames if necesssry.
    std::set<std::string>	leaf_frames(frames.begin(), frames.end());
    std::for_each(leaf_frames.cbegin(), leaf_frames.cend(),
		  [this, now, &leaf_frames](const auto& frame)
		  {
		      std::string parent;
		      if (_listener.getParent(frame, now, parent))
			  leaf_frames.erase(parent);
		  });

    for (const auto& leaf_frame : _leaf_frames)
	ROS_INFO_STREAM("(dhaiba_ros::Bridge) Set leaf_frame to "
			<< leaf_frame);
}

void
Bridge::run()
{
    ros::Rate	looprate(_rate);
    bool	initialized = false;
    
    while (ros::ok())
    {
	if (!initialized)
	{
	    initialized = true;
	}

	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}

void
Bridge::tick()
{
    std::for_each(_armatures.begin(), _armatures.end(),
		  [](auto&& val){ val.second.published = false; });
		 
    const auto	now = ros::Time::now();
    for (const auto& leaf_frame : _leaf_frames)
	send_armatures(leaf_frame, now);
}

bool
Bridge::send_armatures(const std::string& frame, ros::Time time)
{
    if (frame == _root_frame || _armatures[frame].published)
	return true;

    std::string	parent;
    if (_listener.getParent(frame, time, parent) &&
	send_armatures(parent, time))
    {
	try
	{
	    _listener.waitForTransform(_root_frame, frame, time,
				       ros::Duration(10));
	    tf::StampedTransform	transform;
	    _listener.lookupTransform(_root_frame, frame, time, transform);
	    const auto&	R = transform.getBasis();
	    const auto&	t = transform.getOrigin();
	    _armatures[frame].published = true;

	    std::cerr << "sent armature: " << frame << std::endl;
	    
	    return true;
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM("send_armature(): " << err.what());
	}
    }

    return false;
}
    
}	// namespace dhaiba_ros
