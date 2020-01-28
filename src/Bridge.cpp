/*!
* \file		Bridge.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include "Bridge.h"
#include <ros/package.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

namespace dhaiba_ros
{
/************************************************************************
*  class Bridge								*
************************************************************************/
Bridge::Bridge(const std::string& name)
    :_nh(name),
     _listener(),
     _rate(10.0),
     _root_frame("map"),
     _manager(DhaibaConnect::Manager::instance()),
     _armature_pub(  _manager->createPublisher("DhaibaRos.Armature",
					       "dhc::Armature", false, true)),
     _link_state_pub(_manager->createPublisher("DhaibaRos.LinkState",
					       "dhc::LinkState", false, false))
{
    _nh.param("rate",	    _rate,	 10.0);
    _nh.param("root_frame", _root_frame, std::string("map"));

    std::string	description_param;
    _nh.param("description_param", description_param,
	      std::string("/robot_description"));
    std::string	description_xml;
    if (!_nh.getParam(description_param, description_xml))
    {
	ROS_ERROR_STREAM("Failed to get parameter["
			 << description_param << ']');
	throw;
    }
	
    _model = urdf::parseURDF(description_xml);
    if (!_model)
    {
	ROS_ERROR_STREAM("Failed to construct urdf from parameter["
			 << description_param << ']');
	throw;
    }

    std::vector<urdf::LinkSharedPtr>	links;
    _model->getLinks(links);
    for (const auto& link : links)
    {
	std::cerr << link->name
		  << " -> " << link->getParent()->name
		  << std::endl;
    }
}

void
Bridge::run()
{
    ros::Rate	looprate(_rate);
    bool	initialized = false;
    
    while (ros::ok())
    {
	// if (!initialized)
	//     initialized = initialize();
	
	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}
  /*
bool
Bridge::initialize()
{
  // Get all TF frames.
    _frames.clear();
    _listener.getFrameStrings(_frames);

    dhc::Armature	armature;
    auto&		links = armature.links();
    links.resize(_frames.size());
    for (auto& link : links)
    {
    }
}
  */
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
		ROS_ERROR_STREAM("failed to send armature: " << err.what());
	    }
	}
}
    
void
Bridge::armature_cb(DhaibaConnect::PublisherInfo* pub,
		    DhaibaConnect::MatchingInfo*  info)
{
    using namespace	DhaibaConnect;

    std::vector<urdf::LinkSharedPtr>	model_links;
    _model->getLinks(model_links);

    dhc::Armature	armature;
    auto&		dhaiba_links = armature.links();
    dhaiba_links.resize(model_links.size());

    auto	model_link = model_links.cbegin();
    for (const auto& dhaiba_link : dhaiba_links)
    {
	dhaiba_link.linkName() = (*model_link)->name;
	
	const auto	model_parent = (*model_link)->getParent();
	if (model_parent)
	    dhaiba_link.parentLinkName() = model_parent->name;

	++model_link;
    }

    pub->write(&armature);
}
    
}	// namespace dhaiba_ros
