/*!
* \file		Bridge.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>
#include "Bridge.h"

namespace dhaiba_ros
{
/************************************************************************
*  static functions							*
************************************************************************/
static urdf::Pose
operator *(const urdf::Pose& a, const urdf::Pose& b)
{
    urdf::Pose	result;
    result.position = a.rotation * b.position + a.position;
    result.rotation = a.rotation * b.rotation;

    return result;
}

static std::array<double, 4>
position(const urdf::Pose& pose)
{
    return {pose.position.x, pose.position.y, pose.position.z, 1};
}
    
static std::array<double, 16>
transform(const urdf::Pose& pose)
{
    const auto	rx = pose.rotation * urdf::Vector3(1, 0, 0);
    const auto	ry = pose.rotation * urdf::Vector3(0, 1, 0);
    const auto	rz = pose.rotation * urdf::Vector3(0, 0, 1);

    return {rx.x, rx.y, rx.z, 0,
	    ry.x, ry.y, ry.z, 0,
	    rz.x, rz.y, rz.z, 0,
	    pose.position.x, pose.position.y, pose.position.z, 1};
}
    
/************************************************************************
*  class Bridge								*
************************************************************************/
Bridge::Bridge(const std::string& name)
    :_nh(name),
     _listener(),
     _rate(10.0),
     _model(),
     _root_link(),
     _manager(DhaibaConnect::Manager::instance()),
     _armature_pub(nullptr),
     _link_state_pub(nullptr)
{
    _nh.param("rate", _rate, 10.0);

  // Load robot model described in URDF.
    std::string	description_param;
    _nh.param("description_param", description_param,
	      std::string("/robot_description"));
    std::string	description_xml;
    if (!_nh.getParam(description_param, description_xml))
    {
	ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to get parameter["
			 << description_param << ']');
	throw;
    }
	
    _model = urdf::parseURDF(description_xml);
    if (!_model)
    {
	ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to load urdf in parameter["
			 << description_param << ']');
	throw;
    }

  // Get all links in the model.
    std::vector<urdf::LinkSharedPtr>	links;
    _model->getLinks(links);

  // Set root link from root frame name.
    std::string	root_frame;
    _nh.param("root_frame", root_frame, std::string("world"));
    const auto	root_link = std::find_if(links.cbegin(), links.cend(),
					 [&root_frame](const auto& link)
					 { return link->name == root_frame; });
    if (root_link == links.cend())
    {
	ROS_WARN_STREAM("(dhaiba_ros_bridge) Frame \""
			<< root_frame << "\" not found.");
	_root_link = _model->getRoot();
    }
    else
	_root_link = *root_link;
    _root_link = (root_link != links.cend() ? *root_link : _model->getRoot());
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Set root frame to \""
		    << _root_link->name << "\".");

  // Initialize manager.
    std::cerr << "-------" << std::endl;
    _manager->initialize(name);

  // Create publisher for armature.
    _armature_pub = _manager->createPublisher("DhaibaRos.Armature",
					      "dhc::Armature", false, true);
    if (!_armature_pub)
    {
	ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher for armature.");
	throw;
    }
    
  // Create publisher for link state.
    _link_state_pub = _manager->createPublisher("DhaibaRos.LinkState",
						"dhc::LinkState",
						false, false);
    if (!_link_state_pub)
    {
	ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher for link state.");
	throw;
    }
    
  // Register call back for armature pulblisher.
    Connections::connect(&_armature_pub->matched,
			 {[this](DhaibaConnect::PublisherInfo* pub,
				 DhaibaConnect::MatchingInfo* info)
			  {
			      dhc::Armature	armature;
			      create_armature_links(_root_link, urdf::Pose(),
						    armature.links());
			      pub->write(&armature);
			  }});
    std::cerr << "\n-------" << std::endl;
    
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Node \""
		    << name << "\" initialized.");
}

template <class LINKS> void
Bridge::create_armature_links(const urdf::LinkConstSharedPtr& link,
			      const urdf::Pose& pose,
			      LINKS& armature_links) const
{
    for (const auto& child_link : link->child_links)
    {
	const auto
	    child_joint = std::find_if(link->child_joints.cbegin(),
				       link->child_joints.cend(),
				       [&child_link](const auto& joint)
				       {
					   return (joint->child_link_name
						   == child_link->name);
				       });
	if (child_joint == link->child_joints.cend())
	{
	    ROS_ERROR_STREAM("(dhaiba_ros_bridge) Internal inconsistency! Child link["
			     << child_link->name
			     << "] is not found in child joints of link["
			     << link->name << "].");
	    throw;
	}
	

	const auto
	    child_pose = (*child_joint)->parent_to_joint_origin_transform
		       * pose;
	typename LINKS::value_type	armature_link;
	armature_link.linkName()	= (*child_joint)->child_link_name;
	armature_link.parentLinkName()	= (*child_joint)->parent_link_name;
	armature_link.Twj0().value()	= transform(child_pose);
	armature_link.tailPosition0().value() = position(child_pose);
	armature_links.push_back(armature_link);

	create_armature_links(child_link, child_pose, armature_links);
    }
}
    
void
Bridge::run()
{
    ros::Rate	looprate(_rate);
    
    while (ros::ok())
    {
	std::cerr << "-------------" << std::endl;
	publish_link_state(_root_link, ros::Time::now());
	ros::spinOnce();
	looprate.sleep();
    }
}

void
Bridge::publish_link_state(const urdf::LinkConstSharedPtr& link,
			   ros::Time time) const
{
    for (const auto& child_link : link->child_links)
    {
	try
	{
	    _listener.waitForTransform(_root_link->name, child_link->name,
				       time, ros::Duration(10));
	    tf::StampedTransform	transform;
	    _listener.lookupTransform(_root_link->name, child_link->name,
				      time, transform);
	    const auto&	R = transform.getBasis();
	    const auto&	t = transform.getOrigin();

	    std::cerr << "publish link state: " << child_link->name
		      << std::endl;
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM("(dhaiba_ros_bridge): Failed to publish link state["
			     << child_link->name << "]: " << err.what());
	}

	publish_link_state(child_link, time);
    }
}

}	// namespace dhaiba_ros
