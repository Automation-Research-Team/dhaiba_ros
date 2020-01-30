/*!
* \file		Bridge.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include "Bridge.h"
#include <yaml-cpp/yaml.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

namespace dhaiba_ros
{
/************************************************************************
*  static functions							*
************************************************************************/
static urdf::Pose
operator *(const urdf::Pose& S, const urdf::Pose& T)
{
    urdf::Pose	U;
    U.position = S.rotation * T.position + S.position;
    U.rotation = S.rotation * T.rotation;

    return U;
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
     _armature_pub(  _manager->createPublisher("DhaibaRos.Armature",
					       "dhc::Armature", false, true)),
     _link_state_pub(_manager->createPublisher("DhaibaRos.LinkState",
					       "dhc::LinkState", false, false))
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
    _root_link = (root_link != links.cend() ? *root_link : _model->getRoot());
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Set root frame to \""
		    << _root_link->name << "\".");
}

void
Bridge::armature_cb(DhaibaConnect::PublisherInfo* pub,
		    DhaibaConnect::MatchingInfo*  info)
{
    dhc::Armature	armature;
    create_armature_links(_root_link, armature.links(), urdf::Pose());

    pub->write(&armature);
}

template <class LINKS> void
Bridge::create_armature_links(const urdf::LinkConstSharedPtr& link,
			      LINKS& armature_links, const urdf::Pose& pose)
{
    for (const auto& child_link : link->child_links)
    {
	const auto&	link_name   = link->name;
	const auto	child_joint = std::find_if(
					link->child_joints.cbegin(),
					link->child_joints.cend(),
					[&link_name](const auto& joint)
					{
					    return (joint->parent_link_name
						    == link_name);
					});
	if (child_joint == link->child_joints.cend())
	{
	    throw;
	}
	
	
	typename LINKS::value_type	armature_link;

	armature_link.linkName()       = child_joint->child_link_name;
	armature_link.parentLinkName() = child_joint->parent_link_name;
	
	
	armature_links.push_back(armature_link);

	
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
Bridge::publish_link_state(const urdf::LinkConstSharedPtr& link, ros::Time time)
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
