/*!
* \file		Bridge.h
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>

namespace dhaiba_ros
{
/************************************************************************
*  class Bridge								*
************************************************************************/
class Bridge
{
  public:
		Bridge(const std::string& name)		;

    double	rate()				const	{ return _rate; }
    void	run()					;
	
  private:
    template <class LINKS>
    void	create_armature_links(const urdf::LinkConstSharedPtr& link,
				      const urdf::Pose& pose,
				      LINKS& armature_links)	const	;
    void	publish_link_state(const urdf::LinkConstSharedPtr& link,
				   ros::Time time)		const	;
    
  private:
    ros::NodeHandle			_nh;
    const tf::TransformListener		_listener;
    double				_rate;

    urdf::ModelInterfaceSharedPtr	_model;
    urdf::LinkConstSharedPtr		_root_link;
    
    DhaibaConnect::Manager* const	_manager;
    DhaibaConnect::PublisherInfo*	_armature_pub;
    DhaibaConnect::PublisherInfo*	_link_state_pub;
};

}	// namespace dhaiba_ros
