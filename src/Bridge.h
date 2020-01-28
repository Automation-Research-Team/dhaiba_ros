/*!
* \file		Detector.h
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and Dhaiba Works
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
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
    void	tick()						;
    bool	initialize()					;
    void	armature_cb(DhaibaConnect::PublisherInfo* pub,
			    DhaibaConnect::MatchingInfo*  info)	;
    
  private:
    ros::NodeHandle			_nh;
    const tf::TransformListener		_listener;

    double				_rate;
    std::string				_root_frame;
    urdf::ModelInterfaceSharedPtr	_model;
    
    DhaibaConnect::Manager*		_manager;
    DhaibaConnect::PublisherInfo*	_armature_pub;
    DhaibaConnect::PublisherInfo*	_link_state_pub;
};

}	// namespace dhaiba_ros
