/*!
* \file                Bridge.h
* \author        Toshio UESHIBA
* \brief        Bridge software betwenn ROS and DhaibaWorks
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <string>
#include <map>

namespace dhaiba_ros
{
/************************************************************************
*  class Bridge                                                         *
************************************************************************/
class Bridge
{
  public:
    Bridge(const std::string& name);
    void run() const;

  private:
    void create_armature(const urdf::LinkConstSharedPtr& link,
                         const urdf::Pose& pose,
                         dhc::Armature& armature) const;
    void create_link_state(const urdf::LinkConstSharedPtr& link,
                           dhc::LinkState& link_state,
                           const tf::Transform trns) const;
    bool exist_tf(
            const std::string& parent_link_name,
            const std::string& child_link_name
            );

  private:
    ros::NodeHandle                      _nh;
    const tf::TransformListener          _listener;
    double                               _rate;

    urdf::ModelInterfaceSharedPtr        _model;
    urdf::LinkConstSharedPtr             _root_link;
    
    DhaibaConnect::Manager* const        _manager;
    DhaibaConnect::PublisherInfo*        _armature_pub;
    DhaibaConnect::PublisherInfo*        _link_state_pub;

    std::map<std::string, tf::Transform> _init_tf;
};

}        // namespace dhaiba_ros
