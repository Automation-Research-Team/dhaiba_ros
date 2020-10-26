/*!
*  \file	Bridge.cpp
*  \author	Toshio UESHIBA
*  \brief	Bridge software betwenn ROS and DhaibaWorks
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <string>
#include <map>
#include <regex>

namespace dhaiba_ros
{
/************************************************************************
*  global functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const urdf::Vector3& v)
{
    return out << v.x << ' ' << v.y << ' ' << v.z;
}

std::ostream&
operator <<(std::ostream& out, const urdf::Rotation& q)
{
    return out << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w;
}

std::ostream&
operator <<(std::ostream& out, const urdf::Pose& pose)
{
    return out << '[' << pose.position << ';' << pose.rotation << ']';
}

std::ostream&
operator <<(std::ostream& out, const tf::Vector3& v)
{
    return out << '['
	       << v.x() << ' ' << v.y() << ' ' << v.z() << ' ' << v.w()
	       << ']';
}

std::ostream&
operator <<(std::ostream& out, const tf::Matrix3x3& m)
{
    for (int r = 0; r < 3; r++)
    {
	out << '[';
	for (int c = 0; c < 3; c++)
	{
	    if (c > 0)
		out << ' ';
	    out << m[r][c];
	}
	out << ']';
    }
    return out;
}

std::ostream&
operator <<(std::ostream& out, const tf::Quaternion& q)
{
    return out << '['
	       << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
	       << ']';
}

std::ostream&
operator <<(std::ostream& out, const tf::Transform& trns)
{
    const tf::Matrix3x3& R = trns.getBasis();
    const tf::Vector3&	 t = trns.getOrigin();
    const tf::Quaternion q = trns.getRotation();
    return out << "basis["        << trns.getBasis()
	       << "] origin["     << trns.getOrigin()
	       << "] quaternion[" << trns.getRotation() << "]";
}

std::ostream&
operator <<(std::ostream& out, const dhc::Vec4& v)
{
    const auto arr = v.value();
    return out << '['
	       << arr[0] << ' ' << arr[1] << ' ' << arr[2] << ' ' << arr[3]
	       << ']';
}

std::ostream&
operator <<(std::ostream& out, const dhc::Mat44& mat)
{
    return out << '['
	       << mat.value()[0]  << ", " << mat.value()[1]  << ", "
	       << mat.value()[2]  << ", " << mat.value()[3]  << ", " << '\n'
	       << mat.value()[4]  << ", " << mat.value()[5]  << ", "
	       << mat.value()[6]  << ", " << mat.value()[7]  << ", " << '\n'
	       << mat.value()[8]  << ", " << mat.value()[9]  << ", "
	       << mat.value()[10] << ", " << mat.value()[11] << ", " << '\n'
	       << mat.value()[12] << ", " << mat.value()[13] << ", "
	       << mat.value()[14] << ", " << mat.value()[15] << ']';
}

/************************************************************************
*  static functions							*
************************************************************************/
static urdf::Pose
operator *(const urdf::Pose& a, const urdf::Pose& b)
{
    urdf::Pose result;
    result.position = a.rotation * b.position + a.position;
    result.rotation = a.rotation * b.rotation;
    return result;
}

static dhc::Vec4
position(const urdf::Pose& pose)
{
    dhc::Vec4 vec;
    vec.value() = {1000*pose.position.x, 1000*pose.position.y,
                   1000*pose.position.z, 1};
    return vec;
}

static dhc::Mat44
transform(const urdf::Pose& pose)
{
    const auto rx = pose.rotation * urdf::Vector3(1, 0, 0);
    const auto ry = pose.rotation * urdf::Vector3(0, 1, 0);
    const auto rz = pose.rotation * urdf::Vector3(0, 0, 1);
    dhc::Mat44 mat;
#if 1
    mat.value() = {rx.x, rx.y, rx.z, 0,
		   ry.x, ry.y, ry.z, 0,
		   rz.x, rz.y, rz.z, 0,
		   1000*pose.position.x, 1000*pose.position.y,
		   1000*pose.position.z, 1};
#else
    mat.value() = {rx.x, ry.x, rz.x, 0,
		   rx.y, ry.y, rz.y, 0,
		   rx.z, ry.z, rz.z, 0,
		   1000*pose.position.x, 1000*pose.position.y,
		   1000*pose.position.z, 1};
#endif
    return mat;
}

static dhc::Mat44
transform(const tf::Transform& trns)
{
    const auto&	R = trns.getBasis();
    const auto&	t = trns.getOrigin();
    dhc::Mat44	mat;
    mat.value() = {R[0][0],    R[1][0],    R[2][0],    0,
		   R[0][1],    R[1][1],    R[2][1],    0,
		   R[0][2],    R[1][2],    R[2][2],    0,
		   1000*t.x(), 1000*t.y(), 1000*t.z(), 1};
    return mat;
}
    
/************************************************************************
*  class Bridge								*
************************************************************************/
class Bridge
{
  public:
		Bridge(const std::string& name)				;

    void	run()						const	;

  private:
    void	create_armature(const urdf::LinkConstSharedPtr& link,
				const urdf::Pose& pose,
				dhc::Armature& armature)	const	;
    void	create_link_state(const urdf::LinkConstSharedPtr& link,
				  dhc::LinkState& link_state)	const	;
    bool	exist_tf(const std::string& parent_link_name,
			 const std::string& child_link_name)		;

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
    std::vector<urdf::LinkSharedPtr> links;
    _model->getLinks(links);

  // Set root link from root frame name.
    std::string root_frame;
    _nh.param("root_frame", root_frame, std::string("world"));
    const auto root_link = std::find_if(links.cbegin(), links.cend(),
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
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Set root frame to \""
                    << _root_link->name << "\".");

  // Initialize manager.
    std::string participant_name = std::regex_replace(_nh.getNamespace(),
						      std::regex("/"), "");
    ROS_DEBUG_STREAM("name[" << name << "]["
		     << _nh.getNamespace() << "][" << participant_name << "]");
    _manager->initialize(participant_name);

  // Create publisher for armature.
    _armature_pub = _manager->createPublisher("DhaibaRos.Armature::Definition",
                                              "dhc::Armature", false, true);
    if (!_armature_pub)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher for armature.");
        throw;
    }

  // Register callback for armature pulblisher.
    Connections::connect(&_armature_pub->matched,
                         {[this](DhaibaConnect::PublisherInfo* pub,
                                 DhaibaConnect::MatchingInfo* info)
                          {
                              dhc::Armature        armature;
                              create_armature(_root_link, urdf::Pose(),
                                              armature);
                              pub->write(&armature);
                          }});

  // Create publisher for link state.
    _link_state_pub = _manager->createPublisher("DhaibaRos.Armature::LinkState",
                                                "dhc::LinkState",
                                                false, false);
    if (!_link_state_pub)
    {
        ROS_ERROR_STREAM(
            "(dhaiba_ros_bridge) Failed to create publisher for link state.");
        throw;
    }

    std::cout << "\n-----\n";
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Node \"" << name << "\" initialized.");
}

void
Bridge::run() const
{
    ros::Rate looprate(_rate);

    while (ros::ok())
    {
        dhc::LinkState link_state;
        create_link_state(_root_link, link_state);
        if (! link_state.value().empty())
            _link_state_pub->write(&link_state);

        ros::spinOnce();
        looprate.sleep();
    }
}

void
Bridge::create_armature(const urdf::LinkConstSharedPtr& link,
                        const urdf::Pose& pose, dhc::Armature& armature) const
{
    if (link->visual)
    {
        const auto visual = link->visual;

        if (visual->geometry)
        {
            const auto pose     = visual->origin;
            const auto geometry = visual->geometry;

            switch (geometry->type)
            {
              case urdf::Geometry::SPHERE:
                break;
              case urdf::Geometry::BOX:
                break;
              case urdf::Geometry::CYLINDER:
                break;
              case urdf::Geometry::MESH:
                break;
              default:
                break;
            }
        }

        if (visual->material)
        {
            const auto material = visual->material;
        }
    }

    for (const auto& child_link : link->child_links)
    {
      // Find a joint whose child_link_name is equal to the name of child_link.
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

	const auto	child_local_pose
			    = (*child_joint)->parent_to_joint_origin_transform;
	const auto	child_pose = pose * child_local_pose;
        dhc::Link	armature_link;
        armature_link.linkName()       = (*child_joint)->child_link_name;
        armature_link.parentLinkName() = (*child_joint)->parent_link_name;
        armature_link.Twj0()           = transform(child_pose);
        armature_link.tailPosition0()  = position(pose);
        armature.links().push_back(armature_link);

        ROS_DEBUG_STREAM("create_armature: "
			 << (*child_joint)->parent_link_name << " <== "
			 << (*child_joint)->child_link_name << "\n("
			 << (*child_joint)->parent_to_joint_origin_transform
			 << ") (" << child_pose
			 << ")\n(" << armature_link.Twj0()
			 << ") (" << armature_link.tailPosition0() << ")");

        create_armature(child_link, child_pose, armature);
    }
}

void
Bridge::create_link_state(const urdf::LinkConstSharedPtr& link,
                          dhc::LinkState& link_state) const
{
    for (const auto& child_link : link->child_links)
    {
        try
        {
	    tf::StampedTransform stampedTransform;
            _listener.lookupTransform(link->name, child_link->name,
                                      ros::Time(0), stampedTransform);
            link_state.value().push_back(transform(stampedTransform));

	    // ROS_DEBUG_STREAM("create_link_state: "
	    // 		     << link->name << " <-- "
	    // 		     << child_link->name << '\n'
	    // 		     << transform(stampedTransform));
        }
        catch (const std::exception& err)
        {
            ROS_ERROR_STREAM(
                "(dhaiba_ros_bridge): Failed to publish link state["
                << child_link->name << "]. " << err.what());
            link_state.value().clear();
            break;
        }

	create_link_state(child_link, link_state);
    }
}

}        // namespace dhaiba_ros

/************************************************************************
*  main function                                                        *
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dhaiba_ros_bridge");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);

    try
    {
        const dhaiba_ros::Bridge        bridge("~");
        bridge.run();
    }
    catch (const std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        return 1;
    }

    return 0;
}
