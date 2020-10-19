/*!
* \file                Bridge.cpp
* \author        Toshio UESHIBA
* \brief        Bridge software betwenn ROS and DhaibaWorks
*/
#include "Bridge.h"

#include <regex>

namespace dhaiba_ros
{
/************************************************************************
*  global functions                                                        *
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
        << v.x() << ' ' << v.y() << ' ' << v.z() << ' ' << v.w() << ']';
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
        << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ']';
}

std::ostream&
operator <<(std::ostream& out, const tf::Transform& trns)
{
    const tf::Matrix3x3& R = trns.getBasis();
    const tf::Vector3& t = trns.getOrigin();
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
        << arr[0] << ' ' << arr[1] << ' ' << arr[2] << ' ' << arr[3] << ']';
}

std::ostream&
operator <<(std::ostream& out, const dhc::Mat44 mat)
{
    return out << '['
        << mat.value()[0]  << ", " << mat.value()[1]  << ", "
        << mat.value()[2]  << ", " << mat.value()[3]  << ", "
        << mat.value()[4]  << ", " << mat.value()[5]  << ", "
        << mat.value()[6]  << ", " << mat.value()[7]  << ", "
        << mat.value()[8]  << ", " << mat.value()[9]  << ", "
        << mat.value()[10] << ", " << mat.value()[11] << ", "
        << mat.value()[12] << ", " << mat.value()[13] << ", "
        << mat.value()[14] << ", " << mat.value()[15] << ']';
}

/************************************************************************
*  static functions                                                        *
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
    mat.value() = {
#if 1
        rx.x, rx.y, rx.z, 0,
        ry.x, ry.y, ry.z, 0,
        rz.x, rz.y, rz.z, 0,
#else
        rx.x, ry.x, rz.x, 0,
        rx.y, ry.y, rz.y, 0,
        rx.z, ry.z, rz.z, 0,
#endif
        1000*pose.position.x, 1000*pose.position.y, 1000*pose.position.z, 1
        };
    return mat;
}

#if 0
static dhc::Mat44
transform(const tf::Transform& trns)
{
    const auto& R = trns.getBasis();
    const auto& t = trns.getOrigin();
    dhc::Mat44  mat;
    mat.value() = {R[0][0],    R[1][0],    R[2][0],    0,
                   R[0][1],    R[1][1],    R[2][1],    0,
                   R[0][2],    R[1][2],    R[2][2],    0,
                   1000*t.x(), 1000*t.y(), 1000*t.z(), 1};
    return mat;
}
#endif

static tf::Transform
urdf2tf(const urdf::Pose& pose)
{
    tf::Transform result(
        tf::Quaternion(
            pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
        tf::Vector3(pose.position.x, pose.position.y, pose.position.z)
        );

    return result;
}

static dhc::Mat44
transform2(const tf::Transform& child_trans, const tf::Transform& parent_trans,
    const tf::Transform& child_init, const tf::Transform& parent_init)
{
    ROS_DEBUG_STREAM("transform2: "
      << "\nchild_init  " << child_init << "\nchild_trans " << child_trans
      << "\nparent_init  " << parent_init << "\nparent_trans " << parent_trans);

    const auto& Bc = child_trans.getBasis();
    const auto& Oc = child_trans.getOrigin();
    const auto& Rc = child_trans.getRotation();

    const auto& Bp = parent_trans.getBasis();
    const auto& Op = parent_trans.getOrigin();
    const auto& Rp = parent_trans.getRotation();

    const auto& Bp0 = parent_init.getBasis();
    const auto& Op0 = parent_init.getOrigin();
    const auto& Rp0 = parent_init.getRotation();

    const auto& Bc0 = child_init.getBasis();
    const auto& Oc0 = child_init.getOrigin();
    const auto& Rc0 = child_init.getRotation();

    ROS_DEBUG_STREAM("transform2: Rp0" << Rp0 << " Rp" << Rp);
    ROS_DEBUG_STREAM("transform2: Rc0" << Rc0 << " Rc" << Rc);

#if 0
    // const auto& pos = Bp * (Bp0*Bc0).inverse() * (Oc-Oc0) + (Op-Op0);
    // const auto& pos = (Bp0*Bc0).inverse() * Bp * (Oc-Oc0) + (Op-Op0);
    const auto& pos = (Bp0*Bc0).inverse() * Bp * (Oc0-Oc) + (Op0-Op);
#else
    const auto& q1 = Rp0*Rc0*Rp.inverse();
    tf::Matrix3x3 rot1(q1);
    // const auto& pos = rot1 * (Oc-Oc0) + (Op-Op0);
    const auto& pos = rot1 * (Oc0-Oc) + (Op0-Op);
#endif

    const auto& q = (Rp0*Rp.inverse()) * (Rc0*Rc.inverse());

    tf::Matrix3x3 rot(q);
    ROS_DEBUG_STREAM("transform2: q" << q);
    ROS_DEBUG_STREAM("transform2: pos" << pos);
    ROS_DEBUG_STREAM("transform2: rot" << rot);

    dhc::Mat44 mat;
    mat.value() = {
#if 1
            rot[0][0], rot[0][1], rot[0][2], 0,
            rot[1][0], rot[1][1], rot[1][2], 0,
            rot[2][0], rot[2][1], rot[2][2], 0,
#else
            rot[0][0], rot[1][0], rot[1][0], 0,
            rot[0][1], rot[1][1], rot[1][1], 0,
            rot[0][2], rot[1][2], rot[1][2], 0,
#endif
            1000*pos.x(), 1000*pos.y(), 1000*pos.z(), 1
        };
    ROS_DEBUG_STREAM("transform2: mat" << mat);

    return mat;
}

/************************************************************************
*  class Bridge                                                                *
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
    std::string description_param;
    _nh.param("description_param", description_param,
              std::string("/robot_description"));
    std::string description_xml;
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
    _root_link = (root_link != links.cend() ? *root_link : _model->getRoot());
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Set root frame to \""
                    << _root_link->name << "\".");

    {
        // for _init_tf
        dhc::Armature dummy;
        create_armature(_root_link, urdf::Pose(), dummy);
    }

    // Initialize manager.
    std::string participant_name = std::regex_replace(
                                _nh.getNamespace(), std::regex("/"), "");
    ROS_DEBUG_STREAM("name[" << name << "]["
                    << _nh.getNamespace() << "][" << participant_name << "]");
    _manager->initialize(participant_name);

    // Create publisher for armature.
    _armature_pub = _manager->createPublisher("DhaibaRos.Armature::Definition",
                                              "dhc::Armature", false, true);
    if (!_armature_pub)
    {
        ROS_ERROR_STREAM(
            "(dhaiba_ros_bridge) Failed to create publisher for armature.");
        throw;
    }

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

    // Register call back for armature pulblisher.
    Connections::connect(&_armature_pub->matched,
                         {[this](DhaibaConnect::PublisherInfo* pub,
                                 DhaibaConnect::MatchingInfo* info)
                          {
                              dhc::Armature        armature;
                              create_armature(_root_link, urdf::Pose(),
                                              armature);
                              pub->write(&armature);
                          }});

    std::cout << "\n-----\n";
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Node \"" << name << "\" initialized.");
}

void
Bridge::run() const
{
    ros::Rate looprate(_rate);

    while (ros::ok())
    {
#if 1
        dhc::LinkState link_state;
        create_link_state(_root_link, link_state, tf::Transform());
        if (! link_state.value().empty())
            _link_state_pub->write(&link_state);
#endif
        ros::spinOnce();
        looprate.sleep();
    }
}

void
Bridge::create_armature(const urdf::LinkConstSharedPtr& link,
                        const urdf::Pose& pose, dhc::Armature& armature) const
{
    ROS_DEBUG_STREAM("create_armature: link[" << link->name << "]");

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

    if (_root_link->name == link->name)
    {
        ROS_DEBUG_STREAM("create_armature: _init_tf clear [" << link->name << "]");
        _init_tf.clear();
    }
    if (_init_tf.find(link->name) == _init_tf.end())
    {
        ROS_DEBUG_STREAM("create_armature: _init_tf add [" << link->name << "]");
        _init_tf[link->name] = urdf2tf(pose);
    }

    for (const auto& child_link : link->child_links)
    {
        ROS_DEBUG_STREAM("create_armature: child_link[" << child_link->name << "]");
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
            ROS_ERROR_STREAM(
                "(dhaiba_ros_bridge) Internal inconsistency! Child link["
                << child_link->name
                << "] is not found in child joints of link["
                << link->name << "].");
            throw;
        }

#if 0
        const auto
            child_pose = (*child_joint)->parent_to_joint_origin_transform
                       * pose;
#else
        const auto child_pose = pose
                        * (*child_joint)->parent_to_joint_origin_transform;
#endif

        _init_tf[(*child_joint)->child_link_name] = urdf2tf(child_pose);

        dhc::Link armature_link;
        armature_link.linkName()       = (*child_joint)->child_link_name;
        armature_link.parentLinkName() = (*child_joint)->parent_link_name;
        armature_link.Twj0()           = transform(child_pose);
        armature_link.tailPosition0()  = position(child_pose);
        armature.links().push_back(armature_link);

        ROS_DEBUG_STREAM("create_armature: "
                << "parent[" << (*child_joint)->parent_link_name
                << "] child[" << (*child_joint)->child_link_name
                << "]\n(" << (*child_joint)->parent_to_joint_origin_transform
                << ") (" << child_pose
                << ")\n(" << armature_link.Twj0()
                << ") (" << armature_link.tailPosition0() << ")");

        create_armature(child_link, child_pose, armature);
    }
}

bool
Bridge::exist_tf(
        const std::string& parent_link_name, const std::string& child_link_name)
{
    if (_init_tf.find(parent_link_name) == _init_tf.end())
    {
        ROS_DEBUG_STREAM("exist_tf: not exists [" << parent_link_name << "]");
        return false;
    }
    if (_init_tf.find(child_link_name) == _init_tf.end())
    {
        ROS_DEBUG_STREAM("exist_tf: not exists [" << child_link_name << "]");
        return false;
    }
    return true;
}

void
Bridge::create_link_state(const urdf::LinkConstSharedPtr& link,
                          dhc::LinkState& link_state,
                          const tf::Transform trns) const
{
    tf::StampedTransform stampedTransform, trans;

    for (const auto& child_link : link->child_links)
    {
        try
        {
            _listener.lookupTransform(_root_link->name, child_link->name,
                                      ros::Time(0), stampedTransform);
            ROS_DEBUG_STREAM("create_link_state: "
                << _root_link->name << " "
                << child_link->name << " "
                << stampedTransform);
#if 0
            _listener.lookupTransform(
                        _root_link->name, child_link->getParent()->name,
                        ros::Time(0), trans);
#endif

#if 0
            link_state.value().push_back(transform(stampedTransform));
#else

            if (! exist_tf(child_link->getParent()->name, child_link->name))
            {
                ROS_ERROR_STREAM(
                    "create_link_state: not exists in init_tf ["
                        << child_link->getParent()->name
                        << "][" << child_link->name << "]");
                link_state.value().clear();
                break;
            }
            link_state.value().push_back(transform2(
                                stampedTransform, trns,
                                _init_tf[child_link->name],
                                _init_tf[child_link->getParent()->name]
                                ));
#endif
        }
        catch (const std::exception& err)
        {
            ROS_ERROR_STREAM(
                "(dhaiba_ros_bridge): Failed to publish link state["
                << child_link->name << "]. " << err.what());
            link_state.value().clear();
            break;
        }
        create_link_state(child_link, link_state, stampedTransform);
    }
}

}        // namespace dhaiba_ros
