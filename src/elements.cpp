#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <string>
#include <map>
#include <regex>
#include <iostream>
#include <fstream>

namespace dhaiba_ros
{

const std::string log_element = "element";
const int scale = 1000;

static dhc::Mat44
pose_to_mat(const urdf::Pose& pose, const int i = 1)
{
    dhc::Mat44 mat;
    mat.value() = {
                           i,                    0,                    0, 0,
                           0,                    i,                    0, 0,
                           0,                    0,                    i, 0,
        1000*pose.position.x, 1000*pose.position.y, 1000*pose.position.z, 1,

    };
    ROS_DEBUG_STREAM_NAMED(log_element, "pose: "
        << pose.position.x << " " << pose.position.y << " " << pose.position.z);
    return mat;
}

static dhc::Mat44
tf_to_mat(const tf::Transform& trns, const int i = 1)
{
    const auto& R = trns.getBasis();
    const auto& t = trns.getOrigin();
    dhc::Mat44  mat;
    mat.value() = {i*R[0][0],   i*R[1][0],  i*R[2][0], 0,
           i*R[0][1],   i*R[1][1],  i*R[2][1], 0,
           i*R[0][2],   i*R[1][2],  i*R[2][2], 0,
           1000*t.x(), 1000*t.y(), 1000*t.z(), 1};
    return mat;
}

bool
Bridge::loadSTL(const std::string& url, std::vector<char>& data)
{
    ROS_DEBUG_STREAM_NAMED(log_element,
        "loadSTL: url=" << url << " data max_size=" << data.max_size());
    data.clear();

    const char* type[] = {
        "package://",
        "file://",
    };

    std::string::size_type pos1, pos2;
    std::string path;
    for (int i = 0; i < 2; i++)
    {
        pos1 = url.find(type[i], 0);
        if (pos1 == std::string::npos)
            continue;
        if (i == 0)
        {
            unsigned int len = strlen(type[i]);
            pos2 = url.find("/", pos1+len);
            std::string package_name = url.substr(pos1+len, pos2-(pos1+len));
            std::string file_name = url.substr(pos2+1);
            ROS_DEBUG_STREAM_NAMED(log_element,
                "loadSTL: package=" << package_name << " file=" << file_name);
            path = ros::package::getPath(package_name);
            path += "/" + file_name;
        }
        else
            path = url.substr(pos1+strlen(type[i]));
        break;
    }
    ROS_DEBUG_STREAM_NAMED(log_element, "loadSTL: path=" << path);
    if (path.size() <= 0)
    {
        ROS_ERROR_STREAM("loadSTL: path size is 0");
        return false;
    }

    try {
        std::ifstream f;
        f.open(path, std::ios_base::in | std::ios_base::binary);
        if (!f)
        {
            ROS_ERROR_STREAM("loadSTL: (" << path << ") can not open");
            return false;
        }
        f.seekg(0, std::ios_base::end);
        auto fsize = f.tellg();
        f.seekg(0);
        data.resize(fsize);
        f.read(data.data(), fsize);
        f.close();
        ROS_DEBUG_STREAM_NAMED(log_element,
                "loadSTL: data size=" << data.size());
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("loadSTL: (" << path << ")" << e.what());
        return false;
    }
    return (data.size() > 0 ? true: false);
}

bool
Bridge::create_visual_publisher_for_mesh(const urdf::LinkConstSharedPtr& link)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry =
                    (const std::shared_ptr<urdf::Mesh>&)(visual->geometry);

    dhc::GeometryBinaryFile data;

    if (! loadSTL(geometry->filename, data.fileData().data()))
    {
        ROS_ERROR_STREAM("create_visual_publisher_for_mesh: Failed to load "
                << geometry->filename);
        return false;
    }

    data.fileExtension() = "stl";
    data.description() = link->name + "'s mesh(STL)";
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = pose_to_mat(pose, scale);

    DhaibaConnect::PublisherInfo* pub = _manager->createPublisher(
                link->name + ".Mesh::Definition_BinaryFile",
                "dhc::GeometryBinaryFile", false, true);
    if (!pub)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".Mesh::Definition_BinaryFile");
        return false;
    }
    _vis_def_pubs[link->name] = pub;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(mesh)# " << pub->topicName());

    _vis_mesh_datas[pub->topicName()] = data;

    Connections::connect(&pub->matched,
                {[this](DhaibaConnect::PublisherInfo* pub,
                        DhaibaConnect::MatchingInfo* info)
                {
                    ROS_DEBUG_STREAM_NAMED(log_element,
                        "connect publisher(mesh)# " << pub->topicName() << "]");
                    pub->write(&_vis_mesh_datas[pub->topicName()]);
                }});

    DhaibaConnect::PublisherInfo* pub2 = _manager->createPublisher(
                link->name + ".PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);
    if (!pub2)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".PointSupplier::GeometryState");
        return false;
    }
    _vis_state_pubs[link->name] = pub2;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(mesh)# " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publisher_for_box(const urdf::LinkConstSharedPtr& link)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry = (const std::shared_ptr<urdf::Box>&)(visual->geometry);

    dhc::ShapeBox data;
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = pose_to_mat(pose, scale);
    data.translation().value() = { 0, 0, 0 };
    data.scaling().value() = {
                geometry->dim.x, geometry->dim.y, geometry->dim.z
                };
    data.divisionCount().value() = { 3, 3, 3 };

    DhaibaConnect::PublisherInfo* pub = _manager->createPublisher(
                link->name + ".ShapeBox::Definition",
                "dhc::ShapeBox", false, true);
    if (!pub)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".ShapeBox::Definition");
        return false;
    }
    _vis_def_pubs[link->name] = pub;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(box)# " << pub->topicName());

    _vis_box_datas[pub->topicName()] = data;

    Connections::connect(&pub->matched,
                {[this](DhaibaConnect::PublisherInfo* pub,
                        DhaibaConnect::MatchingInfo* info)
                {
                    ROS_DEBUG_STREAM_NAMED(log_element,
                        "connect publisher(box)# " << pub->topicName() << "]");
                    pub->write(&_vis_box_datas[pub->topicName()]);
                }});

    DhaibaConnect::PublisherInfo* pub2 = _manager->createPublisher(
                link->name + ".PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);
    if (!pub2)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".PointSupplier::GeometryState");
        return false;
    }
    _vis_state_pubs[link->name] = pub2;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(box)# " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publisher_for_sphere(const urdf::LinkConstSharedPtr& link)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry =
                    (const std::shared_ptr<urdf::Sphere>&)(visual->geometry);

    dhc::ShapeSphere data;
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = pose_to_mat(pose, scale);
    data.translation().value() = { 0, 0, 0 };
    // data.scaling().value()
    // data.divisionCountU() = 10;
    // data.divisionCountV() = 10;

    DhaibaConnect::PublisherInfo* pub = _manager->createPublisher(
                link->name + ".ShapeSphere::Definition",
                "dhc::ShapeSphere", false, true);
    if (!pub)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".ShapeSphere::Definition");
        return false;
    }
    _vis_def_pubs[link->name] = pub;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(sphere)# " << pub->topicName());

    _vis_sphere_datas[pub->topicName()] = data;

    Connections::connect(&pub->matched,
                {[this](DhaibaConnect::PublisherInfo* pub,
                        DhaibaConnect::MatchingInfo* info)
                {
                    ROS_DEBUG_STREAM_NAMED(log_element,
                        "connect publisher(sphere)# "
                        << pub->topicName() << "]");
                    pub->write(&_vis_sphere_datas[pub->topicName()]);
                }});

    DhaibaConnect::PublisherInfo* pub2 = _manager->createPublisher(
                link->name + ".PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);
    if (!pub2)
    {
        ROS_ERROR_STREAM("(dhaiba_ros_bridge) Failed to create publisher # "
                << link->name + ".PointSupplier::GeometryState");
        return false;
    }
    _vis_state_pubs[link->name] = pub2;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher (sphere) # " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publishers(const urdf::LinkConstSharedPtr& link)
{
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create_visual_publishers: link[" << link->name << "]");

    bool rt = true;

    if (link->visual)
    {
        const auto visual = link->visual;
        if (visual->geometry)
        {
            switch (visual->geometry->type)
            {
            case urdf::Geometry::MESH:
                rt = create_visual_publisher_for_mesh(link);
                break;
            case urdf::Geometry::BOX:
                rt = create_visual_publisher_for_box(link);
                break;
            case urdf::Geometry::SPHERE:
                rt = create_visual_publisher_for_sphere(link);
                break;
            case urdf::Geometry::CYLINDER:
                break;
            default:
                break;
            }
            if (! rt)
                return false;
        }
        if (visual->material)
        {
            const auto material = visual->material;
        }
    }
    for (const auto& child_link : link->child_links)
    {
        rt = create_visual_publishers(child_link);
        if (! rt)
            break;
    }

    return rt;
}

void
Bridge::send_visual_state(const urdf::LinkConstSharedPtr& link)
{
    for (const auto& child_link : link->child_links)
    {
        try {
            ROS_DEBUG_STREAM_NAMED(log_element, "send_visual_state: "
                << _root_link->name << " " << child_link->name);

            if (_vis_state_pubs.find(child_link->name) != _vis_state_pubs.end())
            {
                const auto pub = _vis_state_pubs[child_link->name];
                ROS_DEBUG_STREAM_NAMED(log_element,
                        "send_visual_state: " << pub->topicName());
                tf::StampedTransform tf;
                _listener.lookupTransform(
                        _root_link->name, child_link->name, ros::Time(0), tf);
                dhc::GeometryState data;
                if (child_link->visual)
                {
                    if (child_link->visual->geometry) {
                        switch (child_link->visual->geometry->type)
                        {
                        case urdf::Geometry::MESH:
                        case urdf::Geometry::BOX:
                        case urdf::Geometry::SPHERE:
                            data.transform() = tf_to_mat(tf, scale);
                            break;
                        case urdf::Geometry::CYLINDER:
                        default:
                            break;
                        }
                    }
                }
                pub->write(&data);
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("send_visual_state: " << e.what());
            break;
        }
        send_visual_state(child_link);
    }
}

}        // namespace dhaiba_ros

