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

static urdf::Pose
operator *(const urdf::Pose& a, const urdf::Pose& b)
{
    urdf::Pose result;
    result.position = a.rotation * b.position + a.position;
    result.rotation = a.rotation * b.rotation;
    return result;
}

static dhc::Mat44
tf_to_mat(const tf::Transform& tf, const tf::Vector3& scale)
{
    const auto& R = tf.getBasis().scaled(scale);
    const auto& t = tf.getOrigin();
    dhc::Mat44  mat;
    mat.value() = {
           R[0][0], R[1][0], R[2][0], 0,
           R[0][1], R[1][1], R[2][1], 0,
           R[0][2], R[1][2], R[2][2], 0,
           1000*t.x(), 1000*t.y(), 1000*t.z(), 1};
    return mat;
}

static dhc::Mat44
tf_to_mat2(
            const tf::Transform& tf1,
            const tf::Transform& tf2,
            const urdf::Vector3& dim,
            const tf::Vector3& scale
            )
{
    tf::Transform tf = tf1 * tf2;
    tf::Vector3 tmp = tf * tf::Vector3(-dim.x/2, -dim.y/2, -dim.z/2);
    tf.setOrigin(tmp);
    return tf_to_mat(tf, scale);
}

bool
Bridge::loadMesh(const std::string& url, std::vector<char>& data)
{
    ROS_DEBUG_STREAM_NAMED(log_element,
        "loadMesh: url=" << url << " data max_size=" << data.max_size());
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
                "loadMesh: package=" << package_name << " file=" << file_name);
            path = ros::package::getPath(package_name);
            path += "/" + file_name;
        }
        else
            path = url.substr(pos1+strlen(type[i]));
        break;
    }
    ROS_DEBUG_STREAM_NAMED(log_element, "loadMesh: path=" << path);
    if (path.size() <= 0)
    {
        ROS_ERROR_STREAM("loadMesh: path size is 0");
        return false;
    }

    try {
        std::ifstream f;
        f.open(path, std::ios_base::in | std::ios_base::binary);
        if (!f)
        {
            ROS_ERROR_STREAM("loadMesh: (" << path << ") can not open");
            return false;
        }
        f.seekg(0, std::ios_base::end);
        auto fsize = f.tellg();
        f.seekg(0);
        data.resize(fsize);
        f.read(data.data(), fsize);
        f.close();
        ROS_DEBUG_STREAM_NAMED(log_element,
                "loadMesh: data size=" << data.size());
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("loadMesh: (" << path << ")" << e.what());
        return false;
    }
    return (data.size() > 0 ? true: false);
}

bool
Bridge::create_visual_publisher_for_mesh(
                const urdf::LinkConstSharedPtr& link, const urdf::Pose& parent)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry =
                    (const std::shared_ptr<urdf::Mesh>&)(visual->geometry);

    dhc::GeometryBinaryFile data;

    if (! loadMesh(geometry->filename, data.fileData().data()))
    {
        ROS_ERROR_STREAM("create_visual_publisher_for_mesh: Failed to load "
                << geometry->filename);
        return false;
    }

    tf::Vector3 scale(
        geometry->scale.x*1000, geometry->scale.y*1000, geometry->scale.z*1000);

    std::string file_extension = "";
    const auto pos = geometry->filename.rfind(".");
    if (pos != std::string::npos)
        file_extension = geometry->filename.substr(pos+1);

    data.fileExtension() = file_extension;
    data.description() = link->name + "'s mesh(" + file_extension + ")";
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = tf_to_mat(
                                transform(parent) * transform(pose), scale);

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
    _vis_scales[link->name] = scale;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(mesh)# " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publisher_for_box(
                const urdf::LinkConstSharedPtr& link, const urdf::Pose& parent)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry = (const std::shared_ptr<urdf::Box>&)(visual->geometry);

    tf::Vector3 scale(1000, 1000, 1000);

    dhc::ShapeBox data;
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = tf_to_mat2(
                transform(parent), transform(pose), geometry->dim, scale);
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
    _vis_scales[link->name] = scale;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher(box)# " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publisher_for_sphere(
                const urdf::LinkConstSharedPtr& link, const urdf::Pose& parent)
{
    const auto visual   = link->visual;
    const auto pose     = visual->origin;
    const auto geometry =
                    (const std::shared_ptr<urdf::Sphere>&)(visual->geometry);

    tf::Vector3 scale(1000, 1000, 1000);

    dhc::ShapeSphere data;
    data.baseInfo().color().r() = 128;
    data.baseInfo().color().g() = 128;
    data.baseInfo().color().b() = 128;
    data.baseInfo().transform() = tf_to_mat(
                                transform(parent) * transform(pose), scale);
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
    _vis_scales[link->name] = scale;
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create publisher (sphere) # " << pub2->topicName());

    return true;
}

bool
Bridge::create_visual_publishers(
        const urdf::LinkConstSharedPtr& link, const urdf::Pose& parent)
{
    ROS_DEBUG_STREAM_NAMED(log_element,
                "create_visual_publishers: link[" << link->name << "]");

    bool rt = true;
    urdf::Pose pose;

    if (link->parent_joint)
        pose = parent * link->parent_joint->parent_to_joint_origin_transform;
    else
        pose = parent;

    if (link->visual)
    {
        const auto visual = link->visual;
        if (visual->geometry)
        {
            switch (visual->geometry->type)
            {
            case urdf::Geometry::MESH:
                rt = create_visual_publisher_for_mesh(link, pose);
                break;
            case urdf::Geometry::BOX:
                rt = create_visual_publisher_for_box(link, pose);
                break;
            case urdf::Geometry::SPHERE:
                rt = create_visual_publisher_for_sphere(link, pose);
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
        rt = create_visual_publishers(child_link, pose);
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
                tf::Vector3 scale(1000, 1000, 1000);
                if (_vis_scales.find(child_link->name) != _vis_scales.end())
                    scale = _vis_scales[child_link->name];

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
                            data.transform() = tf_to_mat(
                                    tf * transform(child_link->visual->origin),
                                    scale
                                    );
                            break;
                        case urdf::Geometry::BOX:
                            data.transform() = tf_to_mat2(
                                    tf,
                                    transform(child_link->visual->origin),
                                    ((const std::shared_ptr<urdf::Box>&)(
                                            child_link->visual->geometry))->dim,
                                    scale
                                    );
                            break;
                        case urdf::Geometry::SPHERE:
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

