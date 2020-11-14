/*!
*  \file	Bridge.cpp
*  \author	Toshio UESHIBA
*  \brief	Bridge software betwenn ROS and DhaibaWorks
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <string>
#include <map>
#include <regex>
#include <iomanip>
#include <fstream>

namespace dhaiba_ros
{
const std::string log_element = "element";

/************************************************************************
*  global functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const tf::Vector3& v)
{
    return out << '[' << v.x() << ' ' << v.y() << ' ' << v.z() << ']';
}

std::ostream&
operator <<(std::ostream& out, const tf::Matrix3x3& m)
{
    return out << '['
	       << m[0][0]  << ' ' << m[0][1]  << ' ' << m[0][2] << "\n "
	       << m[1][0]  << ' ' << m[1][1]  << ' ' << m[1][2] << "\n "
	       << m[2][0]  << ' ' << m[2][1]  << ' ' << m[2][2] << ']';
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
    return out << "basis: "        << trns.getBasis()
	       << "\norigin: "     << trns.getOrigin()
	       << "\nquaternion: " << trns.getRotation();
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
	       << mat.value()[0]  << ' ' << mat.value()[1]  << ' '
	       << mat.value()[2]  << ' ' << mat.value()[3]  << "\n "
	       << mat.value()[4]  << ' ' << mat.value()[5]  << ' '
	       << mat.value()[6]  << ' ' << mat.value()[7]  << "\n "
	       << mat.value()[8]  << ' ' << mat.value()[9]  << ' '
	       << mat.value()[10] << ' ' << mat.value()[11] << "\n "
	       << mat.value()[12] << ' ' << mat.value()[13] << ' '
	       << mat.value()[14] << ' ' << mat.value()[15] << ']';
}

/************************************************************************
*  static functions							*
************************************************************************/
static tf::Transform
transform(const urdf::Pose& pose)
{
    return tf::Transform({pose.rotation.x, pose.rotation.y,
    			  pose.rotation.z, pose.rotation.w},
    			 {pose.position.x, pose.position.y, pose.position.z});
}

static dhc::Vec4
vec4(const tf::Vector3& origin)
{
    dhc::Vec4	vec;
    vec.value() = {1000*origin.x(), 1000*origin.y(), 1000*origin.z(), 1};
    return vec;
}

static dhc::Mat44
mat44(const tf::Transform& trns, bool scale=false)
{
    const auto	s = (scale ? 1000.0 : 1.0);
    const auto& R = trns.getBasis();
    const auto& t = trns.getOrigin();
    dhc::Mat44  mat;
    mat.value() = {s*R[0][0],  s*R[1][0],  s*R[2][0],  0,
		   s*R[0][1],  s*R[1][1],  s*R[2][1],  0,
		   s*R[0][2],  s*R[1][2],  s*R[2][2],  0,
		   1000*t.x(), 1000*t.y(), 1000*t.z(), 1};
    return mat;
}

static std::vector<char>
loadMesh(const std::string& url)
{
    ROS_DEBUG_STREAM_NAMED(log_element, "loadMesh: url=" << url);

  // Extract mesh file path from URL.
    constexpr char*	types[] = {"package://", "file://"};
    std::vector<char>	data;
    std::string		path;
    for (const auto type : types)
    {
        const auto	pos1 = url.find(type, 0);

	if (pos1 == std::string::npos)
            continue;
        if (type == types[0])
        {
	    const auto	len  = strlen(type);
            const auto	pos2 = url.find("/", pos1 + len);
            const auto	package_name = url.substr(pos1 + len,
						  pos2 - (pos1+len));
            const auto	file_name = url.substr(pos2 + 1);
            ROS_DEBUG_STREAM_NAMED(log_element,
				   "loadMesh: package=" << package_name
				   << " file=" << file_name);
            path = ros::package::getPath(package_name) + '/' + file_name;
        }
        else
            path = url.substr(pos1 + strlen(type));
        break;
    }
    ROS_DEBUG_STREAM_NAMED(log_element, "loadMesh: path=" << path);

    std::ifstream	fin(path, std::ios_base::in | std::ios_base::binary);
    if (!fin)
	throw std::runtime_error("loadMesh: cannot open mesh file["
				 + path + ']');
    fin.seekg(0, std::ios_base::end);
    const auto	fsize = fin.tellg();
    fin.seekg(0);
    data.resize(fsize);
    fin.read(data.data(), fsize);

    ROS_DEBUG_STREAM_NAMED(log_element, "loadMesh: data size=" << data.size());
    
    return data;
}

/************************************************************************
*  class Bridge								*
************************************************************************/
class Bridge
{
  public:
    using base_info_t	= dhc::GeometryBaseInfo;
    using box_t		= dhc::ShapeBox;
    using sphere_t	= dhc::ShapeSphere;
    using mesh_t	= dhc::GeometryBinaryFile;
    
  private:
    using armature_t	= dhc::Armature;
    using boxes_t	= std::vector<box_t>;
    using spheres_t	= std::vector<sphere_t>;
    using meshes_t	= std::vector<mesh_t>;
    using link_cp	= urdf::LinkConstSharedPtr;
    using visual_cp	= urdf::VisualConstSharedPtr;
    using material_cp	= urdf::MaterialConstSharedPtr;
    
  public:
		Bridge(const std::string& name)				;

    void	run()						const	;

  private:
    DhaibaConnect::PublisherInfo*
		create_publisher(const std::string& topicName,
				 const std::string& typeName,
				 bool highReliability)		const	;
    void	create_primitives(const link_cp& link,
				  const tf::Transform& Twp0)		;
    void	create_link_state(const link_cp& link,
				  dhc::LinkState& link_state)	const	;
    void	publish_geometry_state(const link_cp& link)	const	;

    box_t	create_box(const link_cp& link,
			   const tf::Transform& Twp0)			;
    sphere_t	create_sphere(const link_cp& link,
			      const tf::Transform& Twp0)		;
    mesh_t	create_mesh(const link_cp& link,
			    const tf::Transform& Twp0)			;
    base_info_t	create_base_info(const material_cp& visual,
				 const tf::Transform& Twp0)		;

  private:
    ros::NodeHandle				_nh;
    const tf::TransformListener			_listener;
    double					_rate;

    urdf::ModelInterfaceSharedPtr		_model;
    urdf::LinkConstSharedPtr			_root_link;
    std::map<std::string, tf::Transform>	_Tj0p0;
    std::map<std::string, tf::Transform>	_Tjo;

    armature_t					_armature;
    boxes_t					_boxes;
    spheres_t					_spheres;
    meshes_t					_meshes;
    
    DhaibaConnect::Manager* const		_manager;
    DhaibaConnect::PublisherInfo*		_armature_pub;
    DhaibaConnect::PublisherInfo*		_link_state_pub;

    DhaibaConnect::PublisherInfo*		_shape_box_pub;
    DhaibaConnect::PublisherInfo*		_shape_sphere_pub;
    DhaibaConnect::PublisherInfo*		_mesh_pub;
    DhaibaConnect::PublisherInfo*		_geometry_state_pub;

    // std::map<std::string, DhaibaConnect::PublisherInfo*> _vis_def_pubs;
    // std::map<std::string, DhaibaConnect::PublisherInfo*> _vis_state_pubs;
    std::map<std::string, tf::Vector3>             _vis_scales;
    std::map<std::string, dhc::GeometryBinaryFile> _vis_mesh_datas;
    std::map<std::string, dhc::ShapeBox>           _vis_box_datas;
    std::map<std::string, dhc::ShapeSphere>        _vis_sphere_datas;

};

Bridge::Bridge(const std::string& name)
    :_nh(name),
     _listener(),
     _rate(10.0),
     _model(),
     _root_link(),
     _Tj0p0(),
     _manager(DhaibaConnect::Manager::instance()),
     _armature_pub(nullptr),
     _link_state_pub(nullptr),
     _shape_box_pub(nullptr),
     _shape_sphere_pub(nullptr),
     _mesh_pub(nullptr),
     _geometry_state_pub(nullptr)
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

  // Create publishers
    _armature_pub	= create_publisher("DhaibaRos.Armature::Definition",
					   "dhc::Armature", true);
    _link_state_pub	= create_publisher("DhaibaRos.Armature::LinkState",
					   "dhc::LinkState", false);
    _shape_box_pub	= create_publisher("DhaibaRos.ShapeBox::Definition",
					   "dhc::ShapeBox", true);
    _shape_sphere_pub	= create_publisher("DhaibaRos.ShapeSphere::Definition",
					   "dhc::ShapeSphere", true);
    _mesh_pub		= create_publisher(
				    "DhaibaRos.Mesh::Definition_BinaryFile",
				    "dhc::GeometryBinaryFile", true);
    _geometry_state_pub	= create_publisher(
				    "DhaibaRos.PointSupplier::GeometryState",
				    "dhc::GeometryState", false);

    create_primitives(_root_link,
		      tf::Transform({0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}));
    std::cerr << _armature.links().size() << " armatrue links, "
	      << _boxes.size() << " boxes, "
	      << _spheres.size() << " spheres, "
	      << _meshes.size() << " meshes"
	      << std::endl;
		 
  // Register callback for pulblishers.
    Connections::connect(&_armature_pub->matched,
                         {[this](DhaibaConnect::PublisherInfo* pub,
				 DhaibaConnect::MatchingInfo* info)
                          {
                              pub->write(&_armature);
                          }});
    Connections::connect(&_shape_box_pub->matched,
                         {[this](DhaibaConnect::PublisherInfo* pub,
				 DhaibaConnect::MatchingInfo* info)
                          {
			      for (const auto& box : _boxes)
				  pub->write(&box);
                          }});
	       /*
    Connections::connect(&_shape_sphere_pub->matched,
                         {[&spheres](DhaibaConnect::PublisherInfo* pub,
				     DhaibaConnect::MatchingInfo* info)
                          {
			      for (const auto& sphere : spheres)
				  pub->write(&sphere);
                          }});
	       */
    Connections::connect(&_mesh_pub->matched,
                         {[this](DhaibaConnect::PublisherInfo* pub,
				 DhaibaConnect::MatchingInfo* info)
                          {
			      for (const auto& mesh : _meshes)
				  pub->write(&mesh);
                          }});

  // Create publisher for link state.
    ROS_INFO_STREAM("(dhaiba_ros_bridge) Node[" << name << "] initialized.");
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

      //publish_geometry_state(_root_link);

        ros::spinOnce();
        looprate.sleep();
    }
}

DhaibaConnect::PublisherInfo*
Bridge::create_publisher(const std::string& topicName,
			 const std::string& typeName,
			 bool highReliability) const
{
    const auto	pub = _manager->createPublisher(topicName, typeName, 
						false, highReliability);
    if (!pub)
    {
        ROS_ERROR_STREAM(
            "(dhaiba_ros_bridge) Failed to create publisher for topic["
	    << topicName << " of type[" << typeName << ']');
        throw;
    }

    return pub;
}

void
Bridge::create_primitives(const link_cp& link, const tf::Transform& Twp0)
{
    if (link->visual && link->visual->geometry)
    {
	try
	{
	    switch (link->visual->geometry->type)
	    {
	      case urdf::Geometry::BOX:
		_boxes.push_back(create_box(link, Twp0));
		break;
	      case urdf::Geometry::SPHERE:
	      //spheres.push_back(create_sphere());
		break;
	      case urdf::Geometry::CYLINDER:
		break;
	      case urdf::Geometry::MESH:
		_meshes.push_back(create_mesh(link, Twp0));
		break;
	      default:
		break;
	    }
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM("(dhaiba_ros_bridge) " << err.what());
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

	const auto	Tp0j0 = transform((*child_joint)
				    ->parent_to_joint_origin_transform);
	const auto	Twj0  = Twp0 * Tp0j0;
        dhc::Link	armature_link;
        armature_link.linkName()       = (*child_joint)->child_link_name;
        armature_link.parentLinkName() = (*child_joint)->parent_link_name;
        armature_link.Twj0()           = mat44(Twj0);
        armature_link.tailPosition0()  = vec4(Twj0.getOrigin());
        _armature.links().push_back(armature_link);

	_Tj0p0[armature_link.linkName()] = Tp0j0.inverse();

        ROS_DEBUG_STREAM("create_primitives: " << _root_link->name
			 << " <== "	       << armature_link.linkName()
			 << "\n  Twj0:  "      << armature_link.Twj0());

        create_primitives(child_link, Twj0);
    }
}

void
Bridge::create_link_state(const link_cp& link,
			  dhc::LinkState& link_state) const
{
    for (const auto& child_link : link->child_links)
    {
        try
        {
	    tf::StampedTransform	Tpj;
            _listener.lookupTransform(link->name, child_link->name,
                                      ros::Time(0), Tpj);
            link_state.value().push_back(mat44(_Tj0p0[child_link->name]*Tpj));

	    ROS_DEBUG_STREAM_NAMED("link_state", "create_link_state: "
	    		     << link->name << " <== " << child_link->name
	    		     << '\n' << std::fixed << std::setprecision(3)
			     << link_state.value().back());
        }
        catch (const std::exception& err)
        {
            ROS_WARN_STREAM("(dhaiba_ros_bridge): Failed to publish link state["
			    << child_link->name << "]. " << err.what());
            link_state.value().clear();
            break;
        }

	create_link_state(child_link, link_state);
    }
}

Bridge::box_t
Bridge::create_box(const link_cp& link, const tf::Transform& Twj0)
{
    _Tjo[link->name] = transform(link->visual->origin);

    box_t	box;
    box.baseInfo() = create_base_info(link->visual->material,
				      Twj0 * _Tjo[link->name]);
    const auto&	dim = static_cast<const urdf::Box*>(link->visual->
						    geometry.get())->dim;
    box.translation().value()   = {-dim.x/2, -dim.y/2, -dim.z/2};
    box.scaling().value()       = {dim.x, dim.y, dim.z};
    box.divisionCount().value() = { 3, 3, 3 };

    return box;
}

Bridge::sphere_t
Bridge::create_sphere(const link_cp& link, const tf::Transform& Twj0)
{
    _Tjo[link->name] = transform(link->visual->origin);

    sphere_t	sphere;
    sphere.baseInfo() = create_base_info(link->visual->material,
					 Twj0 * _Tjo[link->name]);
    sphere.translation().value()  = { 0, 0, 0 };
    // sphere.scaling().value()
    // sphere.divisionCountU() = 10;
    // sphere.divisionCountV() = 10;

    return sphere;
}

Bridge::mesh_t
Bridge::create_mesh(const link_cp& link, const tf::Transform& Twj0)
{
    const auto	gmesh = static_cast<const urdf::Mesh*>(link->visual->
						       geometry.get());
    const auto	Tjo   = transform(link->visual->origin);
    _Tjo[link->name] = tf::Transform(Tjo.getBasis().scaled({gmesh->scale.x,
							    gmesh->scale.y,
							    gmesh->scale.z}),
				     Tjo.getOrigin());
			
    mesh_t	mesh;
    mesh.baseInfo() = create_base_info(link->visual->material,
				       Twj0 * _Tjo[link->name]);
    mesh.fileData().data() = loadMesh(gmesh->filename);
    std::string	file_extension = "";
    const auto	pos = gmesh->filename.rfind(".");
    if (pos != std::string::npos)
        file_extension = gmesh->filename.substr(pos + 1);
    mesh.fileExtension() = file_extension;
    mesh.description()   = gmesh->filename;

    return mesh;
}

Bridge::base_info_t
Bridge::create_base_info(const material_cp& material,
			 const tf::Transform& Two0)
{
    base_info_t	base_info;
    if (material)
    {
	base_info.color().r() = short(255*material->color.r);
	base_info.color().g() = short(255*material->color.g);
	base_info.color().b() = short(255*material->color.b);
    }
    else
    {
	base_info.color().r() = 128;
	base_info.color().g() = 128;
	base_info.color().b() = 128;
    }
    base_info.transform() = mat44(Two0, true);

    return base_info;
}

void
Bridge::publish_geometry_state(const link_cp& link) const
{
    ROS_DEBUG_STREAM_NAMED(log_element, "send_visual_state: " << link->name);
    
    if (link->visual && link->visual->geometry)
    {
	tf::StampedTransform	Twj;

	try
	{
	    _listener.lookupTransform(_root_link->name, link->name,
				      ros::Time(0), Twj);
	}
        catch (const std::exception& err)
        {
            ROS_WARN_STREAM("(dhaiba_ros_bridge): Failed to publish geometry state["
			    << link->name << "]. " << err.what());
            return;
        }
	
	dhc::GeometryState	gstate;
	gstate.transform() = mat44(Twj * _Tjo[link->name], true);
	    
	switch (link->visual->geometry->type)
	{
	  case urdf::Geometry::BOX:
	    _shape_box_pub->write(&gstate);
	    break;
	  case urdf::Geometry::SPHERE:
	    _shape_sphere_pub->write(&gstate);
	    break;
	  case urdf::Geometry::CYLINDER:
	    break;
	  case urdf::Geometry::MESH:
	    _mesh_pub->write(&gstate);
	    break;
	  default:
	    break;
	}
    }

    for (const auto& child_link : link->child_links)
	publish_geometry_state(child_link);
}

}        // namespace dhaiba_ros

/************************************************************************
*  main function                                                        *
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dhaiba_ros_bridge");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    //                                ros::console::levels::Debug);

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
