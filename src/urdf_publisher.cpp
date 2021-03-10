/*!
*  \file	urdf_publisher.cpp
*  \author	Toshio UESHIBA
*  \brief	Bridge software for publishing URDF and TF to DhaibaWorks
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <string>
#include <unordered_map>
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

static DhaibaConnect::PublisherInfo*
create_publisher(const std::string& topicName,
		 const std::string& typeName, bool highReliability)
{
    const auto	manager = DhaibaConnect::Manager::instance();
    const auto	pub = manager->createPublisher(topicName, typeName,
					       false, highReliability);
    if (!pub)
    {
        ROS_ERROR_STREAM(
            "(urdf_publisher) Failed to create publisher for topic["
	    << topicName << "] of type[" << typeName << ']');
        throw;
    }

    return pub;
}

static DhaibaConnect::PublisherInfo*
create_publisher(const urdf::LinkConstSharedPtr& link)
{
    std::string	topicName, typeName;
    switch (link->visual->geometry->type)
    {
      case urdf::Geometry::BOX:
	topicName = link->name + ".ShapeBox::Definition";
	typeName  = "dhc::ShapeBox";
	break;
      case urdf::Geometry::SPHERE:
      	topicName = link->name + ".ShapeSphere::Definition";
      	typeName  = "dhc::ShapeSphere";
      	break;
      // case urdf::Geometry::CYLINDER:
      // 	topicName = link->name + ".ShapeCylinder::Definition";
      // 	typeName  = "dhc::ShapeCylinder";
      // 	break;
      case urdf::Geometry::MESH:
	topicName = link->name + ".Mesh::Definition_BinaryFile";
	typeName  = "dhc::GeometryBinaryFile";
	break;
      default:
	ROS_ERROR_STREAM("(urdf_publisher) Unknown geometry type["
			 << link->visual->geometry->type << ']');
	return nullptr;
    }

    return create_publisher(topicName, typeName, true);
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

static urdf::JointConstSharedPtr
find_joint(const urdf::LinkConstSharedPtr& link,
	   const urdf::LinkConstSharedPtr& child_link)
{
  // Find a joint with a child_link_name equal to the name of child_link.
    const auto	joint = std::find_if(link->child_joints.cbegin(),
				     link->child_joints.cend(),
				     [&child_link](const auto& joint)
				     {
					 return (joint->child_link_name
						 == child_link->name);
				     });
    if (joint == link->child_joints.cend())
    {
	ROS_ERROR_STREAM("(urdf_publisher) Internal inconsistency! Child link["
			 << child_link->name
			 << "] is not found in child joints of link["
			 << link->name << "].");
	throw;
    }

    return *joint;
}

/************************************************************************
*  class URDFPublisher							*
************************************************************************/
class URDFPublisher
{
  private:
    using link_cp = urdf::LinkConstSharedPtr;

    class Element
    {
      public:
		Element(const link_cp& link)				;
		~Element()						;

	void	publish_definition()				 const	;
	void	publish_geometry_state(const tf::Transform& Twj) const	;

      private:
	const urdf::VisualConstSharedPtr	_visual;
	tf::Transform				_Tjo;
	DhaibaConnect::PublisherInfo* const	_definition_pub;
	DhaibaConnect::PublisherInfo* const	_geometry_state_pub;
    };

  public:
		URDFPublisher(const std::string& name)			;
		~URDFPublisher()					;

    void	run()						const	;

  private:
    void	create_armature(const link_cp& link,
				const tf::Transform& Twp0,
				dhc::Armature& armature)	const	;
    void	create_elements(const link_cp& link)			;
    void	create_link_state(const link_cp& link,
				  dhc::LinkState& link_state,
				  bool replace_odom=false)	const	;

  private:
    const ros::NodeHandle				 _nh;
    const tf::TransformListener				 _listener;
    const double					 _rate;
    const bool						 _publish_armature;
    const bool						 _publish_elements;

    urdf::ModelInterfaceSharedPtr			 _model;
    urdf::LinkConstSharedPtr				 _root_link;
    const std::string					 _odom_frame;
    const std::string					 _replaced_odom_frame;
    std::unordered_map<std::string, const tf::Transform> _Tj0p0;
    std::unordered_map<std::string, const Element>	 _elements;

    DhaibaConnect::PublisherInfo*			 _definition_pub;
    DhaibaConnect::PublisherInfo*			 _link_state_pub;
};

URDFPublisher::URDFPublisher(const std::string& name)
    :_nh(name),
     _listener(),
     _rate(_nh.param("rate", 10.0)),
     _publish_armature(_nh.param("publish_armature", true)),
     _publish_elements(_nh.param("publish_elements", true)),
     _model(),
     _root_link(),
     _odom_frame( _nh.param<std::string>("odom_frame", "")),
     _replaced_odom_frame(_nh.param<std::string>("replaced_odom_frame",
						 _odom_frame)),
     _Tj0p0(),
     _elements(),
     _definition_pub(nullptr),
     _link_state_pub(nullptr)
{
  // Load robot model described in URDF.
    std::string	description_param("robot_description");
    _nh.param("description_param", description_param, description_param);
    std::string	description_xml;
    if (!_nh.getParam(description_param, description_xml))
    {
        ROS_ERROR_STREAM("(urdf_publisher) Failed to get parameter["
                         << description_param << ']');
        throw;
    }

    _model = urdf::parseURDF(description_xml);
    if (!_model)
    {
        ROS_ERROR_STREAM("(urdf_publisher) Failed to load urdf in parameter["
                         << description_param << ']');
        throw;
    }

  // Get all links in the model.
    std::vector<urdf::LinkSharedPtr> links;
    _model->getLinks(links);

  // Set root link from root frame name.
    const auto root_frame = _nh.param<std::string>("root_frame", "world");
    const auto root_link = std::find_if(links.cbegin(), links.cend(),
                                        [&root_frame](const auto& link)
                                        { return link->name == root_frame; });
    if (root_link == links.cend())
    {
        ROS_WARN_STREAM("(urdf_publisher) Frame \""
                        << root_frame << "\" not found.");
        _root_link = _model->getRoot();
    }
    else
        _root_link = *root_link;
    ROS_INFO_STREAM("(urdf_publisher) Set root frame to \""
                    << _root_link->name << "\".");

  // Initialize manager.
    std::string participant_name = std::regex_replace(_nh.getNamespace(),
						      std::regex("/"), "");
    ROS_DEBUG_STREAM("name[" << name << "]["
		     << _nh.getNamespace() << "][" << participant_name << "]");
    const auto	manager = DhaibaConnect::Manager::instance();
    manager->initialize(participant_name);

  // Create publishers and register callback for armature.
    if (_publish_armature)
    {
	_definition_pub = create_publisher("armature.Armature::Definition",
					   "dhc::Armature", true);
	_link_state_pub = create_publisher("armature.Armature::LinkState",
					   "dhc::LinkState", false);
	Connections::connect(&_definition_pub->matched,
			     {[this](DhaibaConnect::PublisherInfo* pub,
				     DhaibaConnect::MatchingInfo* info)
				 {
				     dhc::Armature	armature;
				     create_armature(_root_link,
						     tf::Transform(
							 {0.0, 0.0, 0.0, 1.0},
							 {0.0, 0.0, 0.0}),
						     armature);
				     pub->write(&armature);
				 }
			     });
    }

  // Create publishers for geometric elements.
    if (_publish_elements)
	create_elements(_root_link);

  // Create publisher for link state.
    ROS_INFO_STREAM("(urdf_publisher) Node[" << name << "] initialized.");
}

URDFPublisher::~URDFPublisher()
{
    if (_publish_armature)
    {
	const auto	manager = DhaibaConnect::Manager::instance();
	manager->removePublisher(_link_state_pub);
	manager->removePublisher(_definition_pub);
    }
}

void
URDFPublisher::run() const
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
URDFPublisher::create_armature(const link_cp& link, const tf::Transform& Twp0,
			       dhc::Armature& armature) const
{
    for (const auto& child_link : link->child_links)
    {
	const auto	child_joint = find_joint(link, child_link);
	const auto	Tp0j0 = transform(child_joint
					  ->parent_to_joint_origin_transform);
	const auto	Twj0  = Twp0 * Tp0j0;
        dhc::Link	armature_link;
        armature_link.linkName()       = child_joint->child_link_name;
        armature_link.parentLinkName() = child_joint->parent_link_name;
        armature_link.Twj0()           = mat44(Twj0);
        armature_link.tailPosition0()  = vec4(Twj0.getOrigin());
        armature.links().push_back(armature_link);

	_Tj0p0[armature_link.linkName()] = Tp0j0.inverse();

        ROS_DEBUG_STREAM("create_armature: " << _root_link->name
			 << " <== "	     << armature_link.linkName()
			 << "\n  Twj0:  "    << armature_link.Twj0());

        create_armature(child_link, Twj0, armature);
    }
}

void
URDFPublisher::create_elements(const link_cp& link)
{
    if (link->visual && link->visual->geometry)
    	_elements.emplace(std::piecewise_construct,
			  std::forward_as_tuple(link->name),
			  std::forward_as_tuple(link));

    for (const auto& child_link : link->child_links)
        create_elements(child_link);
}

void
URDFPublisher::create_link_state(const link_cp& link,
				 dhc::LinkState& link_state,
				 bool replace_odom) const
{
    if (link->name == _odom_frame)
	replace_odom = true;

    if (_publish_elements && link->visual && link->visual->geometry)
    {
	try
	{
	    tf::StampedTransform	Twj;

	    if (replace_odom)
	    {
		_listener.lookupTransform(_root_link->name,
					  _replaced_odom_frame,
					  ros::Time(0), Twj);
		tf::StampedTransform	Toj;
		_listener.lookupTransform(_odom_frame, link->name,
					  ros::Time(0), Toj);
		Twj *= Toj;
	    }
	    else
		_listener.lookupTransform(_root_link->name, link->name,
					  ros::Time(0), Twj);
	    _elements.find(link->name)->second.publish_geometry_state(Twj);
	}
        catch (const std::exception& err)
        {
            ROS_WARN_STREAM("(urdf_publisher): Failed to publish geometry state["
			    << link->name << "]. " << err.what());
        }
    }

    for (const auto& child_link : link->child_links)
    {
	if (_publish_armature)
	{
	    try
	    {
		tf::StampedTransform	Tpj;
		_listener.lookupTransform(link->name, child_link->name,
					  ros::Time(0), Tpj);
		link_state.value().push_back(mat44(_Tj0p0[child_link->name]*
						   Tpj));

		ROS_DEBUG_STREAM_NAMED("link_state", "create_link_state: "
				       << link->name << " <== "
				       << child_link->name << '\n'
				       << std::fixed << std::setprecision(3)
				       << link_state.value().back());
	    }
	    catch (const std::exception& err)
	    {
		link_state.value().push_back(mat44(_Tj0p0[child_link->name]));

		ROS_WARN_STREAM("(urdf_publisher): Failed to create link state["
				<< child_link->name << "]. " << err.what());
	    }
	}

	create_link_state(child_link, link_state, replace_odom);
    }
}

/************************************************************************
*  class URDFPublisher::Element						*
************************************************************************/
URDFPublisher::Element::Element(const link_cp& link)
    :_visual(link->visual),
     _Tjo(),
     _definition_pub(create_publisher(link)),
     _geometry_state_pub(create_publisher(link->name +
					  ".PointSupplier::GeometryState",
					  "dhc::GeometryState", false))
{
    if (_visual->geometry->type == urdf::Geometry::MESH)
    {
	const auto&	scale = static_cast<const urdf::Mesh*>(
				    _visual->geometry.get())->scale;
	const auto	Tjo   = transform(_visual->origin);
	_Tjo = tf::Transform(Tjo.getBasis().scaled({scale.x,
						    scale.y, scale.z}),
			     Tjo.getOrigin());
    }
    else
	_Tjo = transform(_visual->origin);

    if (!_definition_pub)
	return;

    Connections::connect(&_definition_pub->matched,
			 {[this](DhaibaConnect::PublisherInfo* pub,
				 DhaibaConnect::MatchingInfo* into)
			  {
			      publish_definition();
			  }});
}

URDFPublisher::Element::~Element()
{
    const auto	manager = DhaibaConnect::Manager::instance();
    manager->removePublisher(_geometry_state_pub);
    manager->removePublisher(_definition_pub);
}

void
URDFPublisher::Element::publish_definition() const
{
    dhc::GeometryBaseInfo	base_info;
    if (_visual->material)
    {
	base_info.color().r() = short(255*_visual->material->color.r);
	base_info.color().g() = short(255*_visual->material->color.g);
	base_info.color().b() = short(255*_visual->material->color.b);
    }
    else
    {
	base_info.color().r() = 128;
	base_info.color().g() = 128;
	base_info.color().b() = 128;
    }
    base_info.transform() = mat44(_Tjo, true);

    switch (_visual->geometry->type)
    {
      case urdf::Geometry::BOX:
      {
	const auto&	dim = static_cast<const urdf::Box*>(
				_visual->geometry.get())->dim;
	dhc::ShapeBox	box;
	box.baseInfo()		    = base_info;
	box.translation().value()   = { -dim.x/2, -dim.y/2, -dim.z/2 };
	box.scaling().value()       = { dim.x, dim.y, dim.z };
	box.divisionCount().value() = { 3, 3, 3 };

	_definition_pub->write(&box);
      }
	break;

      case urdf::Geometry::SPHERE:
      {
	const auto	 radius = static_cast<const urdf::Sphere*>(
				     _visual->geometry.get())->radius;
	dhc::ShapeSphere sphere;
	sphere.baseInfo()	     = base_info;
	sphere.translation().value() = { 0, 0, 0 };
	sphere.scaling().value()     = { radius, radius, radius };
	sphere.divisionCountU()	     = 3;
	sphere.divisionCountV()      = 3;

	_definition_pub->write(&sphere);
      }
        break;

      case urdf::Geometry::CYLINDER:
      {
	const auto	   gcylinder = static_cast<const urdf::Cylinder*>(
					   _visual->geometry.get());
	// dhc::ShapeCylinder cylinder;
	// cylinder.baseInfo()		= base_info;
	// cylinder.translation().value()	= { 0, 0, -gcylinder->length/2 };
	// cylinder.scaling().value()	= { gcylinder->radius,
	// 				    gcylinder->radius,
	// 				    gcylinder->length };

	// _definition_pub->write(&cylinder);
      }
	break;

      case urdf::Geometry::MESH:
      {
	const auto		mesh = static_cast<const urdf::Mesh*>(
					   _visual->geometry.get());
	dhc::GeometryBinaryFile	file;
	file.baseInfo()		= base_info;
	file.fileData().data()	= loadMesh(mesh->filename);
	const auto	pos = mesh->filename.rfind(".");
	file.fileExtension()	= (pos != std::string::npos ?
				   mesh->filename.substr(pos + 1) : "");
	file.description()	= mesh->filename;

	_definition_pub->write(&file);
      }
	break;

      default:
	ROS_ERROR_STREAM("(urdf_publisher) Unknown geometry type["
			 << _visual->geometry->type << ']');
	throw;
    }
}

void
URDFPublisher::Element::publish_geometry_state(const tf::Transform& Twj) const
{
    dhc::GeometryState	state;
    state.transform() = mat44(Twj * _Tjo, true);
    _geometry_state_pub->write(&state);
}

}        // namespace dhaiba_ros

/************************************************************************
*  main function                                                        *
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "urdf_publisher");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    //                                ros::console::levels::Debug);

    try
    {
        const dhaiba_ros::URDFPublisher	publisher("~");
        publisher.run();
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
