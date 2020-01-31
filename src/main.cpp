/*!
* \file		main.cpp
* \author	Toshio UESHIBA
* \brief	Bridge software betwenn ROS and DhaibaWorks
*/
#include <ros/ros.h>
#include "Bridge.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dhaiba_ros_bridge");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	const dhaiba_ros::Bridge	bridge("~");
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
