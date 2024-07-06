#include <iostream>
#include "dhaiba_note.h"

namespace dhaiba_ros
{
void
doJob(const std::string& participant, const std::string& element)
{
    NotePublisher	pub(participant, element);

    std::cerr << ">> ";
    for (std::string str; std::getline(std::cin, str); )
    {
        std::cout << "getline[" << str << "]" << std::endl;
	pub.write(str);

	std::cerr << ">> ";
    }
}
}

int
main()
{

    dhaiba_ros::doJob("dhaiba_bridge", "tf");

    return 0;
}
