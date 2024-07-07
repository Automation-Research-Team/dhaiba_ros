#include <iostream>
#include "dhaiba_note.h"
#include <unistd.h>

namespace dhaiba_ros
{
void
print(const std::string& value)
{
    std::cerr << "value: " << value << std::endl;
}

void
doJob(const std::string& participant, const std::string& element)
{
    NoteSubscriber	sub(participant, element);
    sub.registerCallback(print);
    for (;;)
	usleep(500 * 1000);
}
}

int
main()
{

    dhaiba_ros::doJob("dhaiba_bridge", "myNote");

    return 0;
}
