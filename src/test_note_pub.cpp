#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

int
main()
{
    using namespace std;
    using namespace DhaibaConnect;

    const auto	manager = Manager::instance();
    manager->initialize("DhaibaConectNotePub");

    const auto	pubDef = manager->createPublisher(
					"SampleNote.Note::Definition",
					"dhc::String", false, true);
    const auto	pubCur = manager->createPublisher(
					"SampleNote.Note::CurrentText",
					"dhc::String", false, false);

    Connections::connect(&pubDef->matched,
			 {[&](PublisherInfo* pub, MatchingInfo* info)
			  {
			      dhc::String	data;
			      data.value() = "";
			      pub->write(&data);
			  }});

    for (dhc::String data; std::cin >> data.value(); )
    {
	std::cout << "*** Input: " << data.value() << std::endl;
	pubCur->write(&data);
    }

    return 0;
}
