#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

int
main()
{
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
			      dhc::String	note;
			      note.value() = "";
			      pub->write(&note);
			  }});

    std::string data, str;
    while (std::getline(std::cin, str))
    {
        std::cout << "getline[" << str << "]" << std::endl;
        if (str == "---")
        {
            std::cout << "data[" << data << "]" << std::endl;
            dhc::String note;
            note.value() = data;
            pubCur->write(&note);
            data = "";
            continue;
        }
        data += str;
        data += "\n";
    }

    return 0;
}
