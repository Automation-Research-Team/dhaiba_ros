#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/Manager.h>
#include <DhaibaConnectN/PublisherInfo.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

int
main()
{
    const auto	manager = DhaibaConnect::Manager::instance();
    manager->initialize("DhaibaConectNotePub");

    const auto	pubDef = manager->createPublisher(
					"SampleNote.Note::Definition",
					"dhc::Text", false, true);
    const auto	pubCur = manager->createPublisher(
					"SampleNote.Note::CurrentText",
					"dhc::Text", false, false);

    Connections::connect(&pubDef->matched,
			 {[&](DhaibaConnect::PublisherInfo* pub,
			      DhaibaConnect::MatchedStatus* info)
			     {
				 dhc::Text	note;
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
            dhc::Text note;
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
