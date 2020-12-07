#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>


int
main()
{
    using namespace DhaibaConnect;

    const auto	manager = Manager::instance();
    manager->initialize("DhaibaConectNoteSub");

    const auto	subDef = manager->createSubscriber(
				"PARTICIPANT216407/pickingStateNote.Note::Definition",
				"dhc::String", false, true);
    const auto	subCur = manager->createSubscriber(
				"PARTICIPANT216407/pickingStateNote.Note::CurrentText",
				"dhc::String", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&, subDef](SubscriberInfo* sub)
			  {
			      std::cout << "Definition data received." << std::endl;
			      dhc::String	note;
			      SampleInfo	sampleInfo;
			      if (!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.dataChangeType
				 != DhaibaConnect::ALIVE)
				  return;

			      std::cout << "  Note message: " << note.value()
					<< std::endl;
			      // manager->removeSubscriber(subDef);
			  }});


    Connections::connect(&subCur->newDataMessage,
			 {[&](SubscriberInfo* sub)
			  {
			      std::cout << "CurrentText data received." << std::endl;
			      dhc::String	note;
			      SampleInfo	sampleInfo;
			      if(!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.dataChangeType
				 != DhaibaConnect::ALIVE)
				  return;

			      std::cout << "  Note message: " << note.value()
					<< std::endl;
			  }});

    std::cout << "Press any key and return to quit: " << std::endl;
    std::string s;
    std::cin >> s;
    return 0;
}
