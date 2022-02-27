#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/Manager.h>
#include <DhaibaConnectN/SubscriberInfo.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

int
main()
{
    const auto	manager = DhaibaConnect::Manager::instance();
    manager->initialize("DhaibaConectNoteSub");

    const auto	subDef = manager->createSubscriber(
				"PARTICIPANT216407/pickingStateNote.Note::Definition",
				"dhc::Text", false, true);
    const auto	subCur = manager->createSubscriber(
				"PARTICIPANT216407/pickingStateNote.Note::CurrentText",
				"dhc::Text", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&, subDef](DhaibaConnect::SubscriberInfo* sub)
			  {
			      std::cout << "Definition data received."
					<< std::endl;
			      dhc::Text			note;
			      DhaibaConnect::SampleInfo	sampleInfo;
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
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      std::cout << "CurrentText data received."
					<< std::endl;
			      dhc::Text			note;
			      DhaibaConnect::SampleInfo	sampleInfo;
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
