#include <iostream>
#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>


int
main()
{
    using namespace std;
    using namespace DhaibaConnect;

    const auto	manager = Manager::instance();
    manager->initialize("DhaibaConectNoteSub");

    const auto	subDef = manager->createSubscriber(
				"PARTICIPANT22493/myNote.Note::Definition",
				"dhc::String", false, true);
    const auto	subCur = manager->createSubscriber(
				"PARTICIPANT22493/myNote.Note::CurrentText",
				"dhc::String", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&, subDef](SubscriberInfo* sub)
			  {
			      std::cout << "Note data received." << std::endl;
			      dhc::String	note;
			      SampleInfo	sampleInfo;
			      if (!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.dataChangeType
				 != DhaibaConnect::ALIVE)
				  return;

			      std::cout << "  Note message: " << note.value()
					<< std::endl;
			      manager->removeSubscriber(subDef);
			  }});


    Connections::connect(&subCur->newDataMessage,
			 {[&](SubscriberInfo* sub)
			  {
			    //cout << "Link data received." << endl;
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

    cout << "Press any key and return to quit: " << endl;
    std::string s;
    cin >> s;
    return 0;
}
