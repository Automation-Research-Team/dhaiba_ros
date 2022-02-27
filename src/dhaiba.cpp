#include <iostream>
#include "dhaiba.h"

namespace dhaiba_ros
{

note_publisher::note_publisher()
    :pubDef(nullptr), pubCur(nullptr)
{
}

note_publisher::note_publisher(const std::string& participantName,
			       const std::string& topicNameStartsWith)
    :pubDef(nullptr), pubCur(nullptr)
{
    std::cout << "\n#note_publisher# "
	      << participantName << " " << topicNameStartsWith << std::endl;

    const auto	manager = DhaibaConnect::Manager::instance();
    if (manager->participantName() != participantName ||
	!manager->isInitialized())
    {
        manager->initialize(participantName);
        std::cout << "\n#note_publisher# manager initialize" << std::endl;
    }

    pubDef = manager->createPublisher(topicNameStartsWith + "::Definition",
				      "dhc::Text", false, true);
    pubCur = manager->createPublisher(topicNameStartsWith + "::CurrentText",
				      "dhc::Text", false, false);

    Connections::connect(&pubDef->matched,
			 {[&](DhaibaConnect::PublisherInfo* pub,
			      DhaibaConnect::MatchedStatus* info)
			  {
			      dhc::Text note;
			      note.value() = "";
			      pub->write(&note);
			  }});
}

note_publisher::~note_publisher()
{
  // std::cout << "\n#note_publisher::~note_publisher#" << std::endl;
    const auto	manager = DhaibaConnect::Manager::instance();
    if (pubCur)
        manager->removePublisher(pubCur);
    if (pubDef)
        manager->removePublisher(pubDef);
}

void
note_publisher::write(const std::string& data)
{
    std::cout << "\n# data #\n" << data << std::endl;
    dhc::Text note;
    note.value() = data;
    pubCur->write(&note);
}

note_subscriber::note_subscriber()
    :subDef(nullptr), subCur(nullptr)
{
}

note_subscriber::note_subscriber(const std::string& participantName,
				 const std::string& topicNameStartsWith,
				 const std::function<void(const std::string&)>&
				 callback)
    :subDef(nullptr), subCur(nullptr)
{
    std::cout << "\n#note_subscriber# "
                << participantName << " " << topicNameStartsWith << std::endl;

    const auto	manager = DhaibaConnect::Manager::instance();
    if (manager->participantName() != participantName ||
        !manager->isInitialized())
    {
        manager->initialize(participantName);
        std::cout << "\n#note_subscriber# manager initialize" << std::endl;
    }

    subDef = manager->createSubscriber(topicNameStartsWith + "::Definition",
				       "dhc::Text", false, true);
    subCur = manager->createSubscriber(topicNameStartsWith + "::CurrentText",
				       "dhc::Text", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      std::cout << "Definition data received."
					<< std::endl;
			      dhc::Text note;
			      DhaibaConnect::SampleInfo sampleInfo;
			      if (!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if (sampleInfo.dataChangeType !=
				  DhaibaConnect::ALIVE)
				  return;
			      std::cout << "  Note message: " << note.value()
					<< std::endl;
			  }});

    Connections::connect(&subCur->newDataMessage,
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      dhc::Text note;
			      DhaibaConnect::SampleInfo sampleInfo;
			      if(!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.dataChangeType !=
				 DhaibaConnect::ALIVE)
				  return;
			      std::cout << "\n# data #\n" << note.value()
					<< std::endl;
			      callback(note.value());
			  }});

    std::cout << "Press any key and return to quit: " << std::endl;
    std::string s;
    std::cin >> s;
}

note_subscriber::~note_subscriber()
{
  // std::cout << "\n#note_subscriber::~note_subscriber#" << std::endl;
    const auto	manager = DhaibaConnect::Manager::instance();
    if (subCur)
        manager->removeSubscriber(subCur);
    if (subDef)
        manager->removeSubscriber(subDef);
}

}	/* namespace dhaiba_ros */
