/*!
*  \file	dhaiba_note.cpp
*  \author	Toshio UESHIBA
*  \brief	publisher/subscriber for exchanging messages between ROS and DhaibaWorks
*/
#include <iostream>
#include "dhaiba_note.h"

namespace dhaiba_ros
{
/************************************************************************
*  class NotePublisher							*
************************************************************************/
NotePublisher::NotePublisher(const std::string& participant,
			     const std::string& element)
    :pubDef(nullptr), pubCur(nullptr)
{
    std::cerr << "*** NotePublisher::NotePublisher(): "
	      << participant << '/' << element << ".Note" << std::endl;

    const auto	manager = DhaibaConnect::Manager::instance();
    if (manager->participantName() != participant ||
	!manager->isInitialized())
    {
        manager->initialize(participant);
        std::cerr << "*** NotePublisher::NotePublisher(): initialize manager"
		  << std::endl;
    }

    pubDef = manager->createPublisher(element + ".Note::Definition",
				      "dhc::Text", false, true);
    pubCur = manager->createPublisher(element + ".Note::CurrentText",
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

NotePublisher::~NotePublisher()
{
    std::cerr << "*** NotePublisher::~NotePublisher()" << std::endl;
    const auto	manager = DhaibaConnect::Manager::instance();
    if (pubCur)
        manager->removePublisher(pubCur);
    if (pubDef)
        manager->removePublisher(pubDef);
}

void
NotePublisher::write(const std::string& data)
{
    std::cerr << "*** NotePublisher::write(): " << data << std::endl;
    if (data.size() > 255)
    {
	std::cerr << "*** NotePublisher::wirte(): data size["
		  << data.size() << "] exceeds limit[255]"
		  << std::endl;
	return;
    }
    dhc::Text note;
    note.value() = data;
    pubCur->write(&note);
}

/************************************************************************
*  class NoteSubscriber							*
************************************************************************/
NoteSubscriber::NoteSubscriber(const std::string& participant,
			       const std::string& element,
			       const std::function<void(const std::string&)>&
				     callback)
    :subDef(nullptr), subCur(nullptr)
{
    std::cerr << "*** NoteSubscriber::NoteSubscriber(): "
	      << participant << '/' << element << ".Note" << std::endl;

    const auto	manager = DhaibaConnect::Manager::instance();
    if (manager->participantName() != participant ||
        !manager->isInitialized())
    {
        manager->initialize(participant);
        std::cerr << "*** NoteSubscriber::NoteSubscriber(): initialize manager"
		  << std::endl;
    }

    subDef = manager->createSubscriber(element + ".Note::Definition",
				       "dhc::Text", false, true);
    subCur = manager->createSubscriber(element + ".Note::CurrentText",
				       "dhc::Text", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      std::cerr << "Definition data received."
					<< std::endl;
			      dhc::Text note;
			      DhaibaConnect::SampleInfo sampleInfo;
			      if (!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if (sampleInfo.instanceState !=
				  DhaibaConnect::InstanceStateKind
					       ::ALIVE_INSTANCE_STATE)
				  return;
			      std::cerr << "*** Note message: " << note.value()
					<< std::endl;
			  }});

    Connections::connect(&subCur->newDataMessage,
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      dhc::Text note;
			      DhaibaConnect::SampleInfo sampleInfo;
			      if(!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.instanceState !=
				 DhaibaConnect::InstanceStateKind
					      ::ALIVE_INSTANCE_STATE)
				  return;
			      std::cerr << "*** data: " << note.value()
					<< std::endl;
			      callback(note.value());
			  }});
}

NoteSubscriber::~NoteSubscriber()
{
    std::cerr << "*** NoteSubscriber::~NoteSubscriber()" << std::endl;
    const auto	manager = DhaibaConnect::Manager::instance();
    if (subCur)
        manager->removeSubscriber(subCur);
    if (subDef)
        manager->removeSubscriber(subDef);
}

}	/* namespace dhaiba_ros */
