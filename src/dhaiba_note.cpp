/*!
*  \file	dhaiba_note.cpp
*  \author	Toshio UESHIBA
*  \brief	publisher/subscriber for exchanging messages between ROS and DhaibaWorks
*/
#include <iostream>
#include <thread>
#include <chrono>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
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
    // std::cerr << "*** NotePublisher::write(): " << data << std::endl;
    dhc::Text note;
    if (data.size() > 255)
    {
	std::cerr << "*** NotePublisher::wirte(): data size["
		  << data.size() << "] exceeds limit[255]"
		  << std::endl;
      //return;
	note.value() = data.substr(0, 255);
    }
    else
	note.value() = data;
    pubCur->write(&note);
}

/************************************************************************
*  class NoteSubscriber							*
************************************************************************/
NoteSubscriber::NoteSubscriber(const std::string& participant,
			       const std::string& element)
    :subDef(nullptr), subCur(nullptr)
{
    std::cerr << "*** NoteSubscriber::NoteSubscriber(): "
	      << element << ".Note" << std::endl;

    const auto	manager = DhaibaConnect::Manager::instance();
    if (manager->participantName() != participant ||
        !manager->isInitialized())
    {
        manager->initialize(participant);
        std::cerr << "*** NoteSubscriber::NoteSubscriber(): initialize manager"
		  << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    subDef = manager->createSubscriber(element + ".Note::Definition",
				       "dhc::Text", false, true);
    subCur = manager->createSubscriber(element + ".Note::CurrentText",
				       "dhc::Text", false, false);

    Connections::connect(&subDef->newDataMessage,
			 {[&](DhaibaConnect::SubscriberInfo* sub)
			  {
			      dhc::Text			note;
			      DhaibaConnect::SampleInfo	sampleInfo;
			      if (!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if (sampleInfo.instanceState !=
				  DhaibaConnect::InstanceStateKind
					       ::ALIVE_INSTANCE_STATE)
				  return;
			      std::cerr << "*** Definition data: "
					<< note.value() << std::endl;
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

void
NoteSubscriber::registerCallback(const callback_t& callback)
{
    Connections::connect(&subCur->newDataMessage,
			 {[callback](DhaibaConnect::SubscriberInfo* sub)
			  {
			      dhc::Text			note;
			      DhaibaConnect::SampleInfo	sampleInfo;
			      if(!sub->takeNextData(&note, &sampleInfo))
				  return;
			      if(sampleInfo.instanceState !=
				 DhaibaConnect::InstanceStateKind
					      ::ALIVE_INSTANCE_STATE)
				  return;
			      std::cerr << "*** Current data: "
					<< note.value() << std::endl;

			      try
			      {
				  callback(note.value());
			      }
			      catch (std::exception& err)
			      {
				  std::cerr << "*** err: " << err.what()
					    << std::endl;

			      }
			  }});
}
}	// namespace dhaiba_ros

namespace py = pybind11;

PYBIND11_MODULE(dhaiba_ros, m)
{
    m.doc() = R"pbdoc(Pybind11 dhaiba plugin)pbdoc";

    using namespace dhaiba_ros;

    py::class_<NotePublisher>(m, "note_publisher")
        .def(py::init<const std::string&, const std::string&>())
        .def("write", &NotePublisher::write)
        .def("my_test", &NotePublisher::my_test);

    py::class_<NoteSubscriber>(m, "note_subscriber")
        .def(py::init<const std::string&, const std::string&>())
        .def("register_callback", &NoteSubscriber::registerCallback)
        .def("my_test", &NoteSubscriber::my_test);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
