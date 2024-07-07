/*!
*  \file	dhaiba_note.h
*  \author	Toshio UESHIBA
*  \brief	publisher/subscriber for exchanging messages between ROS and DhaibaWorks
*/
#pragma once

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/Manager.h>
#include <DhaibaConnectN/PublisherInfo.h>
#include <DhaibaConnectN/SubscriberInfo.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

namespace dhaiba_ros
{
/************************************************************************
*  class NotePublisher							*
************************************************************************/
class NotePublisher
{
  public:
		NotePublisher(const std::string& participant,
			      const std::string& element)		;
		~NotePublisher()					;

    void	write(const std::string& data)				;

    int		my_test()			const	{ return 1; }

  private:
    DhaibaConnect::PublisherInfo*	pubDef;
    DhaibaConnect::PublisherInfo*	pubCur;

};

/************************************************************************
*  class NoteSubscriber							*
************************************************************************/
class NoteSubscriber
{
  public:
    using	callback_t = std::function<void(const std::string&)>;

  public:
		NoteSubscriber(const std::string& participant,
			       const std::string& element)		;
		~NoteSubscriber()					;

    void	registerCallback(const callback_t& callback)		;
    int		my_test()			const	{ return 2; }

  private:
    DhaibaConnect::SubscriberInfo*	subDef;
    DhaibaConnect::SubscriberInfo*	subCur;
};

}	/* namespace dhaiba_ros */
