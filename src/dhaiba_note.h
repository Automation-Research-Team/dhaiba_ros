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
*  class node_publisher							*
************************************************************************/
class note_publisher
{
  public:
		note_publisher()					;
		~note_publisher()					;

		note_publisher(const std::string& participantName,
			       const std::string& topicNameStartsWith)	;

    void	write(const std::string& data)				;

    int		my_test()			const	{ return 1; }

  private:
    DhaibaConnect::PublisherInfo*	pubDef;
    DhaibaConnect::PublisherInfo*	pubCur;

};

/************************************************************************
*  class node_subscriber						*
************************************************************************/
class note_subscriber
{
  public:
		note_subscriber()					;
		~note_subscriber()					;

		note_subscriber(const std::string& participantName,
				const std::string& topicNameStartsWith,
				const std::function<void(const std::string&)>&
					callback)			;

    int		my_test()			const	{ return 2; }

  private:
    DhaibaConnect::SubscriberInfo*	subDef;
    DhaibaConnect::SubscriberInfo*	subCur;
};

}	/* namespace dhaiba_ros */
