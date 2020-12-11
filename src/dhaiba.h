#ifndef __DHAIBA_H
#define __DHAIBA_H

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

namespace dhaiba_ros
{
    using namespace DhaibaConnect;

class note_publisher
{
public:
    note_publisher();
    ~note_publisher();

    note_publisher(
                const std::string& participantName,
                const std::string& topicNameStartsWith
                );

    void write(const std::string& data);

    int my_test() { return 1; }

private:
    PublisherInfo* pubDef;
    PublisherInfo* pubCur;

};

class note_subscriber
{
public:
    note_subscriber();
    ~note_subscriber();

    note_subscriber(
                const std::string& participantName,
                const std::string& topicNameStartsWith,
                const std::function<void(const std::string&)>& callback
                );

    int my_test() { return 2; }

private:
    SubscriberInfo* subDef;
    SubscriberInfo* subCur;

};

} /* namespace dhaiba_ros */

#endif /* __DHAIBA_H */
