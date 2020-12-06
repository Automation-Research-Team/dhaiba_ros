#include <iostream>

#include "dhaiba.h"

namespace dhaiba_ros
{

note_publisher::note_publisher(
                const std::string& participantName,
                const std::string& topicNameStartsWith
                )
{
    std::cout << "\n#note_publisher# "
                << participantName << " " << topicNameStartsWith << std::endl;

    const auto manager = Manager::instance();
    if (manager->participantName() != participantName ||
        ! manager->isInitialized())
    {
        manager->initialize(participantName);
        std::cout << "\n#note_publisher# manager initialize" << std::endl;
    }

    pubDef = manager->createPublisher(
                                topicNameStartsWith + "::Definition",
                                "dhc::String", false, true);
    pubCur = manager->createPublisher(
                                topicNameStartsWith + "::CurrentText",
                                "dhc::String", false, false);

    Connections::connect(&pubDef->matched,
        {[&](PublisherInfo* pub, MatchingInfo* info)
            {
                dhc::String note;
                note.value() = "";
                pub->write(&note);
            }});
}

void note_publisher::write(const std::string& data)
{
    std::cout << "\n# data #\n" << data << std::endl;
    dhc::String note;
    note.value() = data;
    pubCur->write(&note);
}

note_subscriber::note_subscriber(
                const std::string& participantName,
                const std::string& topicNameStartsWith,
                const std::function<void(const std::string&)>& callback
                )
{
    std::cout << "\n#note_subscriber# "
                << participantName << " " << topicNameStartsWith << std::endl;

    const auto manager = Manager::instance();
    if (manager->participantName() != participantName ||
        ! manager->isInitialized())
    {
        manager->initialize(participantName);
        std::cout << "\n#note_subscriber# manager initialize" << std::endl;
    }

    const auto subDef = manager->createSubscriber(
                                topicNameStartsWith + "::Definition",
                                "dhc::String", false, true);
    const auto subCur = manager->createSubscriber(
                                topicNameStartsWith + "::CurrentText",
                                "dhc::String", false, false);

    Connections::connect(&subDef->newDataMessage,
        {[&, subDef](SubscriberInfo* sub)
            {
                std::cout << "Definition data received." << std::endl;
                dhc::String note;
                SampleInfo sampleInfo;
                if (!sub->takeNextData(&note, &sampleInfo))
                    return;
                if(sampleInfo.dataChangeType != DhaibaConnect::ALIVE)
                    return;
                std::cout << "  Note message: " << note.value() << std::endl;
                // manager->removeSubscriber(subDef);
            }});

    Connections::connect(&subCur->newDataMessage,
        {[&](SubscriberInfo* sub)
            {
                dhc::String note;
                SampleInfo sampleInfo;
                if(!sub->takeNextData(&note, &sampleInfo))
                    return;
                if(sampleInfo.dataChangeType != DhaibaConnect::ALIVE)
                    return;
                std::cout << "\n# data #\n" << note.value() << std::endl;
                callback(note.value());
            }});

    std::cout << "Press any key and return to quit: " << std::endl;
    std::string s;
    std::cin >> s;
}

} /* namespace dhaiba_ros */
