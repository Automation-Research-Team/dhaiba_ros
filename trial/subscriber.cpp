#include <iostream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

using namespace DhaibaConnect;

std::ostream&
operator << (std::ostream& out, const dhc::Mat44& mat)
{
    return out << '['
           << mat.value()[0]  << ", " << mat.value()[1]  << ", "
           << mat.value()[2]  << ", " << mat.value()[3]  << ", " << '\n'
           << mat.value()[4]  << ", " << mat.value()[5]  << ", "
           << mat.value()[6]  << ", " << mat.value()[7]  << ", " << '\n'
           << mat.value()[8]  << ", " << mat.value()[9]  << ", "
           << mat.value()[10] << ", " << mat.value()[11] << ", " << '\n'
           << mat.value()[12] << ", " << mat.value()[13] << ", "
           << mat.value()[14] << ", " << mat.value()[15] << ']';
}

std::ostream&
operator << (std::ostream& out, const dhc::Color& c)
{
    return out << "(r, g, b)=("
                << c.r() << ", " << c.g() << ", " << c.b() << ")";
}

int main(int argc, char *argv[])
{
    Manager* manager = Manager::instance();
    manager->initialize("stl_sub");

    std::string topic_name = "stl1/TrialSTL1.Mesh::Definition_BinaryFile";
    if (argc > 1) {
        switch (argv[1][0]) {
        case '3':
            topic_name = "stl3/TrialSTL3.Mesh::Definition_BinaryFile";
            break;
        case '1':
        default:
            break;
        }
    }

    SubscriberInfo* sub_stl1 = manager->createSubscriber(
                topic_name,
                "dhc::GeometryBinaryFile", false, true);
    Connections::connect(&sub_stl1->newDataMessage,
    {[&, sub_stl1](SubscriberInfo* sub){
        std::cout << "data received." << std::endl;
        dhc::GeometryBinaryFile data;  SampleInfo sampleInfo;
        if(!sub->takeNextData(&data, &sampleInfo)) return;
        if(sampleInfo.dataChangeType != DhaibaConnect::ALIVE) return;
        std::cout << "[" << sub->topicName() << "]"
            << "\n  baseInfo transform :\n" << data.baseInfo().transform()
            << "\n  baseInfo color : " << data.baseInfo().color()
            << "\n  fileExtension  : " << data.fileExtension()
            << "\n  description    : " << data.description()
            << "\n  fileData size  : " << data.fileData().data().size()
            << std::endl;
        for (int i = 0; i < 16; i++) {
            printf("%02X ", data.fileData().data()[i]);
        }
        printf("\n");
        manager->removeSubscriber(sub);
    }});

    std::cout << "Press any key and return to quit: " << std::endl;
    std::string s;
    std::cin >> s;
    return 0;
}
