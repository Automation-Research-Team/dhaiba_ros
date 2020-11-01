#include <iostream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <thread>

#include "trial_com.h"

using namespace DhaibaConnect;

int main(int argc, char *argv[])
{
    double r = 0.0;
    std::vector<char> mesh_data;
    char *url = "./link_1.stl";
    if (argc > 1)
        url = argv[1];

    if (! loadSTL(url, mesh_data)) {
        std::cout << "failed loadSTL()" << std::endl;
        return 0;
    }

    Manager* manager = Manager::instance();
    manager->initialize("stl2");

    PublisherInfo* pub = manager->createPublisher(
                "TrialSTL2.Mesh::Definition_BinaryFile",
                "dhc::GeometryBinaryFile", false, false);

    std::thread th([&]() {
        while(1){
            geometry_binary_file(pub, mesh_data, r);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            r += 0.1;
        }
    });
    th.detach();

    std::cout << "Press any key and return to quit: " << std::endl;
    std::string s;
    std::cin >> s;
    return 0;
}

