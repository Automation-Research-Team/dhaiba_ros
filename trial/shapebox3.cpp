#include <iostream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <thread>

#include "trial_com.h"

using namespace DhaibaConnect;

int main()
{
    double r = 0.0;

    Manager* manager = Manager::instance();
    manager->initialize("shapebox1");

    PublisherInfo* pub = manager->createPublisher(
                "TrialShapebox3.Mesh::Definition",
                "dhc::ShapeBox", false, true);
    Connections::connect(&pub->matched,
                {[&](PublisherInfo* pub, MatchingInfo* info){
                    shape_box(pub, r);
                }});

    PublisherInfo* pub2 = manager->createPublisher(
                "TrialShapebox3.PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);

    std::thread th([&]() {
        while(1){
            geometry_state(pub2, r);
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

