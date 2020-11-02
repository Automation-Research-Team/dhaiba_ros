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
    manager->initialize("shapesphere1");

    PublisherInfo* pub = manager->createPublisher(
                "TrialShapeSphere1.ShapeSphere::Definition",
                "dhc::ShapeSphere", false, true);
    Connections::connect(&pub->matched,
                {[&](PublisherInfo* pub, MatchingInfo* info){
                    shape_sphere(pub, r);
                }});

    PublisherInfo* pub2 = manager->createPublisher(
                "TrialShapeSphere1.PointSupplier::GeometryState",
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

