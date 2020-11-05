#include <iostream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <thread>
#include <math.h>

#include "trial_com.h"

using namespace DhaibaConnect;

void geometry_binary_file2(
                DhaibaConnect::PublisherInfo* pub, const std::string& url)
{
    try {
        std::ifstream f(url, std::ios_base::in | std::ios_base::binary);
        if (!f) {
            std::cout << "geometry_binary_file2: ("
                        << url << ") can not open" << std::endl;
            return;
        }
        f.seekg(0, std::ios_base::end);
        auto fsize = f.tellg();
        f.seekg(0);

        dhc::GeometryBinaryFile data;
        data.fileData().data().resize(fsize);
        char *tgt = data.fileData().data().data();
        f.read(tgt, fsize);
        f.close();
        data.fileExtension() = "stl";

        data.description() = "mesh(STL)";
        data.baseInfo().color().r() = 0;
        data.baseInfo().color().g() = 255;
        data.baseInfo().color().b() = 0;
        data.baseInfo().transform().value() = {
                    1000, 0, 0, 0,
                    0, 1000, 0, 0,
                    0, 0, 1000, 0,
                    0, 0, 0, 1,
                };
        pub->write(&data);
        std::cout << "geometry_binary_file2: "
                << data.fileData().data().size() << std::endl;

        std::ofstream f2("/tmp/stl_data",
                        std::ios_base::out | std::ios_base::binary);
        f2.write(data.fileData().data().data(), data.fileData().data().size());

    } catch (const std::exception& e) {
        std::cout << "geometry_binary_file2: ("
                << url << ") " << e.what() << std::endl;
        return;
    }
}

int main(int argc, char *argv[])
{
    double r = 0.0;

    std::vector<char> mesh_data;
    char *url = "./link_1.stl";
    if (argc > 1)
        url = argv[1];

    Manager* manager = Manager::instance();
    manager->initialize("stl3");

    PublisherInfo* pub = manager->createPublisher(
                "TrialSTL3.Mesh::Definition_BinaryFile",
                "dhc::GeometryBinaryFile", false, true);
    Connections::connect(&pub->matched,
                {[&](PublisherInfo* pub, MatchingInfo* info){
                    geometry_binary_file2(pub, url);
                }});

    std::cout << "\ntopicName :" << pub->topicName()
              << "\nglobalTopicName :" << pub->globalTopicName() << std::endl;

    PublisherInfo* pub2 = manager->createPublisher(
                "TrialSTL3.PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);

    std::thread th([&]() {
        while(1){
            geometry_state2(pub2, r);
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

