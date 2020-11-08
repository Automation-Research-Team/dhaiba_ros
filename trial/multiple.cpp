#include <iostream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include <vector>
#include <map>
#include <thread>
#include <math.h>

#include "trial_com.h"

using namespace DhaibaConnect;


void _geometry_binary_file(
                DhaibaConnect::PublisherInfo* pub,
                const std::vector<char>& mesh_data,
                const dhc::Color& color,
                const dhc::Mat44& transform
                )
{
    if (mesh_data.size() <= 0) {
        std::cout << "geometry_binary_file: mesh_data is zero" << std::endl;
        return;
    }
    dhc::GeometryBinaryFile data;
    data.fileExtension() = "stl";
    data.fileData().data() = mesh_data;
    data.description() = "mesh(STL)";
    data.baseInfo().color() = color;
    data.baseInfo().transform() = transform;
    pub->write(&data);

    std::cout << "geometry_binary_file: "
            << data.fileData().data().size() << std::endl;
    std::ofstream f;
    f.open("/tmp/trial_data", std::ios_base::out | std::ios_base::binary);
    auto size = data.fileData().data().size();
    for (int i = 0; i < size; i++)
        f.put(data.fileData().data()[i]);
    f.close();
}

void _geometry_state(
        DhaibaConnect::PublisherInfo* pub, const dhc::Mat44& transform)
{
    dhc::GeometryState data;
    data.transform() = transform;
    pub->write(&data);
}

void _shape_box(DhaibaConnect::PublisherInfo* pub,
                const dhc::Color& color,
                const dhc::Mat44& transform,
                const dhc::Vec3&  scaling
                )
{
    dhc::ShapeBox data;
    data.baseInfo().color() = color;
    data.baseInfo().transform() = transform;
    data.scaling() = scaling;
    data.divisionCount().value() = { 3, 3, 3 };
    pub->write(&data);

    std::cout << "shape_box: " << std::endl;
}

void create_publisher_mesh(
                const Manager* manager,
                std::vector<PublisherInfo*>& def_pubs,
                std::vector<PublisherInfo*>& state_pubs,
                std::map<std::string, std::vector<char>>& mesh_datas,
                std::map<std::string, dhc::Color>& colors,
                std::map<std::string, dhc::Mat44>& transforms
                )
{
    for (int i = 0; i < 2; i++) {
        std::string num_str = std::to_string(i+1);

        std::vector<char> mesh_data;
        std::string url = "../link_" + num_str + ".stl";
        if (! loadSTL(url, mesh_data)) {
            std::cout << "failed loadSTL()" << std::endl;
            break;
        }
        dhc::Color color;
        color.r() = 100*i; color.g() = 0; color.b() = 255;
        dhc::Mat44 transform;
        transform.value() = {
                 1000,         0,         0, 0,
                    0,      1000,         0, 0,
                    0,         0,      1000, 0,
            100*(i+1), 200*(i+1), 300*(i+1), 1,
            };

        PublisherInfo* pub = manager->createPublisher(
                "TrialSTL" + num_str + ".Mesh::Definition_BinaryFile",
                "dhc::GeometryBinaryFile", false, true);

        def_pubs.push_back(pub);
        mesh_datas[pub->topicName()] = mesh_data;
        colors[pub->topicName()] = color;
        transforms[pub->topicName()] = transform;

        Connections::connect(&pub->matched,
            {[&](PublisherInfo* pub, MatchingInfo* info){
                _geometry_binary_file(pub,
                                    mesh_datas[pub->topicName()],
                                    colors[pub->topicName()],
                                    transforms[pub->topicName()]
                                    );
            }});
        PublisherInfo* pub2 = manager->createPublisher(
                "TrialSTL" + num_str + ".PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);
        state_pubs.push_back(pub2);

        std::cout << "\ntopicName :" << pub->topicName()
                  << "\nglobalTopicName :" << pub->globalTopicName()
                  << "\ntopicName :" << pub2->topicName()
                  << "\nglobalTopicName :" << pub2->globalTopicName()
                  << std::endl;
    }
}

void create_publisher_shapebox(
                const Manager* manager,
                std::vector<PublisherInfo*>& def_pubs,
                std::vector<PublisherInfo*>& state_pubs,
                std::map<std::string, dhc::Color>& colors,
                std::map<std::string, dhc::Mat44>& transforms,
                std::map<std::string, dhc::Vec3>&  scalings
                )
{
    for (int i = 0; i < 2; i++) {
        std::string num_str = std::to_string(i+1);

        dhc::Color color;
        color.r() = 255; color.g() = 100*i; color.b() = 0;
        dhc::Mat44 transform;
        transform.value() = {
                    1,         0,         0, 0,
                    0,         1,         0, 0,
                    0,         0,         1, 0,
             10+100*i,  20+100*i,  30+100*i, 1,
            };
        dhc::Vec3 scaling;
        scaling.value() = { 100*(i+1), 100*(i+1), 100*(i+1) };

        PublisherInfo* pub = manager->createPublisher(
                "TrialShapebox" + num_str + ".ShapeBox::Definition",
                "dhc::ShapeBox", false, true);

        def_pubs.push_back(pub);
        colors[pub->topicName()] = color;
        transforms[pub->topicName()] = transform;
        scalings[pub->topicName()] = scaling;

        Connections::connect(&pub->matched,
            {[&](PublisherInfo* pub, MatchingInfo* info){
                _shape_box(pub,
                           colors[pub->topicName()],
                           transforms[pub->topicName()],
                           scalings[pub->topicName()]
                           );
            }});

        PublisherInfo* pub2 = manager->createPublisher(
                "TrialShapebox" + num_str + ".PointSupplier::GeometryState",
                "dhc::GeometryState", false, false);
        state_pubs.push_back(pub2);

        std::cout << "\ntopicName :" << pub->topicName()
                  << "\nglobalTopicName :" << pub->globalTopicName()
                  << "\ntopicName :" << pub2->topicName()
                  << "\nglobalTopicName :" << pub2->globalTopicName()
                  << std::endl;
    }
}

int main(int argc, char *argv[])
{
    double r = 0.0;

    Manager* manager = Manager::instance();
    manager->initialize("multiple");

    std::vector<PublisherInfo*> def_pubs;
    std::vector<PublisherInfo*> state_pubs;
    std::map<std::string, std::vector<char>> mesh_datas;
    std::map<std::string, dhc::Color>        colors;
    std::map<std::string, dhc::Mat44>        transforms;
    std::map<std::string, dhc::Vec3>         scalings;

    if (argc > 1) {
        const char* p = argv[1];
        while (*p) {
            switch (*p) {
            case 'b':
                std::cout << "### shapebox ###" << std::endl;
                create_publisher_shapebox(manager,
                    def_pubs, state_pubs, colors, transforms, scalings);
                break;
            case 'm':
                std::cout << "### mesh ###" << std::endl;
                create_publisher_mesh(manager,
                    def_pubs, state_pubs, mesh_datas, colors, transforms);
                break;
            default:
                break;
            }
            p++;
        }
    } else {
        create_publisher_shapebox(manager,
                def_pubs, state_pubs, colors, transforms, scalings);
        create_publisher_mesh(manager,
                def_pubs, state_pubs, mesh_datas, colors, transforms);
    }

    std::thread th([&]() {
        while(1){
            int i1 = 0, i2 = 0;
            for (auto pub : state_pubs) {
                dhc::Mat44 transform;
                double d = 100.0 * sin(r);
                // std::cout << "loop: " << pub->topicName() << std::endl;
                if (pub->topicName().find("STL") != std::string::npos) {
                    // std::cout << "loop: STL" << std::endl;
                    transform.value() = {
                                1000,            0,            0, 0,
                                   0,         1000,            0, 0,
                                   0,            0,         1000, 0,
                        100*(i1+1)+d, 200*(i1+1)+d, 300*(i1+1)+d, 1,
                        };
                    i1++;
                } else
                if (pub->topicName().find("Shapebox") != std::string::npos) {
                    // std::cout << "loop: Shapebox" << std::endl;
                    transform.value() = {
                                   1,            0,            0, 0,
                                   0,            1,            0, 0,
                                   0,            0,            1, 0,
                         10+100*i2+d,  20+100*i2+d,  30+100*i2+d, 1,
                        };
                    i2++;
                }
                _geometry_state(pub, transform);
            }
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

