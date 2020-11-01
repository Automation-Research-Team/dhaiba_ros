#include <string>
#include <map>
#include <iostream>
#include <fstream>

#include <math.h>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

#include "trial_com.h"

bool loadSTL(const std::string& url, std::vector<char>& data)
{
    std::cout << "loadSTL: url=" << url << std::endl;
    data.clear();
    // std::cout << "loadSTL: data max_size=" << data.max_size() << std::endl;

    const char* type = "file://";

    std::string::size_type pos;
    std::string path;

    pos = url.find(type, 0);
    if (pos != std::string::npos) {
        path = url.substr(pos+strlen(type));
    } else {
        path = url;
    }

    std::cout << "loadSTL: path=" << path << std::endl;
    if (path.size() <= 0) {
        std::cout << "loadSTL: path size is 0 << std::endl";
        return false;
    }

    try {
        std::ifstream f;
        char buf[1024];
        f.open(path, std::ios_base::in | std::ios_base::binary);
        if (!f) {
            std::cout << "loadSTL: (" << path << ") can not open" << std::endl;
            return false;
        }
        while (! f.eof()) {
            f.read(buf, 1024);
            auto size = f.gcount();
            // std::cout << "loadSTL: read size=" << size << std::endl;
            for (int i = 0; i < size; i++)
                data.push_back(buf[i]);
        }
        std::cout << "loadSTL: data size=" << data.size() << std::endl;
    } catch (const std::exception& e) {
        std::cout << "loadSTL: (" << path << ")" << e.what() << std::endl;
        return false;
    }
    return (data.size() > 0 ? true: false);
}

void geometry_binary_file(
    DhaibaConnect::PublisherInfo* pub, std::vector<char>& mesh_data, double r)
{
    if (mesh_data.size() <= 0) {
        std::cout << "geometry_binary_file: mesh_data is zero" << std::endl;
        return;
    }

    double d = 100.0 * sin(r);
    dhc::GeometryBinaryFile data;
    data.fileExtension() = "stl";
    data.fileData().data() = mesh_data;
    data.description() = "mesh(STL)";
    data.baseInfo().color().r() = 0;
    data.baseInfo().color().g() = 255;
    data.baseInfo().color().b() = 0;
    data.baseInfo().transform().value() = {
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                10+d, 20+d, 30+d, 0,
                };
    pub->write(&data);
    // std::cout << "geometry_binary_file: " << data.fileData().data().size() << std::endl;
}

void shape_box(DhaibaConnect::PublisherInfo* pub, double r)
{
    double d = 100.0 * sin(r);
    dhc::ShapeBox data;
    data.baseInfo().transform().value() = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        10, 20, 30, 0,
        };
    data.baseInfo().color().r() = 0;
    data.baseInfo().color().g() = 255;
    data.baseInfo().color().b() = 0;
    data.translation().value() = { 10+d, 20+d, 30+d };
    data.scaling().value() = { 200, 300, 400 };
    data.divisionCount().value() = { 3, 3, 3 };
    pub->write(&data);
    // std::cout << "shape_box: " << std::endl;
}

void geometry_state(DhaibaConnect::PublisherInfo* pub, double r)
{
    double d = 100.0 * sin(r);
    dhc::GeometryState data;
    data.transform().value() = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        d, d, d, 0
        };
    pub->write(&data);
}

