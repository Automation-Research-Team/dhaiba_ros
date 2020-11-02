#ifndef __TRIAL_COM_H__
#define __TRIAL_COM_H__

#include <string>
#include <map>
#include <iostream>
#include <fstream>

#include <DhaibaConnectN/Common.h>
#include <DhaibaConnectN/idl/TopicDataTypeCore.h>

bool loadSTL(const std::string& url, std::vector<char>& data);
void geometry_binary_file(
    DhaibaConnect::PublisherInfo* pub, std::vector<char>& mesh_data, double r);
void shape_box(DhaibaConnect::PublisherInfo* pub, double r);
void shape_sphere(DhaibaConnect::PublisherInfo* pub, double r);
void geometry_state(DhaibaConnect::PublisherInfo* pub, double r);

#endif  /* __TRIAL_COM_H__ */

