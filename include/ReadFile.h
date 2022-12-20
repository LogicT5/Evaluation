#ifndef READFILE_H
#define READFILE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <direct.h>
#include <io.h>
#include <stdio.h> 
#include <ctime> 
#include <time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::vector<std::string> GetFileList(std::string path);
std::string get_date_time();

#endif // !READFILE_H
