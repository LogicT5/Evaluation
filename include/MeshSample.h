#ifndef MESHSAMPLE_H
#define MESHSAMPLE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkMassProperties.h>
#include <pcl/visualization/vtk/pcl_vtk_compatibility.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples, bool calc_normal, bool calc_color, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out);
void uniform_sampling(pcl::PolygonMesh mesh, std::size_t n_samples, bool calc_normal, bool calc_color, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out);
void uniform_sampling(pcl::PolygonMesh mesh, std::size_t n_samples, pcl::PointCloud<pcl::PointXYZ>& cloud_out);

#endif