#ifndef COMPUTERMSE_H
#define COMPUTERMSE_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  
#include <pcl/search/kdtree.h> //单独使用kd树的头文件
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <Eigen/Dense>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include<cmath>

void computePointToPointRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source, float* rmselist);
void computePointToPlaneRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, float* rmse);
pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud);
void computeHausdorff(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b, float* Hausdorffdist);
void CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int blocksNum, std::vector<pcl::PointCloud<pcl::PointXYZ>> *outcloud, bool direction = true);
void computePrecision(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, float d, float* Precision);
void computeRecall(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, float d, float* Recall);
void computeFscore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, float d, float* F_score);
#endif // !COMPUTERMSE_H

