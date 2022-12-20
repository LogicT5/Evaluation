#pragma once
#ifndef __EVALUATION_H__
#define __EVALUATION_H__

#include <pcl/point_types.h>        // 点类型定义头文件
#include <pcl/registration/registration.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/vtk/pcl_vtk_compatibility.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h> //单独使用kd树的头文件（方法之一）
#include <Eigen/Dense>
#include<cmath>
#include <random>
#include "./MeshSample.h"
#include "./ReadFile.h"
#include <string>
#include <iostream>


class EvaluateModel
{
	public:
		std::string recon_mesh_path = ".\\dataset\\cb_res.ply";
		std::string gt_points_path = ".\\dataset\\Cassette.ply";
		int   MESH_SAMPLE_DENSITY = 1000;
		float filter_z = -9;
		float d = 0.03;



		EvaluateModel();
		~EvaluateModel();

		void ReadGtPoints(std::string gt_points_path);
		void ReadReconMesh(std::string recon_mesh_path);

		//void setInputReconCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud);
		//void setInputGtCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud);

		void ComputePointToPointRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
															  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
															  float* rmse);
		void ComputePointToPlaneRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
															  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
															  float* rmse);
		pcl::PointCloud<pcl::Normal>::Ptr ComputeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		void ComputeHausdorff(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
											   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
											   float* Hausdorffdist);
		void CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
										int blocksNum, 
										std::vector<pcl::PointCloud<pcl::PointXYZ>>* outcloud, 
										bool direction = true);
		void ComputePrecision(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
											  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
											  pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, 
											  float d, 
											  float* Precision);
		void ComputeRecall(pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud, 
										 pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
										 pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, 
										 float d,
										 float* Recall);
		void ComputeFscore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
										  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
										 float d, 
										 float* F_score, float* F);
		void CalculateAccuracy(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
											  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud);
		void ComputeAccuracy();
		void CalculateCompleteness(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
													 pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud);
		void ComputeCompleteness();
		void ComputeChamferDistance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
														  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
														  float* chamfer_distance);

		void SetEvluationMeshPath(std::string mesh_path)
		{
			recon_mesh_path = mesh_path;
		}

		void SetGroundTruthPath(std::string gt_path)
		{
			gt_points_path = gt_path;
		}

		pcl::PolygonMesh GetReconMesh()
		{
			return mesh;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetReconSamplePointCloud()
		{
			return recon_sample_points;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetGroundTruthPointCloud()
		{
			return gt_points;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetPrecisionErrorPointCloud()
		{
			return precision_error_point;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetRecallErrorPointCloud()
		{
			return recall_error_point;
		}

		void sf_output_evalution()
		{
			std::string sf_output_path = ".\\dataset\\sf_output_path\\";
			std::vector<std::string> sf_FileList;
			sf_FileList = GetFileList(".\\dataset\\sf_output_path\\*");
			FILE* fp;

			std::string output_filename = ".\\output\\sf_2_gt_" + get_date_time() + ".csv";
			fp = fopen(output_filename.c_str(), "w");
			fprintf(fp, "%s ,%s ,%s  ,%s ,%s ,%s ,%s ,%s ,%s ,%s \n",
				"Mesh", "PointToPointRMSE", "PointToPlaneRMSE", "Hausdorff ",
				"Precision", "Recall", "F_score", "Accuracy", "Completeness", "ChamferDistance");

			float* F;
			for (auto filename : sf_FileList)
			{
				F = new float[5];
				if (filename.find("_mesh.ply") != std::string::npos) //mesh文件
				{
					std::cout << sf_output_path + filename << std::endl;
					std::cout << "Read Mesh" << std::endl;
					ReadReconMesh(sf_output_path + filename);

					float* PointToPointRMSE = new float;
					ComputePointToPointRMSE(recon_sample_points, gt_points, PointToPointRMSE);
					std::cout << "PointToPointRMSE : " << *PointToPointRMSE << std::endl;
					float* PointToPlaneRMSE = new float;
					ComputePointToPlaneRMSE(recon_sample_points, gt_points, PointToPlaneRMSE);
					std::cout << "PointToPlaneRMSE : " << *PointToPlaneRMSE << std::endl;
					float* Hausdorff = new float;
					ComputeHausdorff(recon_sample_points, gt_points, Hausdorff);
					std::cout << "Hausdorff : " << *Hausdorff << std::endl;
					//CD  F-score
					CalculateAccuracy(recon_sample_points, gt_points);
					ComputeAccuracy();
					CalculateCompleteness(recon_sample_points, gt_points);
					ComputeCompleteness();
					float* chamfer_distance = new float();
					ComputeChamferDistance(recon_sample_points, gt_points, chamfer_distance);
					float* Fscore = new float;
					ComputeFscore(recon_sample_points, gt_points, d, Fscore, F);
					float Accuracy, Completeness;
					Accuracy = std::accumulate(accuracy.begin(), accuracy.end(), 0.f) / accuracy.size();
					Completeness = std::accumulate(completeness.begin(), completeness.end(), 0.f) / completeness.size();
					fprintf(fp, "%s ,%f,%f,%f,%f,%f,%f,%f,%f,%f \n",
						filename.c_str(), *PointToPointRMSE, *PointToPlaneRMSE, *Hausdorff,
						F[0], F[1], *Fscore, Accuracy, Completeness, *chamfer_distance);
				}
			}
			fprintf(fp, "%s ,%d,%s ,%f \n","MESH_SAMPLE_DENSITY", MESH_SAMPLE_DENSITY,"d",d);
			fclose(fp);
		}


		void mf_output_evalution()
		{
			std::string mf_output_path = ".\\dataset\\mf_output_path\\";
			std::vector<std::string> mf_FileList;
			mf_FileList = GetFileList(".\\dataset\\mf_output_path\\*");
			FILE* fp;

			std::string output_filename = ".\\output\\mf_2_gt_" + get_date_time() + ".csv";
			fp = fopen(output_filename.c_str(), "w");
			fprintf(fp, "%s ,%s ,%s  ,%s ,%s ,%s ,%s ,%s ,%s ,%s \n",
				"Mesh", "PointToPointRMSE", "PointToPlaneRMSE", "Hausdorff ",
				"Precision", "Recall", "F_score", "Accuracy", "Completeness", "ChamferDistance");

			float* F;
			for (auto filename : mf_FileList)
			{
				F = new float[5];
				//if (filename.compare("mf_Map_PCNormal.ply") == 0)
				//	continue;

				if (filename.find("_mesh.ply") != std::string::npos) //mesh文件
				{
					std::cout << mf_output_path + filename << std::endl;
					std::cout << "Read Mesh" << std::endl;
					ReadReconMesh(mf_output_path + filename);

					float* PointToPointRMSE = new float;
					ComputePointToPointRMSE(recon_sample_points, gt_points, PointToPointRMSE);
					std::cout << "PointToPointRMSE : " << *PointToPointRMSE << std::endl;
					float* PointToPlaneRMSE = new float;
					ComputePointToPlaneRMSE(recon_sample_points, gt_points,PointToPlaneRMSE);
					std::cout << "PointToPlaneRMSE : " << *PointToPlaneRMSE << std::endl;
					float* Hausdorff = new float;
					ComputeHausdorff(recon_sample_points, gt_points, Hausdorff);
					std::cout << "Hausdorff : " << *Hausdorff << std::endl;
					//CD  F-score
					CalculateAccuracy(recon_sample_points, gt_points);
					ComputeAccuracy();
					CalculateCompleteness(recon_sample_points, gt_points);
					ComputeCompleteness();
					float* chamfer_distance = new float();
					ComputeChamferDistance(recon_sample_points, gt_points, chamfer_distance);
					float* Fscore = new float;
					ComputeFscore(recon_sample_points, gt_points, d, Fscore, F);
					float Accuracy, Completeness;
					Accuracy = std::accumulate(accuracy.begin(), accuracy.end(), 0.f) / accuracy.size();
					Completeness = std::accumulate(completeness.begin(), completeness.end(), 0.f) / completeness.size();
					
					fprintf(fp, "%s ,%f,%f,%f,%f,%f,%f,%f,%f,%f \n",
						filename.c_str(), *PointToPointRMSE, *PointToPlaneRMSE, *Hausdorff,
						F[0], F[1], *Fscore, Accuracy, Completeness, *chamfer_distance);
				}
			}
			fprintf(fp, "%s ,%d,%s ,%f \n", "MESH_SAMPLE_DENSITY", MESH_SAMPLE_DENSITY, "d", d);
			fclose(fp);
			
		}

		void test()
		{
			std::cout << "start" << std::endl;


			std::cout << "Read Ground Truth" << std::endl;
			ReadGtPoints(gt_points_path);

			mf_output_evalution();
			sf_output_evalution();
			

			//std::string sf_output_path = ".\\dataset\\sf_output_path\\";
			//std::string mf_output_path = ".\\dataset\\mf_output_path\\";

			//std::vector<std::string> sf_FileList;
			//std::vector<std::string> mf_FileList;
			//sf_FileList = GetFileList(".\\dataset\\sf_output_path\\*");
			//mf_FileList = GetFileList(".\\dataset\\mf_output_path\\*");

			//FILE* fp;
			//fp = fopen(".\\output\\test.csv", "w");
			//fprintf(fp, "%s ,%s ,%s  ,%s ,%s ,%s ,%s ,%s ,%s ,%s \n",
			//	"Mesh", "ComputePointToPointRMSE :", "ComputePointToPlaneRMSE", "ComputeHausdorff ",
			//	"Precision", "Recall", "F_score", "Accuracy", "Completeness", "ChamferDistance");

			//float* F;
			//for (auto filename : sf_FileList)
			//{
			//	F = new float[50];
			//	if (filename.find("_mesh.ply") != std::string::npos) //mesh文件
			//	{
			//		std::cout << sf_output_path + filename << std::endl;
			//		std::cout << "Read Mesh" << std::endl;
			//		ReadReconMesh (sf_output_path + filename);

			//		float* ComputePointToPointrmse = new float;
			//		ComputePointToPointRMSE(recon_sample_points, gt_points, ComputePointToPointrmse);
			//		std::cout << "ComputePointToPointRMSE : " << *ComputePointToPointrmse<<std::endl;
			//		float* ComputePointToPlanermse = new float;
			//		ComputePointToPlaneRMSE(recon_sample_points, gt_points, ComputePointToPlanermse);
			//		std::cout << "ComputePointToPlaneRMSE : " << *ComputePointToPlanermse << std::endl;
			//		float* Hausdorff = new float;
			//		ComputeHausdorff(recon_sample_points, gt_points, Hausdorff);
			//		std::cout << "ComputeHausdorff : " << *Hausdorff << std::endl;
			//		//CD  F-score
			//		CalculateAccuracy(recon_sample_points, gt_points);
			//		ComputeAccuracy();
			//		CalculateCompleteness(recon_sample_points, gt_points);
			//		ComputeCompleteness();
			//		float* chamfer_distance = new float();
			//		ComputeChamferDistance(recon_sample_points, gt_points, chamfer_distance);
			//		float* Fscore = new float;
			//		ComputeFscore(recon_sample_points, gt_points, 1, Fscore,F);
			//		float Accuracy, Completeness;
			//		Accuracy = std::accumulate(accuracy.begin(), accuracy.end(), 0.f) / accuracy.size();
			//		Completeness = std::accumulate(completeness.begin(), completeness.end(), 0.f) / completeness.size();
			//		fprintf(fp, "%s ,%f ,%f  ,%f ,%f ,%f ,%f ,%f ,%f ,%f \n",
			//			filename, *ComputePointToPointrmse, *ComputePointToPlanermse, *Hausdorff,
			//			F[0], F[1], *Fscore, Accuracy, Completeness, *chamfer_distance);
			//	}
			//}
			//fclose(fp);
			
		}
	
	private:

		std::vector<double> accuracy;
		std::vector<double> completeness;
		pcl::PolygonMesh mesh;
		pcl::PointCloud<pcl::PointXYZ>::Ptr recon_sample_points;
		pcl::PointCloud<pcl::PointXYZ>::Ptr gt_points;

		pcl::search::KdTree<pcl::PointXYZ> recon_kdtree;
		pcl::search::KdTree<pcl::PointXYZ> gt_kdtree;

		pcl::PointCloud<pcl::PointXYZ>::Ptr precision_error_point;
		pcl::PointCloud<pcl::PointXYZ>::Ptr recall_error_point;
};


#endif // !__EVALUATION_H__
