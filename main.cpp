#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>        // 点类型定义头文件
#include <pcl/registration/icp.h>   // ICP配准类相关的头文件
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
#include "include/MeshSample.h"
#include "include/ReadFile.h"
#include "include/ComputeRMSE.h"
#include "include/Evaluation.h"
//char mesh_path[256] = ".\\dataset\\mesh.ply";
//char SingleFrame_path[256] = ".\\dataset\\cloudswithnormal.ply";
//char sampleClouds_path[256] = ".\\dataset\\MeshSampled.ply";
//char scene_path[256] = ".\\dataset\\scene1.ply";


//int SAMPLE_POINTS = 50000;
//
int viewer(pcl::PointCloud<pcl::PointXYZRGBNormal> ::Ptr cloud);
int viewer(pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud);
void viewer(pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_b);
void viewer(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ> ::Ptr error_cloud);

int main()
{
	std::string mesh_path = ".\\dataset\\sf_output_path\\sf_0001_mesh.ply";
	std::string gt_path = ".\\dataset\\mf_output_path\\mf_Map_PCNormal.ply";
	EvaluateModel EM;
	
	EM.SetEvluationMeshPath(mesh_path);

	EM.SetGroundTruthPath(gt_path);
	EM.test();
	//viewer(EM.GetReconMesh(), EM.GetRecallErrorPointCloud());
	//viewer(EM.GetReconMesh(), EM.GetReconSamplePointCloud()); // 可视化采样点云

	return 0;
}


/*
int main()
{
	pcl::PolygonMesh MeshModel;
	pcl::PointCloud<pcl::PointXYZ>::Ptr SampleClouds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr SingleFrame(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Scene(new pcl::PointCloud<pcl::PointXYZ>);

	float * PointToPointRMSEList;
	PointToPointRMSEList = new float[7];
	float *PointToPlaneRMSE = new float;
	*PointToPlaneRMSE = 0.0;
	float* Hausdorffdist = new float;
	*Hausdorffdist = 0.0;
	float SHausdorffdist = 0.0;

	///*
	//加载模型和点云
	pcl::io::loadPolygonFilePLY(mesh_path, MeshModel);
	pcl::io::loadPLYFile(SingleFrame_path,*SingleFrame);
	//pcl::io::loadPLYFile(scene_path, *Scene);
	CreateCloudFromTxt(scene_path,Scene);
	//pcl::io::savePLYFileASCII(".\\dataset\\scene1.ply", *Scene);

	uniform_sampling(MeshModel, 3000, *SampleClouds);
	std::cout << "采样完成" << std::endl;

	std::vector<pcl::PointCloud<pcl::PointXYZ> > outcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	std::default_random_engine e;
	std::uniform_real_distribution<double> random(0, 1);

	float* p2pointRMSEList;
	p2pointRMSEList = new float[50];
	float* p2planeRMSEList;
	p2planeRMSEList = new float[50];
	float* averageHausdorffdistlist;
	averageHausdorffdistlist = new float[50];
	int poinszlist[50];


	int SAMPLE_POINTS = SingleFrame->points.size();
	for (int i = 0;i<5;i ++)
	{
		SHausdorffdist = 0.0;
		uniform_sampling(MeshModel, SAMPLE_POINTS, *SampleClouds);
		std::cout << "采样结束" << i << std::endl;
		computePointToPointRMSE(SampleClouds, Scene, PointToPointRMSEList);
		std::cout << "SampleClouds points_sz: " << SampleClouds->points.size() << "      Scene points_sz: " << Scene->points.size() << std::endl;
		std::cout << "点到点距离的均方根误差RMSE为：" << PointToPointRMSEList[3] << std::endl;
		computePointToPlaneRMSE(SampleClouds, Scene, PointToPlaneRMSE);
		std::cout << "点到面距离的均方根误差RMSE为：" << *PointToPlaneRMSE << std::endl;
		computeHausdorff(SampleClouds, Scene, Hausdorffdist);
		std::cout << "SampleClouds, Scene Hausdorffdist为：" << *Hausdorffdist << std::endl;
		CutPointCloud(SampleClouds, 8, &outcloud);
		for (int s = 0; s < 8; s++)
		{
			std::cout << s << " :" << outcloud[s].points.size() <<":";
			*cloud = outcloud[s];
			computeHausdorff(cloud, Scene, Hausdorffdist);
			SHausdorffdist += *Hausdorffdist;
			std::cout << "Hausdorffdist为：" << *Hausdorffdist << std::endl;
		}
		outcloud.clear();
		std::cout << " average Hausdorffdist为：" << SHausdorffdist / 8 << std::endl;

		p2pointRMSEList[i] = PointToPointRMSEList[3];
		p2planeRMSEList[i] = *PointToPlaneRMSE;
		averageHausdorffdistlist[i] = SHausdorffdist / 8;
		poinszlist[i] = SAMPLE_POINTS;
		SAMPLE_POINTS += 10000;
	}

	FILE* fp;
	fp = fopen(".\\output\\demo50.csv", "w");
	for (int i = 0; i < 50; i++) {
		fprintf(fp, "%d,%f,%f ,%f\n",  // \bred{!!!注意 \%s 后有一个空格}
			poinszlist[i],p2pointRMSEList[i], p2planeRMSEList[i], averageHausdorffdistlist[i]);
		fprintf(fp, "%d,%d,%d ,%d\n", 1, 2, 3,4);
	}
	fclose(fp);
}
*/

//int main()
//{
//	pcl::PolygonMesh MeshModel;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr SampleClouds(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr SingleFrame(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr Scene(new pcl::PointCloud<pcl::PointXYZ>);
//
//	float* PointToPointRMSEList;
//	PointToPointRMSEList = new float[7];
//	float* PointToPlaneRMSE = new float;
//	*PointToPlaneRMSE = 0.0;
//	float* Hausdorffdist = new float;
//	*Hausdorffdist = 0.0;
//	float SHausdorffdist = 0.0;
//
//	///*
//	//加载模型和点云
//	pcl::io::loadPolygonFilePLY(mesh_path, MeshModel);
//	//pcl::io::loadPLYFile(SingleFrame_path, *SingleFrame);
//	pcl::io::loadPLYFile(scene_path, *Scene);
//	//CreateCloudFromTxt(scene_path, Scene);
//	//pcl::io::savePLYFileASCII(".\\dataset\\scene1.ply", *Scene);
//
//	uniform_sampling(MeshModel, SAMPLE_POINTS, *SampleClouds);
//	std::cout << "采样完成" << SampleClouds->points.size() << std::endl;
//	//viewer(Scene);
//
//	//std::vector<pcl::PointCloud<pcl::PointXYZ> > outcloud;
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	//std::default_random_engine e;
//	//std::uniform_real_distribution<double> random(0, 1);
//	/*
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	for (int s = 0; s < 8; s++)
//	{
//		std::cout << s <<" :" << outcloud[s].points.size() << std::endl;
//		*cloud = outcloud[s];
//		computeHausdorff(cloud, Scene, Hausdorffdist);
//		SHausdorffdist += *Hausdorffdist;
//		std::cout << "             Hausdorffdist为：" << *Hausdorffdist << std::endl;
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, int(random(e) * 255) , int(random(e) * 255), int(random(e) * 255)); // green
//		viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud" + std::to_string(s));
//	}
//	std::cout << " average Hausdorffdist为：" << SHausdorffdist/8 << std::endl;
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "MidPintCloud");
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	//*/
//
//	//int SAMPLE_POINTS = SingleFrame->points.size();
//	//uniform_sampling(MeshModel, SAMPLE_POINTS, *SampleClouds);
//
//	float* Precision = new float;
//	float d = 0.03;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr error_point(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr error_point1(new pcl::PointCloud<pcl::PointXYZ>);
//	computePrecision(SampleClouds, Scene, error_point, d, Precision);
//	viewer(MeshModel, error_point);
//	float* Recall = new float;
//	error_point->clear();
//	computeRecall(Scene, SampleClouds, error_point1, d, Recall);
//	float* Fscore = new float;
//	computeFscore(SampleClouds, Scene, d, Fscore);
//
//	viewer(MeshModel, error_point1);
//
//	//computePointToPointRMSE(SampleClouds, Scene, PointToPointRMSEList);
//	//std::cout << "SampleClouds points_sz: " << SampleClouds->points.size() << "      Scene points_sz: " << Scene->points.size() << std::endl;
//	//std::cout << "点到点距离的均方根误差RMSE为：" << PointToPointRMSEList[3] << std::endl;
//	//computePointToPlaneRMSE(SampleClouds, Scene, PointToPlaneRMSE);
//	//std::cout << "点到面距离的均方根误差RMSE为：" << *PointToPlaneRMSE << std::endl;
//	//computeHausdorff(SampleClouds, Scene, Hausdorffdist);
//	//std::cout << "SampleClouds, Scene Hausdorffdist为：" << *Hausdorffdist << std::endl;
//	//CutPointCloud(SampleClouds, 8, &outcloud);
//	//for (int s = 0; s < 8; s++)
//	//{
//	//	std::cout << s << " :" << outcloud[s].points.size() << ":";
//	//	*cloud = outcloud[s];
//	//	computeHausdorff(cloud, Scene, Hausdorffdist);
//	//	SHausdorffdist += *Hausdorffdist;
//	//	std::cout << "Hausdorffdist为：" << *Hausdorffdist << std::endl;
//	//}
//	//std::cout << " average Hausdorffdist为：" << SHausdorffdist / 8 << std::endl;
//
//}

int viewer(pcl::PointCloud<pcl::PointXYZRGBNormal> ::Ptr cloud) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloud, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, single_color, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

int viewer(pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

void viewer(pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_b)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_a(cloud_a, 0, 255, 0); // green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_b(cloud_b, 255, 0, 0); // green

	viewer->addPointCloud<pcl::PointXYZ>(cloud_a, single_color_a, "sample clouda");
	viewer->addPointCloud<pcl::PointXYZ>(cloud_b, single_color_b, "sample cloudb");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void viewer(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ> ::Ptr error_cloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> error_cloud_color(error_cloud, 255, 0, 0); // green

	viewer->addPolygonMesh(mesh, "mesh");
	viewer->addPointCloud<pcl::PointXYZ>(error_cloud, error_cloud_color, "sample cloudb");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
