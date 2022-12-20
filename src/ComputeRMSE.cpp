#include "../include/ComputeRMSE.h"
#include <pcl/common/distances.h>

void computePointToPointRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source, float* rmselist)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	cloudA = target;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	cloudB = source;

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);
	pcl::Correspondences all;
	//core.determineCorrespondences(all_correspondences,0.05);//确定输入点云与目标点云之间的对应关系：

	core.determineReciprocalCorrespondences(all);   //确定输入点云与目标点云之间的交互对应关系。
	float sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
	std::vector<float>Co;
	for (size_t j = 0; j < all.size(); j++) {
		sum += all[j].distance;
		Co.push_back(all[j].distance);
		sum_x += pow((cloudB->points[all[j].index_match].x - cloudA->points[all[j].index_query].x), 2);
		sum_y += pow((cloudB->points[all[j].index_match].y - cloudA->points[all[j].index_query].y), 2);
		sum_z += pow((cloudB->points[all[j].index_match].z - cloudA->points[all[j].index_query].z), 2);
	}
	rmse = sqrt(sum / all.size());     //均方根误差
	rmse_x = sqrt(sum_x / all.size()); //X方向均方根误差
	rmse_y = sqrt(sum_y / all.size()); //Y方向均方根误差
	rmse_z = sqrt(sum_z / all.size()); //Z方向均方根误差
	std::vector<float>::iterator max = max_element(Co.begin(), Co.end());//获取最大距离的对应点
	std::vector<float>::iterator min = min_element(Co.begin(), Co.end());//获取最小距离的对应点

	//std::cout << "匹配点对个数" << all.size() << std::endl;
	//std::cout << "距离最大值" << sqrt(*max) * 100 << "厘米" << std::endl;
	//std::cout << "距离最小值" << sqrt(*min) * 100 << "厘米" << std::endl;

	//std::cout << "均方根误差" << rmse << "米" << std::endl;
	//std::cout << "X均方根误差" << rmse_x << "米" << std::endl;
	//std::cout << "Y均方根误差" << rmse_y << "米" << std::endl;
	//std::cout << "Z均方根误差" << rmse_z << "米" << std::endl;

	rmselist[0] = all.size();
	rmselist[1] = sqrt(*max) * 100;
	rmselist[2] = sqrt(*min) * 100;
	rmselist[3] = rmse;
	rmselist[4] = rmse_x;
	rmselist[5] = rmse_y;
	rmselist[6] = rmse_z;
}

// 计算源点云中的点 到 与其在目标点云中最近邻点确定的平面 的距离 的rmse。注意:目标点云需要包含法向量
void computePointToPlaneRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, float *rmse)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals_target = computeNormal(target);
	float RMSE = 0.0;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(target);
	for (auto point_i : *source)
	{
		// 判断是否有无效点，如果有则跳过该点
		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
			continue;
		// K近邻搜索查找最近邻点
		pcl::Indices nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances))
			continue;
		std::size_t point_nn_i = nn_indices.front(); // front返回当前vector容器中起始元素的引用
		// 使用Map将点坐标和法向量转化为Eigen支持的数据结构
		Eigen::Vector3f normal_target = (*normals_target)[point_nn_i].getNormalVector3fMap(),
			point_source = point_i.getVector3fMap(),
			point_target = (*target)[point_nn_i].getVector3fMap();
		// 使用向量内积，计算点到面的距离
		float dist = normal_target.dot(point_source - point_target);
		RMSE += dist * dist;
	}
	*rmse = std::sqrt(RMSE / static_cast<float> (source->size()));

}


// 计算法向量
pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(target_cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	return normals;
}

void computeHausdorff(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b,float* Hausdorffdist)
{
	// 计算点集A到点集B的单向 Hausdorff 距离
	pcl::search::KdTree<pcl::PointXYZ> tree_b;
	tree_b.setInputCloud(cloud_b);
	float max_dist_a = -std::numeric_limits<float>::max();
	std::cout << "max_dist_a" << max_dist_a << " " << std::endl;
	for (const auto& point : (*cloud_a).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		tree_b.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] > max_dist_a)
			max_dist_a = sqr_distances[0];
	}
	max_dist_a = std::sqrt(max_dist_a);
	/*
	// 计算点集B到点集A的单向 Hausdorff 距离
	pcl::search::KdTree<pcl::PointXYZ> tree_a;
	tree_a.setInputCloud(cloud_a);
	float max_dist_b = -std::numeric_limits<float>::max();
	for (const auto& point : (*cloud_b).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		tree_a.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] > max_dist_b)
			max_dist_b = sqr_distances[0];
	}
	max_dist_b = std::sqrt(max_dist_b);
	// 计算点集AB的双向 Hausdorff 距离
	*Hausdorffdist = std::max(max_dist_a, max_dist_b);
	*/
	* Hausdorffdist = max_dist_a;
}


void CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,int blocksNum, std::vector<pcl::PointCloud<pcl::PointXYZ>> *outcloud,bool direction)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr block1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr block2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ cutpoint;
	int X = floor(cloud->points.size());
	/*
	pcl::PointXYZ minpoint, maxpoint, centroidpoint;
	//std::cout << "点云中最远距离  " << pcl::getMaxSegment(*cloud, minpoint, maxpoint) << std::endl;
	//std::cout << minpoint.x << " " << minpoint.y << " " << minpoint.z << " " << std::endl;
	//std::cout << maxpoint.x << " " << maxpoint.y << " " << maxpoint.z << " " << std::endl;
	centroidpoint.x = (minpoint.x + maxpoint.x) / 2;
	centroidpoint.y = (minpoint.y + maxpoint.y) / 2;
	centroidpoint.z = (minpoint.z + maxpoint.z) / 2;
	//*/
	///*
	Eigen::Vector4f centroid;
	pcl::PointXYZ centroidpoint;
	pcl::compute3DCentroid(*cloud, centroid);
	centroidpoint.x = centroid[0];
	centroidpoint.y = centroid[1];
	centroidpoint.z = centroid[2];
	//*/
	if (blocksNum > 2)
	{
		for (int i = 0; i < X; i++)
		{
			pcl::copyPoint(cloud->points[i], cutpoint);
			if ((cutpoint.x < centroidpoint.x) && direction)
			{
				block1->push_back(cutpoint);
			}
			else if ((cutpoint.x > centroidpoint.x) && direction)
			{
				block2->push_back(cutpoint);
			}
			else if ((cutpoint.y < centroidpoint.y) && !direction)
			{
				block1->push_back(cutpoint);
			}
			else  if ((cutpoint.y > centroidpoint.y) && !direction)
			{
				block2->push_back(cutpoint);
			}
		}
		CutPointCloud(block1, blocksNum / 2, outcloud,!direction);
		CutPointCloud(block2, blocksNum / 2, outcloud,!direction);
	}
	else if (blocksNum == 2)
	{
		for (int i = 0; i < X; i++)
		{
			pcl::copyPoint(cloud->points[i], cutpoint);
			if ((cutpoint.x < centroidpoint.x) && direction)
			{
				block1->push_back(cutpoint);
			}
			else if ((cutpoint.x > centroidpoint.x) && direction)
			{
				block2->push_back(cutpoint);
			}
			else if ((cutpoint.y < centroidpoint.y) && !direction)
			{
				block1->push_back(cutpoint);
			}
			else  if ((cutpoint.y > centroidpoint.y) && !direction)
			{
				block2->push_back(cutpoint);
			}
		}
		outcloud->push_back(*block1);
		outcloud->push_back(*block2);
		//std::cout << block1->points.size() << std::endl;
		//std::cout << block2->points.size() << std::endl;

	}
}

 //计算点云的倒角距离
void computeChamferDistance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, float* chamfer_distance)
{
	// cloud_a中每一个点在cloud_b中查找其最近邻点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_b;
	kdtree_b.setInputCloud(cloud_b);
	double sum_a = 0.0;
	std::vector<int> pointIdxKNNSearch(1);
	std::vector<float> pointKNNSquaredDistance(1);

	for (auto point_iter_a = (*cloud_a).begin(); point_iter_a != (*cloud_a).end(); ++point_iter_a)
	{
		pcl::PointXYZ point_a = *point_iter_a;
		if (kdtree_b.nearestKSearch(point_a, 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			sum_a += pointKNNSquaredDistance[0];
		}
	}

	// cloud_b中每一个点在cloud_a中查找其最近邻点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_a;
	kdtree_a.setInputCloud(cloud_a);
	double sum_b = 0.0;
	// 清空vector容器
	pointIdxKNNSearch.clear();
	pointKNNSquaredDistance.clear();

	for (auto point_iter_b = (*cloud_b).begin(); point_iter_b != (*cloud_b).end(); ++point_iter_b)
	{
		pcl::PointXYZ point_b = *point_iter_b;
		if (kdtree_a.nearestKSearch(point_b, 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			sum_b += pointKNNSquaredDistance[0];
		}
	}

	*chamfer_distance=(1.0 / (*cloud_a).size()) * sum_a + (1.0 / (*cloud_b).size()) * sum_b;// 计算倒角距离
}


void computePrecision(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr error_point,float d ,float* Precision)
{
	pcl::search::KdTree<pcl::PointXYZ> tree_b;
	tree_b.setInputCloud(cloud_b);
	int num = cloud_a->points.size();
	for (const auto& point : (*cloud_a).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		tree_b.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] >= d)
		{
			num--;
			error_point->push_back(point);
		}
	}
	std::cout << cloud_a->points.size()<< "小于"<<d<<"的点的个数" << num << " " << std::endl;

	*Precision = 100*(float)num / (float)cloud_a->points.size();
	std::cout << "Precision:  " << *Precision << " " << std::endl;
}


void computeRecall(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, float d, float* Recall)
{
	pcl::search::KdTree<pcl::PointXYZ> tree_b;
	tree_b.setInputCloud(cloud_b);
	int num = cloud_a->points.size();
	for (const auto& point : (*cloud_a).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		tree_b.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] >= d)
		{
			num--;
			error_point->push_back(point);
		}
	}
	std::cout << cloud_a->points.size() << "小于" << d << "的点的个数" << num << " " << std::endl;
	*Recall = 100*(float)num / (float)cloud_a->points.size();
	std::cout << "Recall:  " << *Recall << " " << std::endl;
}

void computeFscore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b, float d, float* F_score)
{
	float* Precision = new float;
	pcl::PointCloud<pcl::PointXYZ>::Ptr error_point(new pcl::PointCloud<pcl::PointXYZ>);
	computePrecision(cloud_a, cloud_b, error_point, d, Precision);
	float* Recall = new float;
	error_point->clear();
	computeRecall(cloud_b, cloud_a, error_point, d, Recall);

	*F_score = 2.0 * (*Precision) * (*Recall) / (*Precision + *Recall);
	std::cout << "F_score  " << *F_score << " " << std::endl;
}