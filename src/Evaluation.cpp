#include "../include/Evaluation.h"

EvaluateModel::EvaluateModel(): recon_sample_points(new pcl::PointCloud<pcl::PointXYZ>),
													gt_points(new pcl::PointCloud<pcl::PointXYZ>) ,
													precision_error_point(new pcl::PointCloud<pcl::PointXYZ>),
													recall_error_point(new pcl::PointCloud<pcl::PointXYZ>)
{
}
EvaluateModel::~EvaluateModel() {}

void
EvaluateModel::ReadGtPoints(std::string  gt_points_path)
{
	char temp[4];
	int lenx = gt_points_path.size() + 1;
	char *buf = new char[lenx];//是将sendstr中的内容拷贝到buf
	strcpy_s(buf,lenx, gt_points_path.c_str());
	int len = strlen(buf);
	char* p = buf + len - 1;
	while (*p != '.' && p != buf) p--;
	if (p == buf) temp[0] = '\0';
	else strcpy(temp, p + 1);
	if (strcmp(temp,"ply") == 0)
	{
		pcl::io::loadPLYFile(gt_points_path, *gt_points);
	}
	else if (strcmp(temp, "txt") == 0)
	{
		CreateCloudFromTxt(gt_points_path, gt_points);
	}
	gt_kdtree.setInputCloud(gt_points);
	std::cout << "Ground truth point cloud size "<< gt_points->points.size() <<std::endl;
}

void
EvaluateModel::ReadReconMesh(std::string  recon_mesh_path)
{
	pcl::io::loadPolygonFilePLY(recon_mesh_path, mesh);
	uniform_sampling(mesh, MESH_SAMPLE_DENSITY, *recon_sample_points);
	std::cout << "Sampling points on the mesh according to density " 
		          << MESH_SAMPLE_DENSITY<<"/m^2, result in "
				  << recon_sample_points->points.size() 
				  << " points" << std::endl;
	recon_kdtree.setInputCloud(recon_sample_points);
}


void 
EvaluateModel::ComputePointToPointRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
																	   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
																	   float* rmse)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(recon_cloud);
	core.setInputTarget(gt_cloud);
	pcl::Correspondences all;
	//core.determineCorrespondences(all_correspondences,0.05);//确定输入点云与目标点云之间的对应关系：

	core.determineReciprocalCorrespondences(all);   //确定输入点云与目标点云之间的交互对应关系。
	float sum = 0.0;
	//sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
	std::vector<float>Co;
	for (size_t j = 0; j < all.size(); j++) {
		sum += all[j].distance;
		//Co.push_back(all[j].distance);
		//sum_x += pow((gt_cloud->points[all[j].index_match].x - recon_cloud->points[all[j].index_query].x), 2);
		//sum_y += pow((gt_cloud->points[all[j].index_match].y - recon_cloud->points[all[j].index_query].y), 2);
		//sum_z += pow((gt_cloud->points[all[j].index_match].z - recon_cloud->points[all[j].index_query].z), 2);
	}
	*rmse = sqrt(sum / all.size());     //均方根误差
}

// 计算源点云中的点 到 与其在目标点云中最近邻点确定的平面 的距离 的rmse。注意:目标点云需要包含法向量
void 
EvaluateModel::ComputePointToPlaneRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
																	   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
																	   float* rmse)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals_target = ComputeNormal(gt_cloud);
	float RMSE = 0.0;

	for (auto point_i : *recon_cloud)
	{
		// 判断是否有无效点，如果有则跳过该点
		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
			continue;
		// K近邻搜索查找最近邻点
		pcl::Indices nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!gt_kdtree.nearestKSearch(point_i, 1, nn_indices, nn_distances))
			continue;
		std::size_t point_nn_i = nn_indices.front(); // front返回当前vector容器中起始元素的引用
		// 使用Map将点坐标和法向量转化为Eigen支持的数据结构
		Eigen::Vector3f normal_target = (*normals_target)[point_nn_i].getNormalVector3fMap(),
			point_source = point_i.getVector3fMap(),
			point_target = (*gt_cloud)[point_nn_i].getVector3fMap();
		// 使用向量内积，计算点到面的距离
		float dist = normal_target.dot(point_source - point_target);
		RMSE += dist * dist;
	}
	*rmse = std::sqrt(RMSE / static_cast<float> (recon_cloud->size()));

}


// 计算法向量
pcl::PointCloud<pcl::Normal>::Ptr 
EvaluateModel::ComputeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	return normals;
}

void 
EvaluateModel::ComputeHausdorff(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
														pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
														float* Hausdorffdist)
{
	// 计算点集A到点集B的单向 Hausdorff 距离
	float max_dist_a = -std::numeric_limits<float>::max();
	//std::cout << "max_dist_a" << max_dist_a << " " << std::endl;
	for (const auto& point : (*recon_cloud).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		gt_kdtree.nearestKSearch(point, 1, indices, sqr_distances);
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


void 
EvaluateModel::CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
												 int blocksNum, 
												 std::vector<pcl::PointCloud<pcl::PointXYZ>>* outcloud,
												 bool direction)
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
		CutPointCloud(block1, blocksNum / 2, outcloud, !direction);
		CutPointCloud(block2, blocksNum / 2, outcloud, !direction);
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

void 
EvaluateModel::ComputePrecision(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
													   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
													   pcl::PointCloud<pcl::PointXYZ>::Ptr error_point,
													   float d, 
													   float* Precision)
{
	int num = recon_cloud->points.size();
	for (const auto& point : (*recon_cloud).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		gt_kdtree.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] >= d)
		{
			num--;
			error_point->push_back(point);
		}
	}

	*Precision = 100 * (float)num / (float)recon_cloud->points.size();
	
}


void 
EvaluateModel::ComputeRecall(pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud, 
												  pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
												  pcl::PointCloud<pcl::PointXYZ>::Ptr error_point, 
												  float d, 
												  float* Recall)
{
	int num = gt_cloud->points.size();
	for (const auto& point : (*gt_cloud).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);

		recon_kdtree.nearestKSearch(point, 1, indices, sqr_distances);
		if (sqr_distances[0] >= d)
		{
			num--;
			error_point->push_back(point);
		}
	}
	//for(int i = 0 ;i < accuracy.size())
	

	*Recall = 100 * (float)num / (float)gt_cloud->points.size();
	
}

void 
EvaluateModel::ComputeFscore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud, 
												   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
												  float d, float* F_score,float *F)
{
	float* Precision = new float;
	ComputePrecision(recon_cloud, gt_cloud, precision_error_point, d, Precision);
	float* Recall = new float;
	ComputeRecall(gt_cloud, recon_cloud, recall_error_point, d, Recall);

	*F_score = 2.0 * (*Precision) * (*Recall) / (*Precision + *Recall);
	F[0] = *Precision;
	F[1] = *Recall;
	std::cout << "Precision:  " << *Precision << "  precision_error_point " << precision_error_point ->points.size() << " points " << std::endl;
	std::cout << "Recall:  " << *Recall << "  recall_error_point "<< recall_error_point->points.size() << " points " << std::endl;
	std::cout << "F_score  " << *F_score << " " << std::endl;
}

// 计算精度
void 
EvaluateModel::CalculateAccuracy(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
													   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud)
{
	for (const auto& point : (*recon_cloud).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);
		gt_kdtree.nearestKSearch(point, 1, indices, sqr_distances);
		accuracy.push_back(sqr_distances[0]);
	}
}

void
EvaluateModel::ComputeAccuracy()
{
	std::vector<double> sorted_accuracy(accuracy);
	std::sort(sorted_accuracy.begin(), sorted_accuracy.end());

	std::vector<double> accuracy_threshold{
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 50)],
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 60)],
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 70)],
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 80)],
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 90)],
	sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 95)]
	};

	std::cout
		<< "50% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.5), 0.f) / (int)(sorted_accuracy.size() * 0.5)
		<< std::endl
		<< "60% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.6), 0.f) / (int)(sorted_accuracy.size() * 0.6)
		<< std::endl
		<< "70% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.7), 0.f) / (int)(sorted_accuracy.size() * 0.7)
		<< std::endl
		<< "80% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.8), 0.f) / (int)(sorted_accuracy.size() * 0.8)
		<< std::endl
		<< "90% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.9), 0.f) / (int)(sorted_accuracy.size() * 0.9)
		<< std::endl
		<< "95% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.95), 0.f) / (int)(sorted_accuracy.size() * 0.95)
		<< std::endl
		<< "100% Average Accuracy: "
		<< std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()), 0.f) / (int)(sorted_accuracy.size())
		<< std::endl;

	std::vector<double> accuracy_range_threshold{ 0.005, 0.01, 0.02,
																				0.03,  0.05, 0.1,
																				0.2,   0.5,  1 };
	std::vector<double> accuracy_range(accuracy_range_threshold.size(), 0);
	for (int i_point = 0; i_point < accuracy.size(); i_point++) {
		for (int i_threshold = 0;
			i_threshold < accuracy_range_threshold.size();
			++i_threshold) {
			if (accuracy_range[i_threshold] == 0 && sorted_accuracy[i_point] > accuracy_range_threshold[i_threshold])
				accuracy_range[i_threshold]
				= (double)i_point / accuracy.size() * 100;
		}
	}

	for (int i_threshold = 0; i_threshold < accuracy_range_threshold.size(); ++i_threshold)
		std::cout << accuracy_range[i_threshold]
		<< "% of points has error lower than "
		<< accuracy_range_threshold[i_threshold]
		<< std::endl;

}

//计算完整性
void
EvaluateModel::CalculateCompleteness(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
															  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud)
{
	for (const auto& point : (*gt_cloud).points)
	{
		pcl::Indices indices(1);
		std::vector<float> sqr_distances(1);
		recon_kdtree.nearestKSearch(point, 1, indices, sqr_distances);
		completeness.push_back(sqr_distances[0]);
	}
}

void
EvaluateModel::ComputeCompleteness()
{
	std::vector<double> sorted_completeness(completeness);
	std::sort(sorted_completeness.begin(), sorted_completeness.end());

	std::vector<double> completeness_threshold{
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 50)],
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 60)],
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 70)],
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 80)],
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 90)],
		sorted_completeness[(int)(sorted_completeness.size() / 100 * 95)]
	};


	std::cout
		<< "50% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.5), 0.f) / (int)(sorted_completeness.size() * 0.5)
		<< std::endl
		<< "60% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.6), 0.f) / (int)(sorted_completeness.size() * 0.6)
		<< std::endl
		<< "70% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.7), 0.f) / (int)(sorted_completeness.size() * 0.7)
		<< std::endl
		<< "80% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.8), 0.f) / (int)(sorted_completeness.size() * 0.8)
		<< std::endl
		<< "90% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.9), 0.f) / (int)(sorted_completeness.size() * 0.9)
		<< std::endl
		<< "95% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() * 0.95), 0.f) / (int)(sorted_completeness.size() * 0.95)
		<< std::endl
		<< "100% Average Completeness: "
		<< std::accumulate(sorted_completeness.begin(), sorted_completeness.begin() + (int)(sorted_completeness.size() ), 0.f) / (int)(sorted_completeness.size() )
		<< std::endl;

	std::vector<double> completeness_range_threshold{ 0.005, 0.01, 0.02,
												  0.03,  0.05, 0.1,
												  0.2,   0.5,  1 };
	std::vector<double> completeness_range(
		completeness_range_threshold.size(), 0
	);
	for (int i_point = 0; i_point < completeness.size(); i_point++) {
		for (int i_threshold = 0;
			i_threshold < completeness_range_threshold.size();
			++i_threshold) {
			if (completeness_range[i_threshold] == 0 && sorted_completeness[i_point] > completeness_range_threshold[i_threshold])
				completeness_range[i_threshold]
				= (double)i_point / completeness.size() * 100;
		}
	}

	for (int i_threshold = 0;i_threshold < completeness_range_threshold.size();++i_threshold)
		std::cout << completeness_range[i_threshold]
		<< "% of points has error lower than "
		<< completeness_range_threshold[i_threshold]
		<< std::endl;
}

//计算点云的倒角距离
void
EvaluateModel::ComputeChamferDistance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr recon_cloud,
																   pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_cloud,
																   float* chamfer_distance)
{
	//// cloud_a中每一个点在cloud_b中查找其最近邻点
	//double sum_a = 0.0;
	//std::vector<int> pointIdxKNNSearch(1);
	//std::vector<float> pointKNNSquaredDistance(1);

	//for (auto point_iter_a = (*recon_cloud).begin(); point_iter_a != (*recon_cloud).end(); ++point_iter_a)
	//{
	//	pcl::PointXYZ point_a = *point_iter_a;
	//	if (gt_kdtree.nearestKSearch(point_a, 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	//	{
	//		sum_a += pointKNNSquaredDistance[0];
	//	}
	//}

	//// cloud_b中每一个点在cloud_a中查找其最近邻点
	//double sum_b = 0.0;
	//// 清空vector容器
	//pointIdxKNNSearch.clear();
	//pointKNNSquaredDistance.clear();

	//for (auto point_iter_b = (*gt_cloud).begin(); point_iter_b != (*gt_cloud).end(); ++point_iter_b)
	//{
	//	pcl::PointXYZ point_b = *point_iter_b;
	//	if (recon_kdtree.nearestKSearch(point_b, 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	//	{
	//		sum_b += pointKNNSquaredDistance[0];
	//	}
	//}

	//*chamfer_distance = (1.0 / (*recon_cloud).size()) * sum_a + (1.0 / (*gt_cloud).size()) * sum_b;// 计算倒角距离
	* chamfer_distance = std::accumulate(accuracy.begin(), accuracy.end(), 0.f) / accuracy.size()
		+ std::accumulate(completeness.begin(), completeness.end(), 0.f) / completeness.size();
	std::cout
		<< "Chamfer distance： "
		<< *chamfer_distance
	<< std::endl;
}
