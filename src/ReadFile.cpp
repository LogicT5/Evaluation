#include "../include/ReadFile.h"

void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream file(file_path.c_str());//c_str()：生成一个const char*指针，指向以空字符终止的数组。
	std::string line;
	pcl::PointXYZ point;
	int i = 0;
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		cloud->push_back(point);
		printf("\rinput point : %d  ", i++); // \r的神奇作用：下次打印仍从行首开始
	}
	std::cout<<std::endl;
	file.close();
}

std::vector<std::string> GetFileList(std::string path)
{
	std::vector<std::string> FileList;
	int lenx = path.size() + 1;
	char* buf = new char[lenx];//是将sendstr中的内容拷贝到buf
	strcpy_s(buf, lenx, path.c_str());

	_finddata64i32_t fileInfo;
	intptr_t hFile = _findfirst(buf, &fileInfo);

	if (hFile == -1) {
		return FileList;
	}
	do
	{
		FileList.push_back(fileInfo.name);
		//std::cout << fileInfo.name << std::endl;

	} while (_findnext(hFile, &fileInfo) == 0);
	return FileList;
}

std::string get_date_time()
{
	time_t t = time(0);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H_%M_%S", localtime(&t));
	return tmp;
}