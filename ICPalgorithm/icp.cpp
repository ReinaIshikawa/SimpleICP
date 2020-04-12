

#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>

#define NOMINMAX
#include <iostream>
#include <Windows.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/grabber.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#define FILENUM 5


/*
reference
how to use pcl
https://www.slideshare.net/masafuminoda/pcl-11030703
*/



////�_�Q���t�@�C������ǂݍ���ŉ�ʂɕ\��
//void read_file_and_show(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data) {
//	//pcd �t�@�C������ǂݍ���
//	pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::io::loadPCDFile(filename, *data);
//
//	//CloudViewer�ł̓_�Q�f�[�^�̕\��
//	pcl::visualization::CloudViewer *viewer = new pcl::visualization::CloudViewer(filename);
//	viewer->showCloud(cloud_data);
//	while (!viewer->wasStopped()) {
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}
//
////PCLVisualizer�ł̓_�Q�f�[�^�̕\��
//void visualize_with_viewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_data) {
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3Dviewer"));
//	int v1(0);
//	viewer->initCameraParameters();
//	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->addCoordinateSystem(1.0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_data, 255, 255, 255);
//	viewer->addPointCloud(cloud_data, single_color, "sample cloud 1", v1);
//	//viewer->showCloud(cloud_data, "cloud");
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce();
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}

//�e�X�g�p�_�Q�쐬
void create_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

		for (int ix = 0; ix < 10; ix++) {
			for (int iy = 0; iy < 2; iy++) {
				for (int iz = 0; iz < 10; iz++) {
					pcl::PointXYZ point;
					point.x = ix * 0.1;
					point.y = iy * 0.1;
					point.z = iz * 0.1;
					cloud->points.push_back(point);
				}
			}
		}
	
	//cloud->width = 0.5;
	//cloud->height = 0.5;
}

////�����l�̏���
//void remove_NaN_of_input(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output) {
//	pcl::PassThrough<pcl::PointXYZRGB> pass;
//	pass.setInputCloud(input);
//	pass.filter(* output);
//}
//
////���ʂ̏���
//void plain_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	seg.setInputCloud(input);
//	seg.setOptimizeCoefficients(true);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.02);
//
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//	seg.segment(*inliers, *coefficients);
//
//	//�_�Q�̒��o�i�����j
//	pcl::ExtractIndices < pcl::PointXYZ> extract;
//	extract.setInputCloud(input);
//	extract.setIndices(inliers);
//	extract.setNegative(true);
//	extract.filter(*output);
//}


//�@���x�N�g���̐���
void predict_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, std::vector < Eigen::Vector3f > & normal_vecs) {
	float searchRadius = 0.5;
	std::vector<int> pointIndices;
	std::vector<float> pointDistances;

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		kdtree.radiusSearch(cloud->points[i], searchRadius, pointIndices, pointDistances);

		//�s��M
		Eigen::Matrix<float, Eigen::Dynamic, 3> M;
		M.resize(pointIndices.size(), 3);
		for (size_t k = 0; k < pointIndices.size(); k++)
		{
			M(k, 0) = cloud->points[pointIndices[k]].x - cloud->points[i].x;
			M(k, 1) = cloud->points[pointIndices[k]].y - cloud->points[i].y;
			M(k, 2) = cloud->points[pointIndices[k]].z - cloud->points[i].z;
		}
		//�ŗL�l���\���o��p���ČŗL�l�ƌŗL�x�N�g�����v�Z
		Eigen::EigenSolver<Eigen::Matrix<float, 3, 3> > solver(M.transpose()*M);
		//�ŏ��̌ŗL�l��T��
		float min = FLT_MAX;
		size_t min_index;
		for(size_t k = 0; k < solver.eigenvalues().size(); k++)
		{
			if (solver.eigenvalues()(k).real() < min) {
				min = solver.eigenvalues()(k).real();
				min_index = k;
			}
		}
		//�@���x�N�g����vector�Ɋi�[�@index��cloud�Ɠ���
		normal_vecs.push_back(solver.eigenvectors().col(min_index).real());
	}
}


//�ŋߖT�T��
/*sourceCloud��[index]�ɑ΂���distCloud�̍ŋߖT��index��Ԃ�*/
int search_kdtree_clouds(int kNum, int index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {

	float searchRadius = 0.5;
	std::vector<int> pointIndices;
	std::vector<float> pointDistances;

	pcl::PointXYZ searchPoint = cloud->points[index];
	if (kdtree.nearestKSearch(searchPoint, kNum, pointIndices, pointDistances) > 0) {
		//output as test
		for (std::size_t i = 0;i < pointIndices.size(); ++i)
		{
			std::cout << i << "point Indices: " << pointIndices[i] << std::endl;
		}
		return pointIndices[0];
	}
	return -1;

}

void create_translation_mat(float tx, float ty, float tz, Eigen::Matrix4f& Mopt_T) {
	Mopt_T = Eigen::Matrix4f::Identity();
	/*
		|1 0 0 t_x|
	T=	|0 1 0 t_y|
		|0 0 1 t_z|
		|0 0 0  1 |
	*/
	Mopt_T(0, 3) = tx;//t_x
	Mopt_T(1, 3) = ty;//t_y
	Mopt_T(2, 3) = tz;//t_z
}

void create_rotation_mat(float rx, float ry, float rz, Eigen::Matrix4f& Mopt_R) {
	Mopt_R = Eigen::Matrix4f::Identity();
	/*
		|r00 r01 r02 0|
	R=	|r10 r11 r12 0|
		|r20 r21 r22 0|
		| 0   0   0  1|
	*/
	Mopt_R(0, 0) = std::cos(rz) * std::cos(ry);
	Mopt_R(0, 1) = -std::sin(rz) * std::cos(rx) + std::cos(rz) * std::sin(ry) * std::sin(rx);
	Mopt_R(0, 2) = std::sin(rz) * std::sin(rx) + std::cos(rz) * std::sin(ry) * std::cos(rx);
	Mopt_R(1, 0) = std::sin(rz) * std::cos(ry);
	Mopt_R(1, 1) = std::cos(rz) * std::cos(rx) + std::sin(rz) * std::sin(ry) * std::sin(rx);
	Mopt_R(1, 2) = -std::cos(rz) * std::sin(rx) + std::sin(rz) * std::sin(ry) * std::cos(rx);
	Mopt_R(2, 0) = -std::sin(ry);
	Mopt_R(2, 1) = std::cos(ry) * std::sin(rx);
	Mopt_R(2, 2) = std::cos(ry) * std::cos(rx);
}

//��̍��W�n������
//�œK����transformedCloud�ɔ��f
void fusing_two_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr preCloud) {
	//create a kdtree of preCloud
	pcl::KdTreeFLANN<pcl::PointXYZ> prekdtree;
	
	for (int i = 0;i < preCloud->size();i++)
		std::cout << preCloud->points[i].x << "," << preCloud->points[i].y << "," << preCloud->points[i].z << "," << std::endl;

	prekdtree.setInputCloud(preCloud);
	//create a kdtree of sourceCloud
	pcl::KdTreeFLANN<pcl::PointXYZ> currentkdtree;
	currentkdtree.setInputCloud(currentCloud);
	
	//sourceCloud��cloud�̖@���x�N�g�������Ƃ߂� index��sourceCloud�Ɠ���
	std::vector<Eigen::Vector3f> normal_vecs;
	predict_normal(preCloud, prekdtree, normal_vecs);

	//�œK�������߂邽�߂�matrix Ax-b
	//Eigen::MatrixXd matrix_A(100, 6);
	Eigen::MatrixXf matrix_A = Eigen::MatrixXf::Zero(currentCloud->points.size(), 6);
	Eigen::MatrixXf matrix_b = Eigen::MatrixXf::Zero(currentCloud->points.size(), 1);
	Eigen::MatrixXf matrix_x = Eigen::MatrixXf::Zero(currentCloud->points.size(), 1);
	//Eigen::Matrix<float, Eigen::Dynamic, 6> matrix_A;
	//Eigen::Matrix<float, Eigen::Dynamic, 1> matrix_b;
	//Eigen::Matrix<float, Eigen::Dynamic, 1> matrix_x;//x = (��, ��, ��, t_x, t_y, t_z)^T

	for (std::size_t i = 0; i < currentCloud->points.size(); ++i)
	{
		//sourceCloud��[index(i)]�ɑ΂���distCloud(kdtree)�̍ŋߖT��index��Ԃ�
		//�Ƃ肠����knn k=1
		int nearest = search_kdtree_clouds(1, i, currentCloud, prekdtree);

		if (nearest < 0) { std::cout << "no nearest " << std::endl; }
		matrix_A(i, 0) = normal_vecs[nearest].z() * currentCloud->points[i].y - normal_vecs[nearest].y() * currentCloud->points[i].z;
		matrix_A(i, 1) = normal_vecs[nearest].x() * currentCloud->points[i].z - normal_vecs[nearest].z() * currentCloud->points[i].x;
		matrix_A(i, 2) = normal_vecs[nearest].y() * currentCloud->points[i].x - normal_vecs[nearest].x() * currentCloud->points[i].y;
		matrix_A(i, 3) = normal_vecs[nearest].x();
		matrix_A(i, 4) = normal_vecs[nearest].y();
		matrix_A(i, 5) = normal_vecs[nearest].z();

		matrix_b(i, 0) = normal_vecs[nearest].x() * preCloud->points[i].x
			+ normal_vecs[nearest].y() * preCloud->points[i].y
			+ normal_vecs[nearest].z() * preCloud->points[i].z
			- normal_vecs[nearest].x() * currentCloud->points[i].x
			- normal_vecs[nearest].y() * currentCloud->points[i].y
			- normal_vecs[nearest].z() * currentCloud->points[i].z;
	}
	//�œK��
	matrix_x = (matrix_A.transpose() * matrix_A).inverse() * matrix_A.transpose() * matrix_b;
	std::cout << "matrix_x =(" << matrix_x(0, 0) << ", " << matrix_x(1, 0) << ", " << matrix_x(2, 0) << ", " 
		<< matrix_x(3, 0) << ", " << matrix_x(4, 0) << ", " << matrix_x(5, 0) << std::endl;
	//�P�ʍs��ɏ�����
	Eigen::Matrix4f Mopt = Eigen::Matrix4f::Identity();
	
	//a 3D rigidbody transform M = T(t_x, t_y, t_z)�ER(��, ��, ��)
	Eigen::Matrix4f Mopt_T;
	Eigen::Matrix4f Mopt_R;

	create_translation_mat(matrix_x(3, 0), matrix_x(4, 0), matrix_x(5, 0), Mopt_T);
	create_rotation_mat(matrix_x(0, 0), matrix_x(1, 0), matrix_x(2, 0), Mopt_R);
	
	Mopt = Mopt_T * Mopt_R;

	//��]
	pcl::transformPointCloud(*currentCloud, *transformedCloud, Mopt);

	
}

int main() {
	try
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr preCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

		create_cloud(preCloud);

		//�����p��currentCloud��preCloud����]�E���s�ړ����������̂ɂ���
		//a 3D rigidbody transform M = T(t_x, t_y, t_z)�ER(��, ��, ��)
		Eigen::Matrix4f Mopt_T;
		Eigen::Matrix4f Mopt_R;

		create_translation_mat(0, 0, 0.2, Mopt_T);
		create_rotation_mat(0, M_PI / 9, 0, Mopt_R);
		pcl::transformPointCloud(*preCloud, *currentCloud, Mopt_T*Mopt_R);

		//fusion
		fusing_two_clouds(transformedCloud, currentCloud, preCloud);
		//fusing_two_clouds(transformedCloud, transformedCloud, preCloud);

		//visualization
		pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
		// Define R,G,B colors for the point cloud
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pre_cloud_color_handler(preCloud, 255, 255, 255);
		viewer.addPointCloud(preCloud, pre_cloud_color_handler, "original_cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> current_cloud_color_handler(currentCloud, 20, 20, 230);
		viewer.addPointCloud(currentCloud, current_cloud_color_handler, "current_cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformedCloud, 230, 20, 20);
		viewer.addPointCloud(transformedCloud, transformed_cloud_color_handler, "transformed_cloud");

		viewer.addCoordinateSystem(1.0, "cloud", 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "current_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
		//viewer.setPosition(800, 400); // Setting visualiser window position

		while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}

		return 0;
	}
	catch (std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
}