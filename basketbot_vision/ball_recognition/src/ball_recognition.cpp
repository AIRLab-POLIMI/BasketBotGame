#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
class BallRecognition
{
	typedef pcl::PointXYZRGB pointType;
	ros::NodeHandle node;
	ros::Subscriber pclSubscriber;
	void pclCallback(const sensor_msgs::PointCloud2::ConstPtr&);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer();

	pcl::PointCloud<pointType>::Ptr cloud;
	pcl::PointCloud<pointType>::Ptr cloudTmp;

	float hmin, hmax,smin,smax;

	void filterVoxelGrid();
	void filterByColor();
	void updateFilterValues();
	bool findSphere(pcl::PointIndices::Ptr,pcl::ModelCoefficients::Ptr);

public:
	void spin();
	BallRecognition();
};
boost::shared_ptr<pcl::visualization::PCLVisualizer> BallRecognition::createViewer()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);

	viewer->initCameraParameters ();
	return (viewer);

}
BallRecognition::BallRecognition()
{
	float colore = 37.6;


	hmin=colore-5.0, hmax=colore+5.0,smin=0.0,smax=2.0;

	pclSubscriber = node.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &BallRecognition::pclCallback,this);
	viewer = createViewer();
	cloud= pcl::PointCloud<pointType>::Ptr(new pcl::PointCloud<pointType>());
	cloudTmp= pcl::PointCloud<pointType>::Ptr(new pcl::PointCloud<pointType>());

}
void BallRecognition::spin()
{
	ros::Rate r(10);
	while(ros::ok() && !viewer->wasStopped ()) {
		ros::spinOnce();
		viewer->spinOnce (10);
		r.sleep();
	}

}

void BallRecognition::filterVoxelGrid()
{
	pcl::VoxelGrid<pointType> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.002f, 0.002f, 0.002f);
	sor.filter (*cloudTmp);
	std::swap(cloud,cloudTmp);

}
void BallRecognition::filterByColor()
{
	std::cout <<"punti iniziali: "<< cloud->size()<<std::endl;
	{
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
		std::vector<int> indices;

		pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

		pcl::PassThrough<pcl::PointXYZHSV> pass;
		pass.setInputCloud (hsvCloud);
		pass.setFilterFieldName ("h");
		pass.setFilterLimits (hmin, hmax);
		//pass.setFilterLimitsNegative (true);
		pass.filter (indices);
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, indices,*cloudTmp);
		std::cout <<"poi: "<< indices.size()<<std::endl;
		std::swap(cloud,cloudTmp);
	}
	{
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
		std::vector<int> indices;

		pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

		pcl::PassThrough<pcl::PointXYZHSV> pass;
		pass.setInputCloud (hsvCloud);
		pass.setFilterFieldName ("s");
		pass.setFilterLimits (smin, smax);
		//pass.setFilterLimitsNegative (true);
		pass.filter (indices);
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, indices,*cloudTmp);

		std::swap(cloud,cloudTmp);
		std::cout <<"punti rimanenti: "<< indices.size()<<std::endl;
	}


}
void BallRecognition::updateFilterValues()
{
	{
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
		std::vector<double> v;
		pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

		for(pcl::PointCloud<pcl::PointXYZHSV>::iterator it = hsvCloud->begin(); it!=hsvCloud->end(); ++it) {
			v.push_back(it->h);
		}

		double sum = std::accumulate(v.begin(), v.end(), 0.0);
		double mean = sum / v.size();

		double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
		double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
		std::cout <<"media: "<<mean<<"   dev: "<<stdev<<std::endl;
		stdev = std::max(stdev,5.0);
		hmin = mean-2.0*stdev;
		hmax = mean+2.0*stdev;
		hmin = std::min(hmin,38.0f);
		hmax = std::max(hmax,38.0f);
	}

	{
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
		std::vector<double> v;
		pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

		for(pcl::PointCloud<pcl::PointXYZHSV>::iterator it = hsvCloud->begin(); it!=hsvCloud->end(); ++it) {
			v.push_back(it->s);
		}

		double sum = std::accumulate(v.begin(), v.end(), 0.0);
		double mean = sum / v.size();

		double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
		double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
		std::cout <<"media: "<<mean<<"   dev: "<<stdev<<std::endl;
		stdev = std::max(stdev,5.0);
		smin = mean-2.5*stdev;
		smax = mean+2.5*stdev;
	}
}
bool BallRecognition::findSphere(pcl::PointIndices::Ptr inliers,pcl::ModelCoefficients::Ptr coefficients)
{
	 pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal> ()); // for sphere
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	normal_estimation.setSearchMethod (tree3);
normal_estimation.setInputCloud (cloud);
normal_estimation.setKSearch (25);
normal_estimation.compute (*cloud_normals3);
	
	
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
	// Optional
	seg.setOptimizeCoefficients (false);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_SPHERE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

	seg.setInputCloud (cloud);
	seg.setInputNormals (cloud_normals3);
	seg.setRadiusLimits (0.09, 0.12);
	//seg.setProbability(0.2);
	seg.setMaxIterations(2000);

	seg.segment (*inliers, *coefficients);


	if(inliers->indices.size() < 10)
		return false;
	return true;
}
/*
void BallRecognition::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& c)
{
	std::cout <<"pcl"<<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg( *c,*cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
	if(!viewer->updatePointCloud<pcl::PointXYZ>(cloud, ss.str())) {
		viewer->addPointCloud<pcl::PointXYZ>(cloud,  ss.str());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,100,100,100,ss.str() );
	}
    j++;
  }
}*/


void BallRecognition::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& c)
{

	pcl::fromROSMsg( *c,*cloud);

	std::cout << "prima del grid"<<cloud->size()<<std::endl;
	// filtra come voxel grid

	filterVoxelGrid();
	filterByColor();
	//


	//pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

	//pcl::PointCloudXYZRGBtoXYZHSV (*cloud,*hsvCloud);

	// elimina i punti distanti
	/*pcl::PassThrough<pointType> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("h");
	pass.setFilterLimits (0.3, 0.7);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloudTmp);
	std::swap(cloud,cloudTmp);
	*/

	// trova le sfere




	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object


	bool found = findSphere(inliers,coefficients);

	if(!found) {
		std::cerr <<"fallito"<<std::endl;
	} else {
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		          << coefficients->values[1] << " "
		          << coefficients->values[2] << " "
		          << coefficients->values[3] << std::endl;


		pcl::ExtractIndices<pointType> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloudTmp);
		std::swap(cloud,cloudTmp);


		updateFilterValues();
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	if(!viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb,"sample cloud")) {
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "ball_recognition");
	BallRecognition br;
	br.spin();
	return 0;


}
