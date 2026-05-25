#include <fstream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char* argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  std::string in_file = argv[1];
  reader.read (in_file, *cloud);
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> vertices;
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  //chull.setInputCloud(cloud);
  chull.setInputCloud (cloud_projected);
  chull.setDimension(2);
  chull.setAlpha (0.3);
  chull.reconstruct (*cloud_hull, vertices);

  std::cerr << "Concave hull has: " << cloud_hull->size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  //writer.write ("projection.pcd", *cloud_projected, false);
  writer.write ("output/hull.pcd", *cloud_hull, false);

  std::ofstream outFile;
  outFile.open("output/pointVector.csv");
  outFile << "x,y\n";
  for (int i = 0; i < cloud_hull->points.size(); i++) {
    pcl::PointXYZ p = cloud_hull->points[i];
    outFile << p.x << "," << p.y << "\n";
  }
  outFile.close();

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setWindowName("Concave Hull Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "input model");
  viewer->addPointCloud<pcl::PointXYZ> (cloud_hull, "concave hull");
  viewer->addPolygonMesh<pcl::PointXYZ> (cloud_hull, vertices, "polygon");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "polygon");
  viewer->setRepresentationToWireframeForAllActors();
  viewer->spin();

  return (0);
}
