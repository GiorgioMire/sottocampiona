
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <pcl/filters/voxel_grid.h>
#include "freenect_grabber.hpp"
#include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/core/core.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <string.h>
#include "boost/filesystem.hpp"

typedef  boost::shared_ptr<pcl::visualization::PCLVisualizer>  VisualizzatorePTR_type;
typedef pcl::visualization::PCLVisualizer Visualizzatore_type;
typedef pcl::PointCloud<pcl::PointXYZRGB> PC_type;
typedef boost::shared_ptr<PC_type> cloudptr_type;
const int distance = 7000;
 // 
//int contatore_save=0;
//Mat rgbframe;
//Mat depthframe;

 struct cookiesStruct{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  cv::Mat rgb;
  cv::Mat depth;
  std::string percorso;
};


void GestoreEventoTastoR(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;
  static int contatoreSalvataggi=0;
  std::cout << "Hai premuto il tasto r; salvo la cloud " << std::endl;
    contatoreSalvataggi++;

    pcl::io::savePCDFileASCII ((cookies->percorso)+"PointCloud"+std::to_string(contatoreSalvataggi)+".pcd", *(cookies->cloud));
    cv::imwrite( (cookies->percorso)+"RGB"+std::to_string(contatoreSalvataggi)+".jpg", cookies->rgb);
    cv::imwrite( (cookies->percorso)+"DEPTH"+std::to_string(contatoreSalvataggi)+".jpg", cookies->depth);

}

void filtra(cloudptr_type cloud, cloudptr_type cloud_filtered_ptr){
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_ptr);
}
 




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookiesVoid)
{

  if (event.getKeySym () == "r" && event.keyDown ())
    GestoreEventoTastoR( cookiesVoid);
}


/*____________________________________________________________________________________________*/

int main( int argc, char** argv )
{
PC_type cloud_filtered=PC_type();
  /*Dichiarazioni*/
// std::string nomeSoggetto;
cloudptr_type cloud;
cloudptr_type cloud_filtered_ptr(new PC_type);
freenectGrabber<pcl::PointXYZRGB> c;
cookiesStruct cookies;

/* Fase iniziale*/


  cloud = c.get_point_cloud(distance, true);
  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;






/*___________________________VIEWER 1________________________________*/


 VisualizzatorePTR_type viewer (new Visualizzatore_type("3D Viewer"));

  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cookies);

filtra(cloud,cloud_filtered_ptr);
 std::cout<<"Fin qui tutto bene";



  /*_______________________VIEWER 2 __________________________________*/
   VisualizzatorePTR_type viewer2 (new Visualizzatore_type("3D Viewer2"));

  viewer2->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_filtered_ptr);

  viewer2->addPointCloud<pcl::PointXYZRGB> (cloud_filtered_ptr, rgb2, "sample cloud");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer2->registerKeyboardCallback (keyboardEventOccurred, (void*)&cookies);
 
 std::cout<<"Fin qui tutto bene";

  while (!viewer->wasStopped ()) {
//memcpy(rgbframe.ptr(),(void*)&c.rgb.front(),640*480*3); 
//memcpy(depthframe.ptr(),(void*)&c.depth_map.front(),640*480*2);






//cv::cvtColor(rgbframe,rgbframe, cv::COLOR_BGR2RGB);
//cout<<depthframe;
//depthframe.convertTo(mapped,CV_8U,256.0/4096.0);

viewer->spinOnce ();
viewer2->spinOnce();
cloud = c.get_point_cloud(distance, true);
filtra(cloud,cloud_filtered_ptr);














    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 
  viewer2->updatePointCloud<pcl::PointXYZRGB> (cloud_filtered_ptr, rgb, "sample cloud"); 
  
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud_filtered_ptr->height = 1;
  cloud_filtered_ptr->width = cloud_filtered_ptr->points.size();
  return 0;
}



