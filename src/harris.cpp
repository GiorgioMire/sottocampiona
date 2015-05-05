
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/harris_3d.h>
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
const int distance =2000;//7000;
 // 
//int contatore_save=0;
//Mat rgbframe;
//Mat depthframe;

 struct cookiesStruct{
  // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  // cv::Mat rgb;
  // cv::Mat depth;
  // std::string percorso;
  float risoluzione=0.01;
  float* dim;
  float* raggio_normali;
  float* raggio_harris;
};

void GestoreEventoTastoR(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;


*(cookies->raggio_harris)+=(cookies->risoluzione);
  std::cout<<std::endl<<"Raggio harris"<<*(cookies->raggio_harris);

}

void GestoreEventoTastoF(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;
if(*(cookies->raggio_harris)-(cookies->risoluzione)>0)
  *(cookies->raggio_harris)-=(cookies->risoluzione);
  std::cout<<std::endl<<"Raggio harris"<<*(cookies->raggio_harris);

}

void GestoreEventoTastoW(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;

  *(cookies->dim)+=(cookies->risoluzione);
  std::cout<<std::endl<<"Volume di campionamento "<<*(cookies->dim);

}
void GestoreEventoTastoS(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;
if (*(cookies->dim)-(cookies->risoluzione)>0.0)
  *(cookies->dim)-=(cookies->risoluzione);
  std::cout<<std::endl<<"Volume di campionamento "<<*(cookies->dim);

}

// void GestoreEventoTastoR(void* cookiesVoid){
//   cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;
//   static int contatoreSalvataggi=0;
//   std::cout << "Hai premuto il tasto w; salvo la cloud " << std::endl;
//     contatoreSalvataggi++;

//     pcl::io::savePCDFileASCII ((cookies->percorso)+"PointCloud"+std::to_string(contatoreSalvataggi)+".pcd", *(cookies->cloud));
//     cv::imwrite( (cookies->percorso)+"RGB"+std::to_string(contatoreSalvataggi)+".jpg", cookies->rgb);
//     cv::imwrite( (cookies->percorso)+"DEPTH"+std::to_string(contatoreSalvataggi)+".jpg", cookies->depth);

// }

void filtra(cloudptr_type cloud, cloudptr_type cloud_filtered_ptr,float dim){
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (dim, dim,dim);
  sor.filter (*cloud_filtered_ptr);
}


void tagliasfondo(cloudptr_type cloud, cloudptr_type cloud_cutted,float zcut){
 pcl::PointXYZRGB p;
  for(int i=0; i<cloud->points.size();i++){

    if(cloud->points[i].z<zcut){
      p.x=cloud->points[i].x;
      p.y=cloud->points[i].y;
      p.z=cloud->points[i].z;
       p.rgb=cloud->points[i].rgb;
    cloud_cutted->push_back(p);
    }

  }




}



void trovakeypoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud,pcl::PointCloud<pcl::PointXYZI>::Ptr result,float raggio_normali,float raggio_harris){
//boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;

pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D 
= new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> 
(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius (raggio_normali);
    harris3D->setRadiusSearch (raggio_harris);
    //keypoint_detector.reset(harris3D);
    harris3D->setInputCloud(inputcloud);
    harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
harris3D->compute(*result);

 
}

 




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookiesVoid)
{

  if (event.getKeySym () == "w" && event.keyDown ())
    GestoreEventoTastoW( cookiesVoid);
  if (event.getKeySym () == "s" && event.keyDown ())
    GestoreEventoTastoS( cookiesVoid);
   if (event.getKeySym () == "r" && event.keyDown ())
    GestoreEventoTastoR( cookiesVoid);
  if (event.getKeySym () == "f" && event.keyDown ())
    GestoreEventoTastoF( cookiesVoid);

}



/*____________________________________________________________________________________________*/

int main( int argc, char** argv )
{
  float dim(0.01);
float raggio_normali(0.01);
float raggio_harris(0.01);

PC_type cloud_filtered=PC_type();
  /*Dichiarazioni*/
// std::string nomeSoggetto;
cloudptr_type cloud;
cloudptr_type cloud_filtered_ptr(new PC_type);
cloudptr_type cloud_cutted_ptr(new PC_type);
freenectGrabber<pcl::PointXYZRGB> c;
cookiesStruct cookies;
cookies.dim=&dim;
cookies.raggio_normali=&raggio_normali;
cookies.raggio_harris=&raggio_harris;
/* Fase iniziale*/


  cloud = c.get_point_cloud(distance, true);
  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;






/*___________________________VIEWER 1________________________________*/


 VisualizzatorePTR_type viewer (new Visualizzatore_type("3D Viewer"));
int v1(0);
viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (255, 255, 255,v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud",v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud",v1);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cookies);

filtra(cloud,cloud_filtered_ptr,dim);
 std::cout<<"Fin qui tutto bene";
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
trovakeypoint(cloud_filtered_ptr,keypoints,raggio_normali,raggio_harris);
std::cout<<std::endl<<"Sono stati trovati keypoint"<<(*keypoints).size();
  keypoints->sensor_orientation_.w () = 0.0;
  keypoints->sensor_orientation_.x () = 1.0;
  keypoints->sensor_orientation_.y () = 0.0;
  keypoints->sensor_orientation_.z () = 0.0;



int v2(1);
viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (255, 255, 255,v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_filtered_ptr);

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered_ptr, rgb2, "sample cloud2",v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2",v2);


pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(keypoints, 255, 0, 0);
viewer->addPointCloud(keypoints,single_color,"keypoints",v1); 
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints",v1);
 
 std::cout<<"Fin qui tutto bene";

  while (!viewer->wasStopped ()) {
viewer->spinOnce ();
cloud = c.get_point_cloud(distance, true);
filtra(cloud,cloud_filtered_ptr,dim);
//tagliasfondo(cloud_filtered_ptr,cloud_cutted_ptr,10);
trovakeypoint(cloud_filtered_ptr,keypoints,raggio_normali,raggio_harris);

pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_filtered_ptr);

    
   
 

viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 
viewer->updatePointCloud<pcl::PointXYZRGB> (cloud_filtered_ptr, rgb2, "sample cloud2"); 
 viewer->updatePointCloud<pcl::PointXYZI> (keypoints, single_color, "keypoints"); 
  
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud_filtered_ptr->height = 1;
  cloud_filtered_ptr->width = cloud_filtered_ptr->points.size();
  return 0;
}



