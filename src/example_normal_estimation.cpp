
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <cstdlib>

typedef pcl::PointXYZ PointType;

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<pcl::PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
	return p.z;
      }
    };
}

std::string int2string(int aa)
{
    std::strstream ss;
    ss << aa;
    std::string out ;
    ss >> out;
    return out;
}

pcl::PointCloud<PointType>::Ptr getNearK(pcl::PointXYZ search, const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &src,int order)
{
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

      kdtree.setInputCloud(src);

      pcl::PointXYZ searchPoint = search;

      int K = 2000;

      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);

      std::cout << "K nearest neighbor search at (" << searchPoint.x
                << " " << searchPoint.y
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

      pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {

          dest->resize(pointIdxNKNSearch.size ());
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
    /*      std::cout << "    "  <<  src->points[ pointIdxNKNSearch[i] ].x
                    << " " << src->points[ pointIdxNKNSearch[i] ].y
                    << " " << src->points[ pointIdxNKNSearch[i] ].z
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
         dest->points[i] = src->points[ pointIdxNKNSearch[i]];
        }
    //    pcl::io::savePCDFileASCII("../test/"+int2string(order)+".pcd",*dest);
      }
      return dest;
}

pcl::PointCloud<PointType>::Ptr getNearKK(pcl::PointXYZ search, const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &src,int order)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(src);

	pcl::PointXYZ searchPoint = search;

    pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);
#if 0
	int K = 2000;

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	std::cout << "K nearest neighbor search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with K=" << K << std::endl;
	//  pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);
	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
//	  pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);
	  dest->resize(pointIdxNKNSearch.size ());
	  for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	  {
	/*	  std::cout << "    "  <<  src->points[ pointIdxNKNSearch[i] ].x
					<< " " << src->points[ pointIdxNKNSearch[i] ].y
					<< " " << src->points[ pointIdxNKNSearch[i] ].z
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
		 dest->points[i] = src->points[ pointIdxNKNSearch[i]];
	  }
	//  pcl::io::savePCDFileASCII("../test/"+int2string(order)+".pcd",*dest);
	}

#else
	float radius = 0.50f;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	std::cout << "Neighbors within radius search at (" << searchPoint.x
			  << " " << searchPoint.y
			  << " " << searchPoint.z
			  << ") with radius=" << radius  ;//<< std::endl;
	//pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);
	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
	//  pcl::PointCloud<PointType>::Ptr dest(new pcl::PointCloud<PointType>);
	  dest->resize(pointIdxRadiusSearch.size ());
	  for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	  {
	/*	std::cout <<"   "<< src->points[ pointIdxRadiusSearch[i] ].x
				  << " " << src->points[ pointIdxRadiusSearch[i] ].y
				  << " " << src->points[ pointIdxRadiusSearch[i] ].z
				  << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;*/
		dest->points[i] = src->points[ pointIdxRadiusSearch[i]];
	  }
//	  pcl::io::savePCDFileASCII("../test/"+int2string(order)+".pcd",*dest);
	}
    std::cout << " NO. of contains points:"<<dest->points.size()  << std::endl;
#endif

	  return dest;
}


/*
 * #include <iostream>
#include <vector>
#include <time.h>
#include <math.h>
using namespace std;

#define Max(a,b) ((a)>(b)? (a):(b))

const float THRESHOLD = 1.0;
typedef struct aboDis
{
  float distance;
  int index;
}AboDis;



int cmp ( const void *a , const void *b )
{

  return  ((AboDis*)a)->distance - ((AboDis *)b)->distance;
}

int KeyPointPart(vector<AboDis> &keypoints, int low ,int high)
{
	AboDis middle = keypoints[low];

	while (low < high)
	{
	   while ((low < high) && (keypoints[high].distance >= middle.distance)) high--;
	        keypoints[low] = keypoints[high];
	   while ((low < high) && (keypoints[low].distance <= middle.distance)) low++;
	        keypoints[high] = keypoints[low];
	}

    keypoints[low] = middle;

	return low;
}

inline void swap(AboDis * a, AboDis *b)
{
   AboDis temp = *a;
  *a = *b;
   *b = temp;
}

void KeyPointSort(vector<AboDis> &keypoints, int low , int high)
{
   if ( low >= high) return;
   int middle = KeyPointPart(keypoints, low, high);
   KeyPointSort(keypoints, low, middle-1 );
   KeyPointSort(keypoints, middle+1 , high);
}

int main()
{
    vector<AboDis> ivec;
    int i;
	srand((int)time(0));

    for(i = 0;i < 20;++i)
	{
		AboDis ll;
		ll.distance =static_cast<float>(rand()%10000)/100;
		ll.index = i;
		ivec.push_back(ll);
	}
    for(vector<AboDis>::iterator it = ivec.begin();it != ivec.end();++it)
		cout << it->distance << "  " << it->index <<endl;

	KeyPointSort(ivec, 0 , ivec.size()-1);

    std::cout << std::endl;
 	for(vector<AboDis>::iterator it = ivec.begin();it != ivec.end(); ++it)
		cout << it->distance << "  " << it->index <<endl;

vector<AboDis>::iterator it= ivec.begin();
#if 0
	for (std::vector<AboDis>::iterator it = ivec.begin(); it < ivec.end() - 1; it++)
	{
	   std::vector<AboDis>::iterator start = it+1;
	   while (start  < ivec.end()  && (abs(it->distance - start->distance) < 0.1))
	   {
		   if (abs(it->distance - start->distance) < 0.1)
		  {
			  it = ivec.erase(start);
		  }
	///	  else
			 start++;
		//	break;
	   }
	}
#else
     while ( it < ivec.end() - 1)
	 {
		 if(abs(it->distance - (it+1)->distance ) < THRESHOLD )
		{
			if (it->distance > (it+1)->distance)
			{
				swap(it,it+1);
				ivec.erase(it);

			}
			else
			{
				it = ivec.erase( it ) ;
         //       it = ivec.erase( it+1 ) ;
		//		it--;
			}
		}
		else
		   it++;
	 }

#endif
	std::cout << std::endl;
 	for(vector<AboDis>::iterator it = ivec.begin(); it != ivec.end(); ++it)
		cout << it->distance << "  " << it->index <<endl;
    return 0;
}
 *
 * */
int quicksort(pcl::PointCloud<PointType>::Ptr mypoints, int low , int high)
{
   PointType middle = mypoints->points[low];

   while (low < high)
   {
	   while (low < high && mypoints->points[high].x > middle.x ) high--;
	     mypoints->points[low] = mypoints->points[high];

	   while (low < high && mypoints->points[low].x < middle.x) low++;
	   mypoints->points[high] = mypoints->points[low];

   }

   mypoints->points[low] = middle;
   return low;
}


void SortKeyPoints(pcl::PointCloud<PointType>::Ptr keypoints, int low ,int high)
{
	if ( low >= high) return;
   	int middle = quicksort(keypoints, low ,high);
   	SortKeyPoints(keypoints, low, middle - 1);
   	SortKeyPoints(keypoints, middle, high);
   	return ;
}

char *itoa(int value,char *string,int radix)
{
   int rt=0;
   if(string==NULL)
      return NULL;
   if(radix<=0||radix>30)
      return NULL;
   rt=snprintf(string,radix,"%d",value);
   if(rt>radix)
      return NULL;
   string[rt]='\0';
   return string;
}

int main(int, char** argv)
{
  std::string filename = "../sincos.pcd";
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
  std::cout << "points: " << cloud_xyz->points.size () <<std::endl;


  // Parameters for sift computation
  const float min_scale = 0.02f; //0.005f,与ｖoxfilter 相关，应当和点云中的点集的点间距相当才可，不可过大或者过小
  const int n_octaves = 10;//6
  const int n_scales_per_octave = 4;//4
  const float min_contrast = 0.02f;//0.005


  // Estimate the sift interest points using z values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_xyz);
  sift.compute(result);

  std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

  // Copying the pointwithscale to pointxyz so as visualize the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(result, *cloud_temp);
  std::cout << "SIFT points in the result are " << cloud_temp->points.size () << std::endl;
  //////////////////////////////////////////////////
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud_xyz);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (tree2);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  normal_estimation.setRadiusSearch (0.05);
  // Compute the features
  normal_estimation.compute (*cloud_normals);
///////////////////////////////////////////////////////////

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud_xyz, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(cloud_xyz, cloud_color_handler, "cloud");
  viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");


//  SortKeyPoints(cloud_temp, 0, cloud_temp->points.size());
  std::cout << " Begin to Sort KeyPoints!" << std::endl;
  for (std::size_t i = 0; i < cloud_temp->points.size(); i++)
	  std::cout << cloud_temp->points[i] << std::endl;
  for (std::size_t i = 0; i < cloud_temp->points.size(); i++)
  {
    std::cout <<i<< " " << cloud_temp->points[i] <<std::endl;
	std::cout <<"     " << cloud_normals->points[i] << std::endl;
	char str[50];
	itoa(i,str,10);
	viewer.addText3D(str,cloud_temp->points[i], 0.1, 1.0 ,1.0 ,1.0, str,0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pp = getNearKK(cloud_temp->points[i], cloud_xyz,i);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_near_color(pp, 0, 0, 255);

	viewer.addPointCloud(pp, keypoints_near_color, strcat(str,"nearkeypoints"));
	//viewer.addSphere(cloud_temp->points[i], 0.35, 233,145 ,122,  std::strcat(str,"sphere"),0);
    //viewer.addCircle(new pcl::ModelCoefficients(cloud_temp->points[i].x,cloud_temp->points[i].y, 0.35), std::strcat(str,"sphere") ,0);
  }

  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
	pcl_sleep(0.1);

  }

  return 0;
}
