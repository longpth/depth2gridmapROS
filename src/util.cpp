#include <dirent.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include "util.h"

namespace depth2gridmap {

  namespace util{

    //Logicool
    //[[932.31743899,   0.        , 454.96161939],
    //[  0.        , 935.95155411, 248.79242231],
    //[  0.        ,   0.        ,   1.        ]]

    static const float FX = 718.856; //5.1885790117450188e+02;//4.34624501e+03/2.0;
    static const float FY = 718.856; //5.1946961112127485e+02;//3.54253924e+03/2.0;
    static const float CX = 607.1928;//3.2558244941119034e+02;//3.19699098e+02;
    static const float CY = 185.2157;//2.5373616633400465e+02;//2.06725334e+02;

    static const float RZ_X_RATE = 416.0/1241.0;
    static const float RZ_Y_RATE = 128.0/376.0;

    static std::vector<int> gCountGrid;
    static std::vector<int8_t> gOcGrid;
    static double gCellResolution;
    static int gGridWidth;
    static int gGridHeight;

    typedef union {
      struct /*anonymous*/ {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
      };
      float float_value;
      long long_value;
    } RGBValue;

    std::vector<std::string> GetDirectoryFiles(const std::string& dir) {
      std::vector<std::string> files;
      std::shared_ptr<DIR> directory_ptr(opendir(dir.c_str()), [](DIR* dir){ dir && closedir(dir); });
      struct dirent *dirent_ptr;
      if (!directory_ptr) {
        std::cout << "[DEBUG]" << "Error opening : " << std::strerror(errno) << dir << std::endl;
        return files;
      }
    
      while ((dirent_ptr = readdir(directory_ptr.get())) != nullptr) {
        files.push_back(dir+'/'+std::string(dirent_ptr->d_name));
      }
      // Sorting the string vector
      sort(files.begin(), files.end(), std::greater<std::string>());
      //remove ".." and "." elements
      files.pop_back();
      files.pop_back();
      return files;
    }

    static void getCameraIntrinsicsInverseFocalLength(CAM_INFO caminfo, float& fxinv, float& fyinv, float& cx, float& cy, float scaleX, float scaleY) {
      fxinv = caminfo.fx*scaleX;
      fyinv = caminfo.fy*scaleY;
      cx = caminfo.cx*scaleX;
      cy = caminfo.cy*scaleY;
      fxinv = 1./ fxinv;
      fyinv = 1./ fyinv;
    }

    /*
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Calculate surface normals with a search radius of 0.03
    */
    static void calcSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double radius=0.03) {
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(cloud);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(radius);
      ne.compute(*cloud_normals);
    }


    /*
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Calculate size of Occupancy Grid
    */
    static void calcSize(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double *xMax, double *yMax, double *xMin, double *yMin){
      for (size_t i = 0; i < cloud->size(); i++){
          double x = cloud->points[i].x;
          double y = cloud->points[i].y;
          if (*xMax < x) {
            *xMax = x;
          }
          if (*xMin > x) {
            *xMin = x;
          }
          if (*yMax < y) {
            *yMax = y;
          }
          if (*yMin > y) {
            *yMin = y;
          }
      }
    }

    /*
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Polpulate grid with cost values
    */
    static void populateMap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map, 
                            double cellResolution, double yMax, double deviation=0.78539816339
                            ){
      for (size_t i = 0; i < cloud->size(); i++) {
        double x = cloud->points[i].x + cellResolution*(double)gGridWidth/2.0; // move point cloud to the middle of grid
        double y = cloud->points[i].y;
        double z = cloud_normals->points[i].normal_z;

        double phi = acos(fabs(z));
        int xCell, yCell;

        if (z == z && y < yMax) {
          xCell = (int) (x / cellResolution);
          yCell = (int) (y / cellResolution);
          if ((yCell * gGridWidth + xCell) > (gGridWidth * gGridHeight)) {
            std::cout << "[DEBUG]" << "x: " << x << ", y: " << y << ", xCell: " << xCell << ", yCell: " << yCell
                << "\n";
          }
          if (phi > deviation) {
            map[yCell * gGridWidth + xCell]++;
          } else {
            map[yCell * gGridWidth + xCell]--;
          }
        }
      }
    }

    /*
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Generate Occupancy Grid
    */
    static void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int buf=5) {
      for (int i = 0; i < countGrid.size(); i++) {
        if (countGrid[i] < buf) {
          ocGrid[i] = 0;
        } else if (countGrid[i] > buf) {
          ocGrid[i] = countGrid[i] < 100 ? countGrid[i] : 100;
        } else if (countGrid[i] == 0) {
          ocGrid[i] = 0;
        }
      }
    }

    /* 
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Initialize Occupancy Grid Msg
    */ 
    void initGrid(nav_msgs::OccupancyGridPtr grid, std::string frame_id, int width, int height, double cellResolution) {
      grid->header.seq                = 1;
      grid->header.frame_id           = frame_id;
      grid->info.origin.position.z    = 0;
      grid->info.origin.position.x    = -(double)width*cellResolution/2.0;// move point cloud to the middle of grid
      grid->info.origin.position.y    = 0;
      grid->info.origin.orientation.w = 1;
      grid->info.origin.orientation.x = 0;
      grid->info.origin.orientation.y = 0;
      grid->info.origin.orientation.z = 0;
      grid->info.width                = width;
      grid->info.height               = height;
      grid->info.resolution           = cellResolution;
      gCountGrid                      = std::vector<int>(width*height);
      gOcGrid                         = std::vector<int8_t>(width*height);
      gCellResolution                 = cellResolution;
      gGridWidth                      = width;
      gGridHeight                     = height;
    }

    /*
    * Reference source: https://library.ndsu.edu/ir/bitstream/handle/10365/25535/Huesman%20Research%20Presentation.pdf?sequence=1&isAllowed=y
    * Update Occupancy Grid Msg
    */
    void updateGrid(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                  nav_msgs::OccupancyGridPtr grid, tf::Transform trans, double maxDepth) {
      /* Figure out size of matrix needed to store data. */
      double xMax = 0, yMax = 0, xMin = 0, yMin = 0;
      calcSize(cloudIn, &xMax, &yMax, &xMin, &yMin);
      std::cout << "[DEBUG]" << "xMax: " << xMax << ", yMax: " << yMax << ", xMin: " << xMin 
          << ", yMin: " << yMin 
          << ", maxDepth: " << maxDepth
          << "\n";
      
      if (yMax > maxDepth) yMax = maxDepth;

      /* Determine resolution of grid (m/cell) */
      int xCells = ((int) ((xMax - xMin) / gCellResolution)) + 1;
      int yCells = ((int) ((yMax - yMin) / gCellResolution)) + 1;
      std::cout << "[DEBUG]" << "xCells: " << xCells << ", yCells: " << yCells << "\n";

      /* Calculate Surface Normals */
      calcSurfaceNormals(cloudIn, cloud_normals);
      /* Populate Map */
      populateMap(cloudIn, cloud_normals, gCountGrid, gCellResolution, yMax);
      /* Generate OccupancyGrid Data Vector */
      genOccupancyGrid(gOcGrid, gCountGrid);

      if(grid == NULL)
        return;

      /* Update grid */
      grid->header.seq++;
      grid->header.stamp.sec       = ros::Time::now().sec;
      grid->header.stamp.nsec      = ros::Time::now().nsec;
      grid->info.map_load_time     = ros::Time::now();
      grid->data                   = gOcGrid;
    }

    /*
    * Create XYZRGB point cloud from depth image and rgb image
    */
    pcl::PointCloud<pcl::PointXYZRGB>* createXYZRGBPointCloud ( const cv::Mat& depth_img, 
                                                                const cv::Mat& rgb_img,
                                                                float scaling,
                                                                float maxDepth,
                                                                CAM_INFO camInfo) {
      pcl::PointCloud<pcl::PointXYZRGB>* cloud (new pcl::PointCloud<pcl::PointXYZRGB>() );
      cloud->is_dense         = true; //single point of view, 2d rasterized NaN where no depth value was found

      float fxinv, fyinv, cx, cy;
      float scaleX = rgb_img.cols/camInfo.width;
      float scaleY = rgb_img.rows/camInfo.height;
      getCameraIntrinsicsInverseFocalLength(camInfo, fxinv, fyinv, cx, cy, scaleX, scaleY);
      cloud->height = depth_img.rows;
      cloud->width = depth_img.cols;

      cloud->points.resize (cloud->height * cloud->width);

      pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
      for (int y = 0; y < depth_img.rows; y++)
        for (int x = 0; x < depth_img.cols; x++, ++pt_iter){
          pcl::PointXYZRGB& pt = *pt_iter;
          int pixel = x+y*depth_img.cols;
          pt.z = (float)depth_img.at<uint16_t>(pixel)*scaling;
          #if 1
          if(pt.z > maxDepth){
            pt.z = std::numeric_limits<float>::quiet_NaN();
            pt.x = ((float)x - cx) * 1.0 * fxinv;
            pt.y = -((float)y - cy) * 1.0 * fyinv;
          }else{
            pt.x = ((float)x - cx) * pt.z * fxinv;
            pt.y = -((float)y - cy) * pt.z * fyinv;
          }
          #else
          pt.x = ((float)x - cx) * pt.z * fxinv;
          pt.y = ((float)y - cy) * pt.z * fyinv;
          #endif
          RGBValue color;//bgr8
          color.Red   = rgb_img.at<uint8_t>(pixel + 2);
          color.Green = rgb_img.at<uint8_t>(pixel + 1);
          color.Blue  = rgb_img.at<uint8_t>(pixel + 0);
          pt.rgb = color.float_value;
        }
      return cloud;
    }

    static bool debug = true;
    /*
    * Create XZYRGB point cloud from depth image and rgb image
    */
    pcl::PointCloud<pcl::PointXYZRGB>* createXZYRGBPointCloud ( const cv::Mat& depth_img, 
                                                                const cv::Mat& rgb_img,
                                                                float scaling,
                                                                float maxDepth,
                                                                CAM_INFO camInfo) {
      pcl::PointCloud<pcl::PointXYZRGB>* cloud (new pcl::PointCloud<pcl::PointXYZRGB>() );
      cloud->is_dense         = true; //single point of view, 2d rasterized NaN where no depth value was found

      float fxinv, fyinv, cx, cy;
      float scaleX = (float)rgb_img.cols/camInfo.width;
      float scaleY = (float)rgb_img.rows/camInfo.height;
      getCameraIntrinsicsInverseFocalLength(camInfo, fxinv, fyinv, cx, cy, scaleX, scaleY);

      if(debug){
        std::cout << "[DEBUG]" << "fxinv " << fxinv
                  << " fyinv " << fyinv
                  << " cx " << cx
                  << " cy " << cy
                  << " scaleX "<< scaleX
                  << " scaleY "<< scaleY
                  << std::endl;
        debug = false;
      }

      cloud->height = depth_img.rows;
      cloud->width = depth_img.cols;

      cloud->points.resize (cloud->height * cloud->width);

      pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
      for (int y = 0; y < depth_img.rows; y++)
        for (int x = 0; x < depth_img.cols; x++, ++pt_iter){
          pcl::PointXYZRGB& pt = *pt_iter;
          int pixel = x+y*depth_img.cols;
          pt.y = (float)depth_img.at<uint16_t>(pixel)*scaling;
          #if 0
          if(pt.y > maxDepth){
            pt.y = maxDepth;
            pt.x = 0;
            pt.z = 0;
          }else{
            pt.x = ((float)x - cx) * pt.y * fxinv;
            pt.z = -((float)y - cy) * pt.y * fyinv;
          }
          #else
          pt.x = ((float)x - cx) * pt.y * fxinv;
          pt.z = -((float)y - cy) * pt.y * fyinv;
          #endif
          RGBValue color;//bgr8
          color.Red   = rgb_img.at<uint8_t>(pixel + 2);
          color.Green = rgb_img.at<uint8_t>(pixel + 1);
          color.Blue  = rgb_img.at<uint8_t>(pixel + 0);
          pt.rgb = color.float_value;
        }
      
      return cloud;
    }

    void print4x4Matrix (const Eigen::Matrix4d & matrix) {
      printf ("Rotation matrix :\n");
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
      printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
      printf ("Translation vector :\n");
      printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }
  }

} //namespace