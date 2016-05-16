#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/gicp.h>   //Generalized Iterative Closest Point
#include <pcl/registration/joint_icp.h>   //Joint Iterative Closest Point
#include <pcl/registration/icp_nl.h>   //Iterative Closest Point Non Linear

//typedef pcl::PointXYZ PointT;   //Iterative Closest Point & Generalized Iterative Closest Point
typedef pcl::PointXYZINormal PointT;   //Iterative Closest Point With Normals
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::IterativeClosestPoint<PointT, PointT> icp_default;
typedef pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp_general;
typedef pcl::IterativeClosestPointWithNormals<PointT, PointT> icp_normal;
typedef pcl::IterativeClosestPointNonLinear<PointT, PointT> icp_nonlin;
typedef pcl::JointIterativeClosestPoint<PointT, PointT> icp_joint;

//global variables
long int timeCnt;
bool next_iteration = false;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

int main (int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in_1 (new PointCloudT); // Original point cloud
  PointCloudT::Ptr cloud_in_2 (new PointCloudT); // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);   // Transformed point cloud

  // Checking program arguments
  const int numOfITems = 4;
  if (argc < numOfITems)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file1.ply name_of_file2.ply number_of_ICP_variant number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide two ply file.\n");
    return (-1);
  }

  // Get iterations
  int iterations = 1;
  if (argc > numOfITems)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[numOfITems]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  // Load files
  pcl::console::TicToc time;
  time.tic ();
  if ( pcl::io::loadPLYFile (argv[1], *cloud_in_1) < 0 || pcl::io::loadPLYFile (argv[2], *cloud_in_2) < 0)
  {
    PCL_ERROR ("Error loading clouds %s %s.\n", argv[1], argv[2]);
    return (-1);
  }
  std::cout << "\nLoaded files: "
                << argv[1] << " (" << cloud_in_1->size () << " points) AND "
                << argv[2] << " (" << cloud_in_2->size ()
                << " points) in " << time.toc () << " ms\n" << std::endl;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // Executing the transformation
  *cloud_tr = *cloud_in_2;  // We backup cloud_icp into cloud_tr for later use

  // The Iterative Closest Point algorithm
  time.tic ();

  //choose ICP
  std::cout << "Choosed ICP variant: ";

  icp_default* ptr;
  switch(atoi(argv[3]))
  {
    case 1: //"icp":
      {
        std::cout << "1 - ICP" << std::endl;
        ptr = new icp_default();
      }
      break;

    case 2: //"gicp":
      {
        std::cout << "2 - Generalized ICP" << std::endl;
        ptr = new icp_general();
      }
      break;

    case 3: //"nicp":
      {
        std::cout << "3 - ICP with normals" << std::endl;
        ptr = new icp_normal();
      }
      break;

    case 4: //"nlicp":
      {
        std::cout << "4 - Non linear ICP" << std::endl;
        ptr = new icp_nonlin();
      }
      break;

    case 5: //"jicp":
      {
        std::cout << "5 - Joint ICP" << std::endl;
        ptr = new icp_joint();
    }
      break;

    default:
      {
        std::cout << "1 - ICP (default)" << std::endl;
        ptr = new icp_default();
      }
      break;
  }

  ptr->setMaximumIterations (iterations);
  ptr->setRANSACOutlierRejectionThreshold(0.5);
  ptr->setInputSource (cloud_in_2);
  ptr->setInputTarget (cloud_in_1);
  ptr->align (*cloud_in_2);
  ptr->setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  timeCnt = time.toc ();

  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << timeCnt << " ms" << std::endl;
  //cout precosion
  const int precision = 10;
  if (ptr->hasConverged ())
  {
    std::cout.precision(precision);
    std::cout << "ICP has converged, score is " << ptr->getFitnessScore () << std::endl;
    std::cout << "ICP transformation " << iterations << " : cloud_in_2 -> cloud_in_1" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
    transformation_matrix = ptr->getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two verticaly separated viewports
  int v1(0);
  int v2(1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_1_color_h (cloud_in_1, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);

  viewer.addPointCloud (cloud_in_1, cloud_in_1_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in_1, cloud_in_1_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_2_color_h (cloud_in_2, 180, 20, 20);
  viewer.addPointCloud (cloud_in_2, cloud_in_2_color_h, "cloud_in_2_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      ptr->align (*cloud_in_2);
      time.toc ();
      timeCnt += time.toc();

      std::cout << "ANOTHER ITERATION:" << std::endl;
      std::cout << "----------------------------------------------------" << std::endl;
      std::cout << "Applied 1 ICP iterations in " << time.toc() << " ms" << std::endl;
      std::cout << "Applied " << iterations << " ICP iterations in " << timeCnt << " ms" << std::endl;

      if (ptr->hasConverged ())
      {
        std::cout.precision(precision);
        std::cout << "ICP has converged, score is " << ptr->getFitnessScore () << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "ICP transformation " << ++iterations << " : cloud_in_1 -> cloud_in_2" << std::endl;
        transformation_matrix *= ptr->getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_in_2, cloud_in_2_color_h, "cloud_in_2_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }

  delete ptr;
  return (0);
}
