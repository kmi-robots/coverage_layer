#include "coverage/coverage.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "coverage_node");
  tf::TransformListener tf(ros::Duration(10));

  coverage::Coverage coverage(tf);

  //ros::MultiThreadedSpinner s;
  ros::spin();
  
  return(0);
}
