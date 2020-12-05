#include "../include/controller/kobuki_controller.hpp"

int main(int argc, char **argv) {
  std::string nodeName = "is_kobuki_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;
  KobukiController kobukiController(nh);
  kobukiController.init();

  // AMCL tra vi tri hien tai (3,3)
  
  return 0;
}
