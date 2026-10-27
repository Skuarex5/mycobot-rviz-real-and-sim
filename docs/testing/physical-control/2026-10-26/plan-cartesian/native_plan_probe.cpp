#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("native_plan_regression",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 int result=1;
 {
 moveit::planning_interface::MoveGroupInterface mgi(node,"arm_group");
 moveit::planning_interface::MoveGroupInterface::Plan plan;
 for(int i=0;i<5;++i) {
  auto code=mgi.plan(plan);
  std::cout<<"Rejected plan returned error="<<code.val<<std::endl;
  if(code.val==1) return 2;
  code=mgi.move();
  std::cout<<"Blocked Plan & Execute returned error="<<code.val<<std::endl;
  if(code.val==1) return 3;
  code=mgi.execute(plan);
  std::cout<<"Invalid Execute returned error="<<code.val<<std::endl;
  if(code.val==1) return 4;
 }
 result=0;
 }
 rclcpp::shutdown();return result;
}
