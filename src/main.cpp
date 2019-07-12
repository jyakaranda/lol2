/**
 * @file main.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "lol2/lolLocalization.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto lol = std::make_shared<localization::LolLocalization>();
  lol->run(lol);
  rclcpp::spin(lol);
  rclcpp::shutdown();
  return 0;
}