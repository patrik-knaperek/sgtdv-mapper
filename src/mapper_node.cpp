/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Patrik Knaperek, Filip Botka
/*****************************************************/

#include "../include/mapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;

  Mapper mapper(nh); 
  
  ros::spin();

  return 0;
}