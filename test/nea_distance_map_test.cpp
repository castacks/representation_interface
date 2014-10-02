#include "representation_interface/nea_distance_map_representation_interface.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  NEA::MappingClient mapping_object;

  // testing intialization
  if(mapping_object.initialize())
  {
    std::cout<<"Mapping Client fails to initialize";
  }
  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
