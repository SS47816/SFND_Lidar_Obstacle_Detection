#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 
#include <vector>

struct BoxQ
{
	int id;
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};

struct Box
{
  public:
	int id;
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

#endif