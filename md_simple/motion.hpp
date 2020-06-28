#pragma once

#include <list>
#include <vector>
#include <Eigen/Dense>

namespace SoyuzSim {

using vector = Eigen::Vector3d;
using matrix = Eigen::Matrix3d;
using quaternion = Eigen::Quaterniond;

using vectorX = Eigen::VectorXd;

struct moving_object_t {

	double mass = 0.0;
	vector x = vector::Zero();				//	position
	vector v = vector::Zero();				//	velocity
	vector a = vector::Zero();				//	acceleration on last step
	vector force = vector::Zero();			//	result force on last step

	matrix I = matrix::Identity();
	quaternion L = quaternion::Identity();	//	quaternion ICRS to CCRS
	vector omega = vector::Zero();			//	angular velocity
	vector eps = vector::Zero();			//	angular acceleration on last step
	vector N = vector::Zero();				//	result force momentum on last step

	vector driving_force = vector::Zero();		//	non-gravity driving force (engines)
	vector driving_momentum = vector::Zero();	//	non-gravity driving momentum (engines)

};	//	struct moving_object_t

struct engine_t {
	
	vector x = vector::Zero();				//	position in CCRS
	vector force = vector::Zero();			//	force if turned on
	bool is_turned_on = false;

};	//	struct engine_t

struct spacecraft_t: moving_object_t {

	std::vector< engine_t > engine;

	//	Compute sum force and momentum, store to moving_object_t fields
	void compute_engine_influence();

};	//	struct spacecraft_t


struct gravity_center_t: moving_object_t {

	//	TODO: possibly add gravity inhomogenity

};	//	struct gravity_center_t


void motion_step( double dt,
	std::list< spacecraft_t * > scl,
	std::list< gravity_center_t * > obj_list,
	bool move_gravity_centers=true
);

}	//	namespace SoyuzSim
