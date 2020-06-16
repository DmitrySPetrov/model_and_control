#pragma once

#include <list>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace SoyuzSim {

using vector = boost::numeric::ublas::vector< double >;
using zero_vector = boost::numeric::ublas::zero_vector< double >;
using matrix = boost::numeric::ublas::matrix< double >;
using identity_matrix = boost::numeric::ublas::identity_matrix< double >;

struct moving_object_t {

	double mass = 0.0;
	vector x = zero_vector(3);			//	position
	vector v = zero_vector(3);			//	velocity
	vector a = zero_vector(3);			//	acceleration on last step
	vector force = zero_vector(3);		//	result force on last step

	matrix I = identity_matrix(3);
	vector L = zero_vector(4);			//	quaternion ICRS to CCRS
	vector omega = zero_vector(3);		//	angular velocity
	vector eps = zero_vector(3);		//	angular acceleration on last step
	vector momentum = zero_vector(3);	//	result force momentum on last step

	vector driving_force = vector(3);		//	non-gravity driving force (engines)
	vector driving_momentum = vector(3);	//	non-gravity driving momentum (engines)

};	//	struct moving_object_t

struct engine_t {
	
	vector x = zero_vector(3);			//	position in CCRS
	vector force = zero_vector(3);		//	force if turned on
	bool is_turned_on = false;

};	//	struct engine_t

struct spacecraft_t: moving_object_t {

	std::list< engine_t > engine;

	//	Compute sum force and momentum, store to moving_object_t fields
	void compute_engine_influence();

};	//	struct spacecraft_t


struct gravity_center_t: moving_object_t {

	//	TODO: possibly add gravity inhomogenity

};	//	struct gravity_center_t


void motion_step( double dt,
	std::list< spacecraft_t * > scl,
	std::list< gravity_center_t * > obj_list
);

}	//	namespace SoyuzSim
