#include <motion.hpp>

const double Grav = 6.67430e-11;

namespace SoyuzSim {

void spacecraft_t::compute_engine_influence() {

	this->driving_force.setZero();
	this->driving_momentum.setZero();

	for( auto & e: engine ) {
		if( e.is_turned_on ) {
			//	Calculate force
			this->driving_force += e.force;
			//	Calculate momentum: cross product of vector and force
			vector dr = e.x - this->x;
			this->driving_momentum += dr.cross(e.force);
		}
	}
}

void motion_step( double dt,
	std::list< spacecraft_t * > scl,
	std::list< gravity_center_t * > masses,
	bool move_gravity_centers=true
) {
	for( auto & sc: scl ) {
		sc->compute_engine_influence();
	}

	std::list< moving_object_t * > obj_list;
	std::copy( scl.begin(), scl.end(), obj_list.end() );
	if( move_gravity_centers ) {
		std::copy( masses.begin(), masses.end(), obj_list.end() );
	}

	//	First initialize all the forces
	for( auto & obj: obj_list ) {
		obj->force.setZero();
		obj->momentum.setZero();
	}
	//	Calculate gravity forces
	for( auto & a: obj_list ) {
		for( auto & b: obj_list ) {
			vector dr = a->x - b->x;
			double dr2 = dr.dot( dr );
			dr.normalize();
			vector f = Grav * a->mass * b->mass * dr / dr2;
		}
	}
}

}	//	namespace SoyuzSim
