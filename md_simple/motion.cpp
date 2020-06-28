#include <motion.hpp>

#include <vector>

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

//	Calculate state vector for t + dt
void motion_step( double dt,
	std::list< spacecraft_t * > scl,
	std::list< gravity_center_t * > masses,
	bool move_gravity_centers=true
) {
	for( auto & sc: scl ) {
		sc->compute_engine_influence();
	}

	std::vector< moving_object_t * > obj_vec;
	std::copy( scl.begin(), scl.end(), obj_vec.end() );
	if( move_gravity_centers ) {
		std::copy( masses.begin(), masses.end(), obj_vec.end() );
	}

	//	First initialize all the forces
	for( auto & obj: obj_vec ) {
		obj->force.setZero();
		obj->momentum.setZero();
	}
	//	Calculate gravity forces
	for( auto & a: obj_vec ) {
		for( auto & b: obj_vec ) {
			vector dr = a->x - b->x;
			double dr2 = dr.dot( dr );
			dr.normalize();
			vector f = Grav * a->mass * b->mass * dr / dr2;
			//	Increment object gravity forces
			a->force += f;
			b->force += f;
		}
	}

	for( auto & obj: obj_vec ) {
		//	Increase coordinates
		obj->x += obj->v * dt;
		//	Increase velocites
		obj->a = obj->force / obj->mass;
		obj->v += obj->a * dt;
	}
}

/*
//	Possibly this function will be used in future for Runge-Kutta solver
vectorX calc_Y( const vectorX & X, auto & obj_vec ) {

	vectorX Y( X.size() );

	//	Calculate gravity forces
	vectorX force = vectorX::Zero( 3 * obj_vec.size() );
	for( size_t i=0; i<obj_vec.size(); ++i ) {
		for( size_t j=0; i<obj_vec.size(); ++j ) {
			vector dr = a->x - b->x;
			double dr2 = dr.dot( dr );
			dr.normalize();
			vector f = Grav * a->mass * b->mass * dr / dr2;
			//	Increment object gravity forces
			a->force += f;
			b->force += f;
		}
	}
}
*/

}	//	namespace SoyuzSim
