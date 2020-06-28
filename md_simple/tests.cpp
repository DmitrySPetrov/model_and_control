//	Compile: g++ --std=c++17 -I. test.cpp
#include <motion.hpp>

#include <list>
#include <iostream>
#include <math.h>

#include <UnitTest++/UnitTest++.h>

using namespace SoyuzSim;

//	Free motion (with constant speed)
TEST( free_motion ) {
	double dt = 1.0;
	double t_end = 1000.0 - dt / 2.0;

	spacecraft_t sc1;
	sc1.v = { 1.0, 0.0, 0.0 };
	sc1.engine.push_back( { .x = { 1.0, 0.0, 0.0 }, .force = { 1.0, 0.0, 0.0 } } );
	spacecraft_t sc2;
	sc2.v = { 0.0, -1.0, 0.0 };
	sc2.engine.push_back( { .x = { 0.0, 1.0, 0.0 }, .force = { 0.0, -1.0, 0.0 } } );

	std::list< spacecraft_t * > sc_list = { &sc1, &sc2 };

	std::list< gravity_center_t * > g_list;

	for( double t=0; t<t_end; t+=dt ) {
		motion_step( dt, sc_list, g_list, false );
	}

	vector x1 = { t_end * 1.0, 0.0, 0.0 }, x2 = { 0.0, t_end * -1.0, 0.0 };
	for( size_t i=0; i<3; ++i ) {
		CHECK_CLOSE( sc1.x[i], x1[i], std::abs( dt * sc1.v[i] ) / 2.0 );
		CHECK_CLOSE( sc2.x[i], x2[i], std::abs( dt * sc2.v[i] ) / 2.0 );
	}
}

//	Motion with constant acceleration: an engine constantly works
TEST( accelerated_motion ) {
	double dt = 0.1;
	double t_end = 100.0 - dt / 2.0;

	spacecraft_t sc1;
	sc1.mass = 1.0;
	sc1.engine.push_back( { .x = { 0.0, 0.0, 1.0 }, .force = { 0.0, 0.0, 0.5 } } );
	sc1.engine[0].is_turned_on = true;
	spacecraft_t sc2;
	sc2.mass = 1.0;
	sc2.engine.push_back( { .x = { 0.0, 1.0, 0.0 }, .force = { 0.0, -0.1, 0.0 } } );
	sc2.engine[0].is_turned_on = true;

	std::list< spacecraft_t * > sc_list = { &sc1, &sc2 };
	std::list< gravity_center_t * > g_list;

	for( double t=0; t<t_end; t+=dt ) {
		motion_step( dt, sc_list, g_list, false );
	}

	vector x1 = { 0.0, 0.0, t_end * t_end * 0.5 / 2.0 }, v1 = { 0.0, 0.0, t_end * 0.5 };
	vector x2 = { 0.0, t_end * t_end * -0.1 / 2.0, 0.0 }, v2 = { 0.0, t_end * -0.1, 0.0 };
	for( size_t i=0; i<3; ++i ) {
		CHECK_CLOSE( sc1.x[i], x1[i], std::abs( dt * sc1.v[i] ) / 2.0 );
		CHECK_CLOSE( sc1.v[i], v1[i], std::abs( dt * sc1.a[i] ) );

		CHECK_CLOSE( sc2.x[i], x2[i], std::abs( dt * sc2.v[i] ) / 2.0 );
		CHECK_CLOSE( sc2.v[i], v2[i], std::abs( dt * sc2.a[i] ) );
	}
}

//	Rotate across mass center: radius=1, velocity=1
TEST( simple_orbital ) {
	double dt = 1.0e-5;
	double t_end = 2.0 * M_PI;

	spacecraft_t sc;
	sc.x = { 1.0, 0.0, 0.0 };
	sc.v = { 0.0, 1.0, 0.0 };
	sc.mass = 1.0;
	gravity_center_t g;
	g.mass = 1.0 / 6.67430e-11;	//	1.0 / gravity_constant

	std::list< spacecraft_t * > sc_list = { &sc };
	std::list< gravity_center_t * > g_list = { &g };

	for( double t=0; t<t_end; t+=dt ) {
		motion_step( dt, sc_list, g_list, false );
	}
	vector x = { 1.0, 0.0, 0.0 }, v = { 0.0, 1.0, 0.0 };
	for( size_t i=0; i<3; ++i ) {
		CHECK_CLOSE( sc.x[i], x[i], 3e-3 );
		CHECK_CLOSE( sc.v[i], v[i], 3e-3 );

		CHECK_EQUAL( g.x[i], 0.0 );
		CHECK_EQUAL( g.v[i], 0.0 );
	}
}

//	Rotate two gravity centers across mass center: radius=1, velocity=1
TEST( masses_orbital ) {
	double dt = 1.0e-5;
	double t_end = M_PI;

	gravity_center_t g1;
	g1.x = { 1.0, 0.0, 0.0 };
	g1.v = { 0.0, 1.0, 0.0 };
	g1.mass = 2.0 / 6.67430e-11;
	gravity_center_t g2;
	g2.x = { -1.0, 0.0, 0.0 };
	g2.v = { 0.0, -1.0, 0.0 };
	g2.mass = 2.0 / 6.67430e-11;

	std::list< spacecraft_t * > sc_list;
	std::list< gravity_center_t * > g_list = { &g1, &g2 };

	for( double t=0; t<t_end; t+=dt ) {
		motion_step( dt, sc_list, g_list, true );
	}

	vector x1 = { -1.0, 0.0, 0.0 }, v1 = { 0.0, -1.0, 0.0 };
	vector x2 = { 1.0, 0.0, 0.0 }, v2 = { 0.0, 1.0, 0.0 };
	for( size_t i=0; i<3; ++i ) {
		CHECK_CLOSE( g1.x[i], x1[i], 3e-3 );
		CHECK_CLOSE( g1.v[i], v1[i], 3e-3 );

		CHECK_CLOSE( g2.x[i], x2[i], 3e-3 );
		CHECK_CLOSE( g2.v[i], v2[i], 3e-3 );
	}
}

int main() {
	return UnitTest::RunAllTests();
}
