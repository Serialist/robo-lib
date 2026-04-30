/**
 * @file sim-module.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief physics module simulator 
 * @version 0.1.0
 * @date 2026-04-23
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SIM_MODULE_HPP
#define SIM_MODULE_HPP

namespace pt
{

class SecondOrderSystem
{
	// state variable
	double x = 0.0; // position
	double v = 0.0; // velocity

	// sys param
	double m;  // mass
	double kf; // damping

public:
	// constructor
	SecondOrderSystem(double mass, double damping);

	// get state
	double position();

	// update system
	void step(double u, double dt);

	// reset sys
	void reset();
};

} // namespace pt

#endif
