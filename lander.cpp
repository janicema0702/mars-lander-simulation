// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <fstream>

bool integration_method = 1; // 0 == EULER, 1 == VERLET
void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
  // ignore parachute and altitude control (assum altitude control is always on and parachute is not deployed)
{
    // INSERT YOUR CODE
    double K_h, K_p, h, e, P;
    vector3d e_r = position.norm();

    //scenario 1 (0.02, 0.8)
    K_h = 0.02;
    K_p = 0.8;
    h = position.abs() - MARS_RADIUS;

    e = -(0.5 + K_h * h + velocity * e_r);
    P = K_p * e;

    //thrust needed to balance mass
    vector3d drag;
    double density = atmospheric_density(position);
    drag = -density * DRAG_COEF_LANDER * M_PI * pow(LANDER_SIZE, 2) * velocity.abs2() * velocity.norm() / 2;
    double mass = UNLOADED_LANDER_MASS + FUEL_DENSITY * FUEL_CAPACITY * fuel;

    // thrust/ max thrust (ratio)
    double delta = ((GRAVITY * MARS_MASS * mass / position.abs2()) + (drag.abs())) / MAX_THRUST;

    if (P <= -delta) {
        throttle = 0;
    }
    else if (-P < 1 - delta) {
        throttle = delta + P;
    }
    else {
        throttle = 1;
   
    }

    // Write the trajectories to file
    std::ofstream fout;
    fout.open("ap.txt", std::ios_base::app);
    if (fout) { // file opened successfully 
        fout << h << ' ' << K_h * h << ' ' << velocity * e_r << endl;
    }
    else {// file did not open successfully
        cout << "Could not open autopilot trajectories file for writing" << endl;
    }

}

vector3d getacceleration(vector3d gravitational_acceleration)
{
    // from Constants
    double mass = UNLOADED_LANDER_MASS + FUEL_DENSITY * FUEL_CAPACITY * fuel;

    //thrust force from predefined function
    vector3d thrust = thrust_wrt_world(); 
    
    // aerodynamic drag force
    vector3d drag;
    double density = atmospheric_density(position);
    drag = -density * DRAG_COEF_LANDER * M_PI * pow(LANDER_SIZE, 2) * velocity.abs2()* velocity.norm()/2;

    if (parachute_status == DEPLOYED) {
        drag += -density * DRAG_COEF_LANDER * 5 * 2.0 * pow(LANDER_SIZE*2.0, 2) * velocity.abs2() * velocity.norm()/2;

    }
    // gravitational force
    vector3d total_acceleration = gravitational_acceleration + (thrust / mass) + (drag / mass);
    return total_acceleration;


}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE
    vector3d acceleration;
    vector3d gravitational_acceleration;
    //static vector3d position0;
    vector3d position1;
    vector3d position2;

    

    if (integration_method == 0) {
        // Euler integration
            // calculate new position and velocity
        gravitational_acceleration = -GRAVITY * MARS_MASS * position.norm() / (pow(position.abs(), 2));
        acceleration = getacceleration(gravitational_acceleration);
        position = position + (velocity * delta_t);
        velocity = velocity + (acceleration * delta_t);
    }

    else if (integration_method == 1) {

        // Verlet integration
            // calculate new position and velocity
        
        \

        if (simulation_time == 0.0) {
            position1 = position + (delta_t * velocity);

        }

        else {
            //position1 = position0;
            position1 = position + (delta_t * velocity);
            gravitational_acceleration = -GRAVITY * MARS_MASS * position.norm() / (pow(position.abs(), 2));
            acceleration = getacceleration(gravitational_acceleration);
            position2 = 2*position1 - position + acceleration * pow(delta_t, 2);
            velocity = (position2 - position1)/ (delta_t);
            position = position1;
            position1 = position2;
        }
        

    }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}