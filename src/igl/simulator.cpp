#include "simulator.h"

#include "intersect.h"

///@file igl/simulator.cpp Simulation. @ingroup igl

/// Updates the simulated data by stepping over the time delta given simulator->steps_per_sec
/// @param simulator Contains data to update and functions to perform the update
/// @param dt The number of seconds to advance the simulator (time delta)
void simulator_update(ParticleSimulator* simulator, float dt) {
    int steps = round(simulator->steps_per_sec * dt);
    float ddt = dt / steps;
    simulator->begin_update(dt);
    for(int i = 0; i < steps; i ++) {
        simulator->begin_step(ddt);
        simulator_update_step(simulator,ddt);
        simulator->end_step(ddt);
    }
    simulator->end_update(dt);
}

/// Updates the simulated data in one step that covers dt seconds by:
/// 1) computing outside forces (ex: gravity),
/// 2) applying internal constraints (ex: spring forces),
/// 3) performing Euler integration,
/// 4) handling collisions, and
/// 5) updating timers.
/// @param simulator Contains data to update and functions to perform the update
/// @param dt The number of seconds to advance the simulator (time delta)
void simulator_update_step(ParticleSimulator* simulator, float dt) {

    // Initially set forces to 0
    for(int i = 0; i < simulator->particles.size(); i++)
        simulator->particles[i]._force = zero3f;

    // Apply internal constraints (for cloth simulation)
    // From graphics.ucsd.edu/courses/cse169_w05/CSE169_16.ppt‎
    for(int i = 0; i < simulator->springs.size(); i++) {
        auto pi = simulator->springs[i].i;
        auto pj = simulator->springs[i].j;
        auto pipj = simulator->particles[pi].pos - simulator->particles[pj].pos;
        auto l = length(pipj);
        auto e = pipj / l;
        auto v1 = dot(e, simulator->particles[pi].vel);
        auto v2 = dot(e, simulator->particles[pj].vel);
        auto fsd = -simulator->springs[i].ks * (l - simulator->springs[i].l) - simulator->springs[i].kd * (v1 - v2);
        auto f1 = e * fsd;
        auto f2 = -f1;
        simulator->particles[pi]._force += f1;
        simulator->particles[pj]._force += f2;
    }

    // Perform Euler integration; slightly modified to ensure stability of calculation
    for(int i = 0; i < simulator->particles.size(); i++) {
        simulator->particles[i]._force += simulator->force(simulator->particles[i]);

        if(simulator->particles[i].pinned == false) {
            auto a = simulator->particles[i]._force/simulator->particles[i].mass;
            simulator->particles[i].vel += a * dt;
            simulator->particles[i].pos += simulator->particles[i].vel * dt + (a * dt * dt)/2;
        }

        // Update timers
        simulator->particles[i].timer -= dt;
    }
}

