#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        for (int index=0;index<num_nodes;index++) {
            float a = (float)index/(float)(num_nodes-1);
            // Mass mmm = Mass((1-a)*start+a*end, node_mass, false);
            // masses[index]=&mmm;
            masses.push_back(new Mass((1-a)*start+a*end, node_mass, false));
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for (int index=0;index<num_nodes-1;index++) {
            // Spring sss = Spring(masses[index],masses[index+1],k);
            // springs[index]=&sss;
            springs.push_back(new Spring(masses[index],masses[index+1],k));
        }
    };

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto a = s->m1->position;
            auto b = s->m2->position;
            auto k = s->k;
            auto v = a-b;
            auto v_norm = v.norm();
            s->m1->forces-=k*(v/v_norm)*(v_norm-s->rest_length);
            s->m2->forces+=k*(v/v_norm)*(v_norm-s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+=gravity*m->mass;
                //m->last_position=m->position;
                m->velocity+=m->forces/m->mass*delta_t;
                m->position+=m->velocity*delta_t;
                // TODO (Part 2): Add global damping
                m->velocity*=(1-1e-5);//damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto a = s->m1->position;
            auto b = s->m2->position;
            auto k = s->k;
            auto v = a-b;
            auto v_norm = v.norm();
            s->m1->forces-=k*(v/v_norm)*(v_norm-s->rest_length);
            s->m2->forces+=k*(v/v_norm)*(v_norm-s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces+=gravity*m->mass;
                m->position+=(1-1e-5)*(m->position-m->last_position)+m->forces/m->mass*delta_t*delta_t;
                //m->position+=m->position-m->last_position+m->forces/m->mass*delta_t*delta_t;
                // TODO (Part 4): Add global Verlet damping
                //m->position-=1e-5*(m->position-m->last_position);

                m->last_position=temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
