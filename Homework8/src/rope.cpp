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
        if(num_nodes == 0 || num_nodes == 1)
            return;
        Vector2D CurrentPosition = start;
        Mass *m2 = new Mass(start, node_mass, false);
        masses.push_back(m2);
        Mass *m1 = nullptr;
        Spring *CurrentSpring = nullptr;
        for(int i = 1; i < num_nodes; i++){
            Vector2D CurrentPosition = start + i * (end - start) / (num_nodes - 1);
            m1 = m2;
            m2 = new Mass(CurrentPosition, node_mass, false);
            CurrentSpring = new Spring(m1, m2, k);
            masses.push_back(m2);
            springs.push_back(CurrentSpring);     
        }

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        bool is_explicit = false;
        
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f = s->k * (ab / ab.norm()) * (ab.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces -= f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                // TODO (Part 2): Add global damping
                if (true)
                {
                    float kd_global = 0.01;
                    m->forces += -kd_global * m->velocity;
                }
                auto a = m->forces / m->mass;
                auto v = m->velocity;
                auto v_next = v + a * delta_t;

                if(is_explicit)
                {
                    m->velocity += a * delta_t;
                    m->position += v * delta_t;
                } else
                {
                    m->velocity += a * delta_t;
                    m->position += v_next * delta_t;
                }

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            if (s->k != -1) 
            {
                // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
                Vector2D ab = s->m2->position - s->m1->position;
                Vector2D f = s->k * (ab / ab.norm()) * (ab.norm() - s->rest_length);
                s->m1->forces += f;
                s->m2->forces -= f;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                auto a = m->forces / m->mass;
                
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D x_t0 = m->last_position;
                Vector2D x_t1 = m->position;

                // TODO (Part 4): Add global Verlet damping
                float damping = 0.00005;
                Vector2D x_t2 = x_t1 + (1 - damping) * (x_t1 - x_t0) + a * delta_t * delta_t;

                m->last_position = x_t1;
                m->position = x_t2;

            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);        
        }

        for (auto &s:springs){
                if (s->k != -1) 
                    continue;
                // k = -1 means infinite
                auto d = s->m2->position - s->m1->position;
                auto dir = d / d.norm();
                auto len = s->rest_length;
                auto offset1 = 0.5 * dir * (d.norm() - len);
                auto offset2 = -0.5 * dir * (d.norm() - len);
                
                if (s->m1->pinned && s->m2->pinned) 
                    continue;

                if (s->m1->pinned)
                {
                    offset2 *= 2;
                    offset1 = Vector2D(0, 0);
                } 

                if (s->m2->pinned)
                {
                    offset1 *= 2;
                    offset2 = Vector2D(0, 0);
                } 

                s->m1->position += offset1;
                s->m2->position += offset2;
            }
    }
}
