/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#ifndef _CS251BASE_HPP_
#define _CS251BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>
#include <iostream>
#include <time.h>
 using namespace std;
#define	RAND_LIMIT 32767

namespace cs251
{

  /*! \class base_sim_t 
      \brief This is the main class which acts as the global class for every program.
  */ 
  class base_sim_t;
  struct settings_t;
  
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t
  {

    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
    b2Vec2 view_center;
    float32 hz;
    int32 velocity_iterations;
    int32 position_iterations;
    int32 draw_shapes;
    int32 draw_joints;
    int32 draw_AABBs;
    int32 draw_pairs;
    int32 draw_contact_points;
    int32 draw_contact_normals;
    int32 draw_contact_forces;
    int32 draw_friction_forces;
    int32 draw_COMs;
    int32 draw_stats;
    int32 draw_profile;
    int32 enable_warm_starting;
    int32 enable_continuous;
    int32 enable_sub_stepping;
    int32 pause;
    int32 single_step;
  };
  
  struct sim_t								
  {
    const char *name;							//
    sim_create_fcn *create_fcn;						//

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;			//definition is in dominos.cpp
  
  
  const int32 k_max_contact_points = 2048;  
  struct contact_point_t
  {
    b2Fixture* fixtureA;
    b2Fixture* fixtureB;
    b2Vec2 normal;
    b2Vec2 position;
    b2PointState state;
  };
  
  class base_sim_t : public b2ContactListener
  {
  public:
    int numb;
    b2World* m_world;     ///Represents the Box2D world
    bool m_contact;       /// Boolean variable used in BeginContact function
    b2Body* mb1;          ///Box2D body which is being manipulated in the Contact Listener
    bool s_contact;       /// Boolean variable used in BeginContact function
    b2Body* ground2;      ///Box2D body which is being manipulated in the Contact Listener
    b2Body* ground5;      ///Box2D body which is being manipulated in the Contact Listener
    b2Body* mb2;          ///Box2D body which is being manipulated in the Contact Listener
    b2Body* mb4;          ///Box2D body which is being manipulated in the Contact Listener
    b2Body* sbody1;          ///Box2D body which is being manipulated in the Contact Listener
    b2Body* ba;           ///Box2D body which is being manipulated in the Contact Listener
    //b2BodyDef bad1;
    clock_t clk;          ///clock_t object for measuring time elapsed, used in step function
    bool clock_tart;      ///Boolean variable used to check whether the above clock has been initialised
    base_sim_t();   

    //! Virtual destructors - amazing objects. Why are these necessary?
    virtual ~base_sim_t();
    
    void set_text_line(int32 line) { m_text_line = line; }
    void draw_title(int x, int y, const char *string);
    
    virtual void step(settings_t* settings);   				//step the simulation

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    /*! \fn BeginContact
        \brief The overridden BeginContact function which gets executed whenever there is a contact between any two objects.
    */
    virtual void BeginContact(b2Contact* contact) { 
        if (contact->GetFixtureA()->GetBody()->GetUserData() != NULL && contact->GetFixtureB()->GetBody()->GetUserData() !=NULL)
        {
            s_contact=true;
        }
        
        else if (contact->GetFixtureA()->GetBody()->GetUserData() == &numb || contact->GetFixtureB()->GetBody()->GetUserData() == &numb){
            m_contact = true;
            //cout<<"Hello World"<<endl;
          }
        // else if (contact->GetFixtureB()->GetBody()->GetUserData() != NULL){
        //   m_contact = true;
        //   cout<<"Hello World"<<endl;
        // }
        else
          m_contact = false;
    }


    virtual void end_contact(b2Contact* contact) { }//B2_NOT_USED(contact); }
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      //B2_NOT_USED(contact);
      //B2_NOT_USED(impulse);
    }

  protected:

    friend class contact_listener_t;
    
    b2Body* m_ground_body;
    b2AABB m_world_AABB;
    contact_point_t m_points[k_max_contact_points];
    int32 m_point_count;

    debug_draw_t m_debug_draw;
    int32 m_text_line;
    

    int32 m_step_count;
    
    b2Profile m_max_profile;
    b2Profile m_total_profile;
  };
}

#endif
