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

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_

namespace cs251
{
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
    
    dominos_t();
    
    static base_sim_t* create()       //returns a new dominos_t (pointer to base class) object to sim_t
    {				      
      return new dominos_t;
    }
    
	b2MouseJoint* mouse_joint =NULL;
	void mouse_down(const b2Vec2&p);
	void mouse_up(const b2Vec2&p);
	void mouse_move(const b2Vec2&p);
	//void begin_contact(b2Contact* contact);
        //void end_contact(b2Contact* contact);
        //void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
        //void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)

b2Body* sbody;    
 b2Body* b1;

/*
    void mouse_up(const b2Vec2& p)
	{
	mouse_pressed = false;
	if (mouse_joint)
		{
		world.DestroyJoint(mouse_joint);
		mouse_joint = false;

		}  
		
	}*/

  };
}
  
#endif
