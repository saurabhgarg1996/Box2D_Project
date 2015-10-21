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

#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{

  b2DistanceJointDef* dj1;
  b2DistanceJointDef* dj2;
  /**  This is the function which take in the x- co-ordinate and three parameters
   *   of a catenary curve and returns the y- co-ordinate of the point on catenary curve.
   */ 
  
  float catenary_equation(float x,float a,float alpha,float beta)
  {
    return beta+(a*(1-cosh((x-alpha)/a)));
  }
  /**  This is the function which takes a vector which is a starting point of the catenary curve,
   *  number of points in the catenary curve,a parameter, and returns the array of vector representing
   *  all the points in the catenary curve
   */ 
  b2Vec2* catenary(b2Vec2 v[],b2Vec2 w,float a,int num)
  {
    float alpha=a*asinh(a/2);
    float beta=-1*a*(1-cosh(asinh(a/2)));
    float x=w.x;float y=w.y;
    float dx=2*alpha/num;
    for(int i=0;i<num;i++)
      {

        x=x+dx;
        y=float(catenary_equation(x-w.x,a,alpha,beta));
	v[i].Set(x,y+w.y);      
	}
    return v;
  }
   /**  This is the function which take in the x- co-ordinate and two parameters of the upper arc
   *   of the circle and returns the y- co-ordinate of the point on arc.
   */
  float arc_equation_upper(float x,float m,float r)
  {
    float d;
    d=(2*r)/sqrt(1+m*m);
    return((sqrt(d*d-4*(x*x-m*d*x))))/2;

  }
  /**  This is the function which take in the x- co-ordinate and two parameters of the lower arc
   *   of the circle and returns the y- co-ordinate of the point on arc.
   */
  float arc_equation_lower(float x,float m,float r)
  {
    float d;
    d=(2*r)/sqrt(1+m*m);
    float result=(-(sqrt(d*d-4*(x*x-m*d*x))))/2;
    return result;

  }
   /**  This is the function which takes a, vector which is a starting point of the upper arc,
   *  number of points in the catenary curve,a two parameters, a boolean (indicate whether to 
   *  start the curve at starting point or a little away from it) and returns the array of vectors representing
   *  all the points in the upper arc
   */
  b2Vec2* arc_upper(b2Vec2 v[],b2Vec2 w,int num,float m,float r,bool bo)
  {
    if(!bo)
    {
    float d=r*(1-(m/sqrt(1+m*m)));
    float e=r/sqrt(1+m*m);
    float dx=(d-1.0f)/num;
    float x=w.x-d;float y=w.y;
    for(int i=0;i<num;i++)
      {
        v[i].Set(x,y+e);
        x=x+dx;
        y=w.y+float(arc_equation_upper(x-w.x,m,r));
      }
    return v;
    }
    else
    {
    float d=r*(1-(m/sqrt(1+m*m)));
    float e=r/sqrt(1+m*m);
    float dx=d/num;
    float x=w.x-d;float y=w.y;
    for(int i=0;i<num;i++)
      {
        v[i].Set(x,y+e);
        x=x+dx;
        y=w.y+float(arc_equation_upper(x-w.x,m,r));
      }
    return v;
    }
  }
   /**  This is the function which takes a, vector which is a starting point of the lower arc,
   *  number of points in the catenary curve,a two parameters, a boolean (indicate whether to 
   *  start the curve at starting point or a little away from it) and returns the array of vectors representing
   *  all the points in the lower arc
   */ 
  b2Vec2* arc_lower(b2Vec2 v[],b2Vec2 w,int num,float m,float r,bool bo)
  {
    if(!bo)
    {
    float d=r*(1-(m/sqrt(1+m*m)));
    float e=r/sqrt(1+m*m);
    float dx=(d-1.0f)/num;
    float x=w.x-d;float y=w.y;
    for(int i=0;i<num;i++)
      {

        v[i].Set(x,y+e);
        x=x+dx;
        y=w.y+float(arc_equation_lower(x-w.x,m,r));
      }
    return v;
    }
    else
    {
    float d=r*(1-(m/sqrt(1+m*m)));
    float e=r/sqrt(1+m*m);
    float dx=d/num;
    float x=w.x-d;float y=w.y;
    for(int i=0;i<num;i++)
      {
 
        v[i].Set(x,y+e);
        x=x+dx;
       	      y=w.y+float(arc_equation_lower(x-w.x,m,r));
      }
    return v;
    }
  }
   /**  This is the function which take in the x- co-ordinate and two parameters of the
   *   helical curve and returns the y- co-ordinate of the point on arc.
   */
  float helix_equation(float x,float a,float b)
  {
      float val=(a-x)/(b+x);
      if(x>=0.0f)
        return x*sqrt(val);
      else
        return -x*sqrt(val);
  }
  /**  This is the function which takes a, vector which is a starting point of upper part of helical curve,
   *  number of points in the upper helical curve,a two parameters, a boolean (indicate whether to 
   *  start the curve at starting point or a little away from it) and returns the array of vectors representing
   *  all the points in the upper helical curve.
   */
b2Vec2* helix_inv(b2Vec2 v[],b2Vec2 w,int num,float a,float b,bool bo)
  {

    if(!bo)
    {
    float dx=(a-1.0f)/(num-10);
    float x=w.x+1.0f;float y=0;
    for(int i=0;i<(num-10);i++)
      {
      	y=float(helix_equation(x-w.x,a,b));
        v[i].Set(x,w.y-y);
        x=x+dx;


      }
          for(int i=num-10;i<num;i++)
    {
    	v[i].Set(x,w.y+float(i)/199.90f);
    }
    return v;
    }
    else
    {
    float dx=a/(num-10);
    float x=w.x;float y=0;
    for(int i=0;i<(num-10);i++)
      {
        v[i].Set(x,w.y-y);
        x=x+dx;
        y=float(helix_equation(x-w.x,a,b));

      }
    for(int i=num-10;i<num;i++)
    {
    	v[i].Set(x,w.y+float(i)/199.90f);
    }
    return v;
    }
  }
   /**  This is the function which takes a, vector which is a starting point of lower part of helical curve,
   *  number of points in the lower helical curve,a two parameters, a boolean (indicate whether to 
   *  start the curve at starting point or a little away from it) and returns the array of vectors representing
   *  all the points in the lower helical curve.
   */
    b2Vec2* helix(b2Vec2 v[],b2Vec2 w,int num,float a,float b,bool bo)
  {
    if(!bo)
    {
    float dx=(a-1.0f)/num;
    float x=w.x+1.0f ;float y=w.y;
    for(int i=0;i<num;i++)
      {
      	        y=w.y+float(helix_equation(x-w.x,a,b));

        v[i].Set(x,y);
        x=x+dx;
      }
    return v;
    }
    else
    {
    float dx=a/num;
    float x=w.x ;float y=w.y;
    for(int i=0;i<num;i++)
      {
        v[i].Set(x,y);
        x=x+dx;
        y=w.y+float(helix_equation(x-w.x,a,b));
      }
    return v;
    }
  }
  /** This is a function which takes the 3 parameters of the complete arc, no of points in the arc, 
  *   starting point of both of its constituting arcs,a  boolean to manipulate the starting point of the curve
  *   ,and a b2World object, and creates the arc in the real world.
  */
void arc_create(b2Vec2 w,float a,float b,float r,bool bo,int num,b2World* world)
    {

        float m = 0.0f;
        if(a>=0&&b>=0)
        	m=sqrt(a/b);
        else if(a<0)
        	m=-(sqrt((-a)/b));
        b2Body* b2;
        b2Body* b3;
        b2Vec2 v_1[num];
        b2Vec2 v_2[num];
        {
        b2ChainShape chain_upper;
        b2ChainShape chain_lower;
        arc_upper(v_1,w,num,m,r,bo);
        arc_lower(v_2,w,num,m,r,bo);
        chain_upper.CreateChain(v_1,num);
        chain_lower.CreateChain(v_2,num);
        b2BodyDef bd_upper;
        b2BodyDef bd_lower;
        b2 = world->CreateBody(&bd_upper);
        b2->CreateFixture(&chain_upper, 0.0f);
        b3 = world->CreateBody(&bd_lower);
        b3->CreateFixture(&chain_lower, 0.0f);
        }
    }
 /** This is a function which takes the 2 parameters of the complete helical path, no of points in the curve, 
  *   starting point of both of its constituting helical curves,a  boolean to manipulate the starting point of 
  *  the curve ,and a b2World object, and the fuction creates the helical curve in the real world.
  */
void helix_create(b2Vec2 w,float a,float b,bool bo,int num,b2World* world)
    {

        b2Body* b2;
        b2Body* b3;
        b2Vec2 v_1[num];
        b2Vec2 v_2[num];
        {
        b2ChainShape chain_upper;
        b2ChainShape chain_lower;
        helix(v_1,w,num,a,b,bo);
        helix_inv(v_2,w,num,a,b,bo);
        chain_upper.CreateChain(v_1,num);
        chain_lower.CreateChain(v_2,num);
        b2BodyDef bd_upper;
        b2BodyDef bd_lower;
        b2 = world->CreateBody(&bd_upper);
        b2->CreateFixture(&chain_upper, 0.0f);
       b3 = world->CreateBody(&bd_lower);
        b3->CreateFixture(&chain_lower, 0.0f);

        }
    }
  /** This is a function which takes the 2 parameters of the complete helical path,
  *   parameters of the complete circular arc,no of points in both the curves, 
  *   starting point of both of the helical curves and both of the circular arcs ,
  *   2 booleans to manipulate the starting point of the helical curve and the circular arc,
  *   number of loops in the spiral loop, a b2World object,and creates the spiral curve  in the real world.
  */
void spiral_create(b2Vec2 w,float arc_a,float arc_b,float arc_r,bool arc_bool,float helix_a,float helix_b,bool helix_bool,int num,int num_loops,b2World* world)
{
    float r=arc_r;
    float m=sqrt(arc_a/arc_b);
    for(int i=0;i<num_loops;i++)
    {
       helix_create(w,helix_a,helix_b,helix_bool,num,world);
        arc_create(w,arc_a,arc_b,arc_r,arc_bool,num,world);
        w.y-=2*r/sqrt(1+m*m);
    }
            arc_create(w,arc_a,arc_b,arc_r,arc_bool,num,world);

}

  dominos_t::dominos_t()
  {
  	 
	//Ground
    ///var b1 (type b2body*) -> brief pointer to the body ground initialized in dominos.hpp 
    /// var bd (type b2BodyDef) -> 
    {
     	/// var w (type b2Vec2)-> 
     	/// var chain (type b2ChainShape) -> 
        b2Vec2 w;
	    int num=100;
	    b2Vec2 v[num];
	    w.Set(-40.0f,0.0f);
	    for(int i=0;i<15;i++)			// Creation of Catenary Curve
	      {
	       float size=2.5f;
	       b2ChainShape chain;
	       catenary(v,w,size,num);
	       chain.CreateChain(v,num);
	       b2BodyDef bd;
	       b1 = m_world->CreateBody(&bd);
	       b1->CreateFixture(&chain, 0.0f);
	       w=v[num-1];
	       }
	w.x+=5.23f;
   for(int i=0;i<4;i++)
      {
       float size=2.5f;
       b2ChainShape chain;
       catenary(v,w,size,num);
       chain.CreateChain(v,num);
       b2BodyDef bd;
       b1 = m_world->CreateBody(&bd);
       b1->CreateFixture(&chain, 0.0f);
       w=v[num-1];
       }
	    w.Set(-44.0f,30.0f);
	    spiral_create(w,5.0,5.0f,6.0f,true,11.0f,5.78f,false,200,2,m_world);
	    w.Set(-42.0f,30.0f);
	    spiral_create(w,5.0,5.0f,6.0f,false,7.5f,7.5f,true,200,2,m_world);

    }
          
    ///Top horizontal plank on which the dominos are kept 
    {
    	
      /**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(17.75f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(5.0f, 23.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    ///Plank onto which spring-mass system is kept at rest
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(2.8f, 0.25f);
  
      b2BodyDef bd;
      bd.position.Set(-5.8f, 32.3f);
      bd.angle = -0.00;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    /// A small peg on which ball which hits cycle is kept 
    {/***************     */
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);
  
      b2BodyDef bd;
      bd.position.Set(20.0f, 42.3f);
      bd.angle = -0.00;
      ground2 = m_world->CreateBody(&bd);
      ground2->CreateFixture(&shape, 0.0f);
      ground2->SetUserData(this);
    }
    {/***************     */
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);
      b2Body* ground3;
      b2BodyDef bd;
      bd.position.Set(20.5f, 42.3f);
      bd.angle = -0.00;
      ground3 = m_world->CreateBody(&bd);
      ground3->CreateFixture(&shape, 0.0f);
      //ground3->SetUserData(this);
    }
    /// Vertical plank that avoids high elasticity ball to come out of that bounded region
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(0.2f, 1.3f);
  
      b2BodyDef bd;
      bd.position.Set(-10.0f, 36.5f);
       bd.angle = -0.00;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    /// Linear part of groove 
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(4.54f, 0.20f);
  
      b2BodyDef bd;
      bd.position.Set(25.0f, 45.0f);
       bd.angle = 0.2;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    /// Angular plank that carries ball to rotating plus
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(18.5f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(8.0f, 40.0f); 
      bd.angle = 0.1;
      ground5 = m_world->CreateBody(&bd);
      ground5->CreateFixture(&shape, 0.0f);
      ground5->SetUserData(this);
    } 
    // Funnel at the mouth of spiral
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
    {	
      b2PolygonShape shape;
      shape.SetAsBox(8.5f, 0.1f);
	
      b2BodyDef bd;
      bd.position.Set(-34.5f, 39.0f); 
      bd.angle = 0.2;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    {
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.1f);
	
      b2BodyDef bd;
      bd.position.Set(-41.5f, 45.5f); 
      bd.angle = 1.2;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    } 
	}
    ///Another Top horizontal shelf on which spring-mass system slides down
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;
      shape.SetAsBox(16.5f, 0.25f);
  
      b2BodyDef bd;
      bd.position.Set(7.0f, 30.5f);
      bd.angle = -0.1;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    ///Dominos
    {
    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
    	/// var fd ->  It is a b2FixtureDef with details for fixtures of dominos 
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 1000.0f;				//Relative high mass to make plank rotate
      fd.friction = 0.5f;
		
  for (int i = 0; i < 30; ++i)
	{
	  b2BodyDef bd;	
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-9.0f + 1.0f * i, 24.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
	 for (int i = 0; i < 30; ++i)
	{
	  b2BodyDef bd1;	
	  bd1.type = b2_dynamicBody;
	  bd1.position.Set(-9.0f + 1.0f * i, 26.25f);
	  b2Body* body = m_world->CreateBody(&bd1);
	  body->CreateFixture(&fd);
	}
    }
	///The Vertical Plank which revolves on hit by dominos.
	{
    	/**var bd2 ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;                                                                         
      shape.SetAsBox(0.25f, 5.0f);                           
      b2BodyDef bd;
      bd.position.Set(-10.0f, 30.0f);                       
      bd.type = b2_dynamicBody;                           
      b2Body* body = m_world->CreateBody(&bd);            
      b2FixtureDef *fd = new b2FixtureDef;                
      fd->density =35.0f;                                  
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);                            
      
      /// Creating an invisible hinge to rotate the platform: Static object.
      b2PolygonShape shape2;                              
      shape2.SetAsBox(0.2f, 1.0f);                       
      b2BodyDef bd2;
      bd2.position.Set(-10.0, 30.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);           
        
      /// Revolute joint between the plank and the hinge.    
      b2RevoluteJointDef jointDef;                         
      jointDef.bodyA = body;                               
      jointDef.bodyB = body2;   
      jointDef.localAnchorA.Set(0,2.0);                    
      jointDef.localAnchorB.Set(0,2.0);
      jointDef.collideConnected = false;                   
      m_world->CreateJoint(&jointDef); 
    } 

	///The Horizontal Plank to initiate Dominos movement which revolves on hit by ballon.
    {

    	/**var bd2 ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape;                                                                       
      shape.SetAsBox(2.0f, 0.25f);                             
      b2BodyDef bd;
      bd.position.Set(22.75f, 23.75f);                       
      bd.type = b2_dynamicBody;                           
      b2Body* body = m_world->CreateBody(&bd);            
      b2FixtureDef *fd = new b2FixtureDef;                
      fd->density = 20.0f;                                  
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);                            
      	
      /// Creating an invisible hinge to rotate the platform: Static object.
    	/**var body2 ->  It is a static b2Body with a rectangular shape **/ 
      b2PolygonShape shape2;                              
      shape2.SetAsBox(0.2f, 0.5f);                        
      b2BodyDef bd2;
      bd2.position.Set(22.75, 23.75f);
      b2Body* body2 = m_world->CreateBody(&bd2);
        
      /// Revolute joint between the plank and the hinge.    
      b2RevoluteJointDef jointDef;                         
      jointDef.bodyA = body;                            
      jointDef.bodyB = body2;   
      jointDef.localAnchorA.Set(-2.5f,0);                     
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;                   
      m_world->CreateJoint(&jointDef); 
}	 
	///Right arc which transfers energy from spring-mass system to ball which initiates cycle
	 {
	 	b2Vec2 w;
		w.Set(30.0f,29.0f);
	 	int num=100;
	 	float a=0.0f;
	 	float b=10.0f;
	 	float r=8.7f;
        float m;
        if(a>=0&&b>=0)
        	m=sqrt(a/b);
        else if(a<0)
        	m=-(sqrt((-a)/b));
        b2Body* b2;
        b2Body* b3;
        b2Vec2 v_1[num];
        b2Vec2 v_2[num];
        {
        b2ChainShape chain_upper;
        b2ChainShape chain_lower;

	    float d=r*(1-(m/sqrt(1+m*m)));
	    float e=r/sqrt(1+m*m);
	    float dx=d/num;
	    float x=w.x-d;float y=w.y;
	    for(int i=0;i<num;i++)
	      {
	        x=x+dx;
	        y=w.y+float(arc_equation_upper(x-w.x+r,m,r));
	        	        v_1[i].Set(x,y+e);

	      }

	    d=r*(1-(m/sqrt(1+m*m)));
	    e=r/sqrt(1+m*m);
	    dx=d/num;
	    x=w.x-d;
	    y=w.y;
	    for(int i=0;i<num;i++)
      {
 
        x=x+dx;
       	      y=w.y+float(arc_equation_lower(x-w.x+r,m,r));
       	              v_2[i].Set(x,y+e);

      }
        chain_upper.CreateChain(v_1,num);
        chain_lower.CreateChain(v_2,num);
        b2BodyDef bd_upper;
        b2BodyDef bd_lower;
        b2 = m_world->CreateBody(&bd_upper);
        b2->CreateFixture(&chain_upper, 0.0f);
        b3 = m_world->CreateBody(&bd_lower);
        b3->CreateFixture(&chain_lower, 0.0f);
        }
	 }
     ///Spring mass system that is kept rest on plank
    {
    	/**var body ->  It is a static b2Body with a circular shape **/ 
      b2CircleShape circle;
      circle.m_radius = 0.8;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 500.0f;				// High Density Sphere
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.9f;			// Ball ahead in spring mass system with high elastic coefficient 
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-3.05f, 33.25f);
      b2Body* body = m_world->CreateBody(&ballbd);
      body->CreateFixture(&ballfd);

      b2CircleShape circle1;
      circle1.m_radius = 0.8;
      b2FixtureDef ballfd1;
      ballfd1.shape = &circle1;
      ballfd1.density = 500.0f;
      ballfd1.friction = 0.00f;
      ballfd1.restitution = 0.0f;
      b2BodyDef ballbd1;
      ballbd1.type = b2_dynamicBody;     
      ballbd1.position.Set(-8.5f, 33.25f);
      b2Body* body2 = m_world->CreateBody(&ballbd1);    
      body2->CreateFixture(&ballfd1); 

      /// PrismaticJoint to mimic spring mass system joint
      /// var jointdef -> Prismatic Joint between two spheres 
      b2PrismaticJointDef jointDef;
	  b2Vec2 worldAxis(2.5f, 0.0f);
	  jointDef.Initialize(body, body2, body->GetWorldCenter(), worldAxis);
	  jointDef.lowerTranslation = -1.5f;
      jointDef.upperTranslation = 1.5f;
	  jointDef.enableLimit = true;
	  jointDef.maxMotorForce =  0.0f;
	  jointDef.motorSpeed = 0.0f;
	  jointDef.enableMotor = true;
	  m_world->CreateJoint(&jointDef);

    } 

    ///The pulley system which makes balloon to fly off
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-11,15);
      bd->fixedRotation = true;
      
      ///The open box in which balls (which mimics water) fall
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 50.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 50.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 50.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      /// The bar that revolves to free balloon which then moves upward to initiate dominos

    	/**var bd ->  It is a static b2Body with a rectangular shape **/ 
      bd->position.Set(10,15);	
      fd1->density = 400.2;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint between open box and horizontal plank
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-11, 15); 
      b2Vec2 worldAnchorOnBody2(10, 15); 
      b2Vec2 worldAnchorGround1(-11, 18); 
      b2Vec2 worldAnchorGround2(10, 18); 
      float32 ratio = 1.0f; 				// Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    /// Balloon which rises up has negative Gravity
    {

    	/**var sbody* ->  It is a static b2Body with a circular shape **/ 
                                                        
      b2CircleShape circle;                                                              
      b2FixtureDef ballfd; 
      b2BodyDef ballbd;
      circle.m_radius = 1.5f;                                
      ballfd.shape = &circle;                              
      ballfd.friction = 0.0f;      
      ballfd.density = 50.0f;                                                    
      ballfd.restitution = 0.0f;                                                          
      ballbd.type = b2_dynamicBody;                       
      ballbd.position.Set(12.0f, 12.0f);
      mb4 = m_world->CreateBody(&ballbd);              
      mb4 -> SetGravityScale(-1.0f);                      // Setting Negative Gravity so that balloon rises up.
      mb4->CreateFixture(&ballfd);                     
    }
     /// Peg at the end of bar that prevent plank move upward
     {
     	/**var sbody* ->  It is a static b2Body with a circular shape **/ 
      b2Body* sbody;                                                   
      b2CircleShape circle;                               
      circle.m_radius = 0.5;                             
      b2FixtureDef ballfd;                                 
      ballfd.shape = &circle;                              
      ballfd.density = 1.0f;                             
      ballfd.friction = 0.0f;                              
      ballfd.restitution = 0.0f;                           
      b2BodyDef ballbd;                                                          
      ballbd.position.Set(7.7f, 14.0f);
      sbody = m_world->CreateBody(&ballbd);              
      sbody->CreateFixture(&ballfd);                     
    }
    /// Peg at the end of bar that prevent plank move ahead
     {
      /**var sbody* ->  It is a static b2Body with a circular shape **/ 
      b2Body* sbody;                                                   
      b2CircleShape circle;                               
      circle.m_radius = 0.5;                             
      b2FixtureDef ballfd;                                 
      ballfd.shape = &circle;                              
      ballfd.density = 1.0f;                             
      ballfd.friction = 0.0f;                              
      ballfd.restitution = 0.0f;                           
      b2BodyDef ballbd;                                                          
      ballbd.position.Set(-8.9f, 29.0f);
      sbody = m_world->CreateBody(&ballbd);              
      sbody->CreateFixture(&ballfd);                     
    } 
    /// Peg at the end of bar that prevent plank's downward movement
    {
    	/**var sbody* ->  It is a static b2Body with a circular shape **/ 
      b2Body* sbody;                                                   
      b2CircleShape circle;                               
      circle.m_radius = 0.5;                             
      b2FixtureDef ballfd;                                 
      ballfd.shape = &circle;                              
      ballfd.density = 1.0f;                             
      ballfd.friction = 0.0f;                              
      ballfd.restitution = 0.0f;                           
      b2BodyDef ballbd;                                                          
      ballbd.position.Set(9.9f, 12.0f);
      sbody = m_world->CreateBody(&ballbd);              
      sbody->CreateFixture(&ballfd);                     
    }
    /// lower half of hour class
    {
    	/**var box1 ->  It is a static b2Body with a rectangular shape **/ 
    	/// var fd1,fd2,fd3 -> It is used to set Fixtures to segments of hour class 
    	b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(-15,20);
      bd->fixedRotation = true;
    	b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
	      fd1->restitution = 0.f;
	      fd1->shape = new b2PolygonShape;
	      b2PolygonShape bs1;
	      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
	      fd1->shape = &bs1;
	      b2FixtureDef *fd2 = new b2FixtureDef;
	      fd2->density = 10.0;
	      fd2->friction = 0.5;
	      fd2->restitution = 0.f;
	      fd2->shape = new b2PolygonShape;
	      b2PolygonShape bs2;
	      bs2.SetAsBox(0.2,1.1, b2Vec2(1.5f,0.9f), 0.3);
	      fd2->shape = &bs2;
	      b2FixtureDef *fd3 = new b2FixtureDef;
	      fd3->density = 10.0;
	      fd3->friction = 0.5;
	      fd3->restitution = 0.f;
	      fd3->shape = new b2PolygonShape;
	      b2PolygonShape bs3;
	      bs3.SetAsBox(0.2,2, b2Vec2(-1.5f,0.f), -0.3);
	      fd3->shape = &bs3;
	       
	      b2Body* box1 = m_world->CreateBody(bd);
	      box1->CreateFixture(fd1);
	      box1->CreateFixture(fd2);
	      box1->CreateFixture(fd3);
    }
     /// upper half of hour glass
     {
     	/**var box1 ->  It is a static b2Body with a rectangular shape **/ 
    	/// var fd1,fd2,fd3 -> It is used to set Fixtures to segments of hour class 
    	b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(-15,24.5);
      bd->fixedRotation = true;
      bd->angle=3.15f;
    	b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
	      fd1->restitution = 0.f;
	      fd1->shape = new b2PolygonShape;
	      b2PolygonShape bs1;
	      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
	      fd1->shape = &bs1;
	      b2FixtureDef *fd2 = new b2FixtureDef;
	      fd2->density = 10.0;
	      fd2->friction = 0.5;
	      fd2->restitution = 0.f;
	      fd2->shape = new b2PolygonShape;
	      b2PolygonShape bs2;
	      bs2.SetAsBox(0.2,2, b2Vec2(1.5f,0.f), 0.3);
	      fd2->shape = &bs2;
	      b2FixtureDef *fd3 = new b2FixtureDef;
	      fd3->density = 10.0;
	      fd3->friction = 0.5;
	      fd3->restitution = 0.f;
	      fd3->shape = new b2PolygonShape;
	      b2PolygonShape bs3;
	      bs3.SetAsBox(0.2,2, b2Vec2(-1.5f,0.f), -0.3);
	      fd3->shape = &bs3;
	       
	      b2Body* box1 = m_world->CreateBody(bd);
	      box1->CreateFixture(fd1);
	      box1->CreateFixture(fd2);
	      box1->CreateFixture(fd3);
     }   
   /// Rotating plus of kinematic body type and not effected by collisions 
   {
	{
	///The Vertical Plank of plus which revolves.
      b2PolygonShape shape;                                                                         
      shape.SetAsBox(0.25f, 5.5f);                            
      b2BodyDef bd;
      bd.position.Set(-25.0f, 40.0f);                       
      bd.type = b2_kinematicBody;                           
      b2Body* body = m_world->CreateBody(&bd);            
      b2FixtureDef *fd = new b2FixtureDef;                
      fd->density = 1.f;                                  
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);                            
      /// The Horizontal Plank of plus which revolves.
      b2PolygonShape shape1;                                                                      
      shape1.SetAsBox(5.5f, 0.25f);                         
      b2BodyDef bd1;
      bd1.position.Set(-25.0f, 40.0f);                      
      bd1.type = b2_kinematicBody;                        
      b2Body* body1 = m_world->CreateBody(&bd1);          
      b2FixtureDef *fd1 = new b2FixtureDef;               
      fd1->density = 1.f;                                
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
      body1->CreateFixture(fd1);

      /// Creating an invisible hinge to rotate the platform: Static object.
      b2PolygonShape shape2;                              
      shape2.SetAsBox(0.2f, 1.0f);                        
      b2BodyDef bd2;
      bd2.position.Set(-25.0, 40.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);          
      b2Body* body3 = m_world->CreateBody(&bd2);          
        
      /// Revolute joint between the plank and the hinge.    
      b2RevoluteJointDef jointDef;                         
      jointDef.bodyA = body;                              
      jointDef.bodyB = body2;   
      jointDef.localAnchorA.Set(0,0);                     
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;                   
      m_world->CreateJoint(&jointDef); 

      b2RevoluteJointDef jointDef1;  
      jointDef1.bodyA = body1;                            
      jointDef1.bodyB = body3 ;   
      jointDef1.localAnchorA.Set(0,0);
      jointDef1.localAnchorB.Set(0,0);
      jointDef1.collideConnected = true;
      m_world->CreateJoint(&jointDef1); 
      body->SetAngularVelocity(2.5f);
      body1->SetAngularVelocity(2.5f);
	}

   }
   /// Rotating plus of kinematic body type and not effected by collisions 
   {
	{
	///The Vertical Plank of plus which revolves.
      b2PolygonShape shape;                                                                        
      shape.SetAsBox(0.25f, 5.5f);                           
      b2BodyDef bd;
      bd.position.Set(-15.0f, 40.0f);                       
      bd.type = b2_kinematicBody;                         
      b2Body* body = m_world->CreateBody(&bd);            
      b2FixtureDef *fd = new b2FixtureDef;                
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
       /// The Horizontal Plank of plus which revolves.
      b2PolygonShape shape1;                                                                    
      shape1.SetAsBox(5.5f, 0.25f);                         
      b2BodyDef bd1;
      bd1.position.Set(-15.0f, 40.0f);                      
      bd1.type = b2_kinematicBody;                         
      b2Body* body1 = m_world->CreateBody(&bd1);            
      b2FixtureDef *fd1 = new b2FixtureDef;                
      fd1->density = 1.f;                               
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
      body1->CreateFixture(fd1);

      /// Creating an invisible hinge to rotate the platform: Static object.
      b2PolygonShape shape2;                              
      shape2.SetAsBox(0.2f, 1.0f); 
      b2BodyDef bd2;                        
      bd2.position.Set(-15.0, 40.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);           
      b2Body* body3 = m_world->CreateBody(&bd2);           
        
      /// Revolute joint between the plank and the hinge.    
      b2RevoluteJointDef jointDef;                          
      jointDef.bodyA = body;                               
      jointDef.bodyB = body2;   
      jointDef.localAnchorA.Set(0,0);                      
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;                   
      m_world->CreateJoint(&jointDef); 

      b2RevoluteJointDef jointDef1;  
      jointDef1.bodyA = body1;                            
      jointDef1.bodyB = body3 ;   
      jointDef1.localAnchorA.Set(0,0);                    
      jointDef1.localAnchorB.Set(0,0);
      jointDef1.collideConnected = true;                  
      m_world->CreateJoint(&jointDef1); 
      body->SetAngularVelocity(2.5f);
      body1->SetAngularVelocity(2.5f);
	}

   }
 
    ///The heavy sphere on the platform which hits the cycle
    {
    	/**var sbody* ->  It is a static b2Body with a circular shape **/ 
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1545.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(20.0f, 48.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    /// The sphere which transfer energy from spring mass system to another sphere
    {
    	/**var sbody* ->  It is a static b2Body with a circular shape **/ 
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 1.5f;			// Elasticity >1 so that ball rises upwards and collides
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(20.0f, 30.0f);
      sbody1 = m_world->CreateBody(&ballbd);
      sbody1->CreateFixture(&ballfd);
      sbody1->SetUserData(this);
      
    }
    /// Cycle---> This is bicycle made of a trapezium and two square wheels connected to it with revolute joints.
     {
       b2PolygonShape chain;
       b2Vec2 v[5];
       b2Vec2 center;
       center.Set(-28.5f,8.0f);
       struct  {
           float height;
           float l1;
           float l2;
           }cycle;
        cycle.height=3.0f;
        cycle.l1=6.0f;
        cycle.l2=3.0f;
       v[0].Set(center.x-cycle.l1,center.y-cycle.height);
       v[1].Set(center.x+cycle.l1,center.y-cycle.height);
       v[2].Set(center.x+cycle.l2,center.y+cycle.height);
       v[3].Set(center.x-cycle.l2,center.y+cycle.height);
       v[4].Set(center.x-cycle.l1,center.y-cycle.height);
       chain.Set(v,5);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.05f;
      fd->shape = &chain;
       b2BodyDef bd1;
      bd1.type=b2_dynamicBody;
       b2Body * body1 = m_world->CreateBody(&bd1);
        body1->CreateFixture(fd);

    ///Wheel 1
    b2PolygonShape shape1;
      float length=3.14f;
      shape1.SetAsBox(length, length);
    b2BodyDef bd2;
      bd2.position.Set(center.x-cycle.l1, center.y-cycle.height);
      bd2.type = b2_dynamicBody;
      bd2.angle=(45.0f/180.0f*3.14f);
      b2Body* body2 = m_world->CreateBody(&bd2);
      body2->SetAngularVelocity(10.0);
      fd->density = 0.05f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape1;
      body2->CreateFixture(fd);

    ///wheel 2
    b2PolygonShape shape2;
      shape2.SetAsBox(length, length);
    b2BodyDef bd3;
      bd3.position.Set(center.x+cycle.l1, center.y-cycle.height);
      bd3.type = b2_dynamicBody;
      bd3.angle=(45.0f/180.0f*3.14f);
      b2Body* body3 = m_world->CreateBody(&bd3);
      body3->SetAngularVelocity(10.0);
      fd->density = 0.05f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape2;
      body3->CreateFixture(fd);

      b2RevoluteJointDef jointDef1;
      jointDef1.bodyA = body1;
      jointDef1.bodyB = body2;
      jointDef1.localAnchorA.Set(center.x-cycle.l1,center.y-cycle.height);
      jointDef1.localAnchorB.Set(0.0f,0.0f);
      jointDef1.collideConnected = false;
      jointDef1.enableMotor=true;
      jointDef1.motorSpeed=0.0f;
      m_world->CreateJoint(&jointDef1);

      b2RevoluteJointDef jointDef2;
      jointDef2.bodyA = body1;
      jointDef2.bodyB = body3;
      jointDef2.localAnchorA.Set(center.x+cycle.l1,center.y-cycle.height);
      jointDef2.localAnchorB.Set(0.0f,0.0f);
      jointDef2.collideConnected = false;
      jointDef2.enableMotor=true;
      jointDef2.motorSpeed=0.0f;
      m_world->CreateJoint(&jointDef2);
    }
     /** Code for Newton's pendulum **/
  {
    /**var newBody ->  It is a static b2Body with a circular shape **/ 
    b2Body* newBody;
    for (int i=0; i<4; i++)
    {
      b2BodyDef bd;
      bd.position.Set(-40.0f+ i*2.0f , 10.0f);
      bd.type = b2_dynamicBody;
      b2CircleShape circle;
      circle.m_radius = 1.0f;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      newBody = m_world->CreateBody(&bd);
      newBody->CreateFixture(&ballfd);
      b2Body* b4;
      b2BodyDef tp;
      tp.position.Set(-40.0f+i*2.0f, 16.10f);
      b2PolygonShape shape;
      shape.SetAsBox(0.02f, 0.02f);
      b4 = m_world->CreateBody(&tp);
      b4->CreateFixture(&shape, 100.0f);
      
      /// Defining a revolute joint
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-40.0f + i*2.0f, 16.0f);
      jd.Initialize(newBody, b4, anchor);
      m_world->CreateJoint(&jd);
    }    
  }
  /** Initial Pulley which is to be pulled down **/
  /// Two pulley joints to make plank between hourglass Dragable 
  /// All planks are mass less
  {
    /// Theres a sbody defined in dominos.hpp which is dragable using mouse click
 	///Body with no density
    /**var mbody ->  It is a static b2Body* with a rectangular shape **/ 
 	b2Body* mbody;
 	b2BodyDef bd4;
 	b2PolygonShape shp;
 	shp.SetAsBox(0.50f, 0.25f);
 	bd4.position.Set(-28.0f, 20.0f);
 	bd4.type = b2_dynamicBody;
 	mbody = m_world->CreateBody(&bd4);
 	mbody->CreateFixture(&shp, 5.0f);
 	mbody->SetGravityScale(0.0f);

 	/// Body equivalent to pull
     /**var sbody ->  It is a static b2Body* with a rectangular shape **/ 
 	
 	b2BodyDef db;
 	b2PolygonShape sph;
 	sph.SetAsBox(2.0f, 0.50f);
 	db.type = b2_dynamicBody;
 	db.position.Set(-20.0f, 15.0f);
 	sbody = m_world->CreateBody(&db);
 	sbody->CreateFixture(&sph, 5.00f);
 	sbody->SetGravityScale(0.0f);
 	/// First pulley joint
 	b2PulleyJointDef* mj = new b2PulleyJointDef();
 	b2Vec2 worldAnchorOnBody1(-25.0f, 20.0f);
 	b2Vec2 worldAnchorOnBody2(-15.0f, 15.0f);
 	b2Vec2 worldAnchorGround1(-25.0f, 20.0f);
 	b2Vec2 worldAnchorGround2(-20.0f, 20.0f);
 	float32 ratio = 1.0f;
 	mj->Initialize(mbody, sbody, worldAnchorGround1, worldAnchorGround2, mbody->GetWorldCenter(), sbody->GetWorldCenter(), ratio);
 	m_world->CreateJoint(mj);

 	/// Third Body
     /**var mbody2 ->  It is a static b2Body* with a rectangular shape **/ 
 	b2Body* mbody2;
 	b2BodyDef db1;
 	b2PolygonShape sph1;
 	sph1.SetAsBox(2.0f, 0.25f);
 	db1.type = b2_dynamicBody;
 	db1.position.Set(-15.0f, 22.0f);
 	mbody2 = m_world->CreateBody(&db1);
 	mbody2->CreateFixture(&sph1, 0.00f);
 	mbody2->SetGravityScale(0.0f);
 
 	///Second pulley joint

 	b2PulleyJointDef* mj1 = new b2PulleyJointDef();
 	b2Vec2 worldAnchorOnBody11(-28.0f, 20.0f);
 	b2Vec2 worldAnchorOnBody12(-15.0f, 22.0f);
 	b2Vec2 worldAnchorGround11(-30.0f, 20.0f);
 	b2Vec2 worldAnchorGround12(-30.0f, 22.0f);
 	ratio = 1.0f;
 	mj1->Initialize(mbody, mbody2, worldAnchorGround11, worldAnchorGround12, mbody->GetWorldCenter(), mbody2->GetWorldCenter(), ratio);
  	m_world->CreateJoint(mj1);
  }
  ///balls in hour glass(mimics water droplets)
    {
      b2Body* spherebody;
	
     b2CircleShape circle;
     circle.m_radius = 0.18;
	
     b2FixtureDef ballfd;
     ballfd.shape = &circle;
      ballfd.density = 60.0f;
     ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	for(int j=0;j<5;j++){
     for (int i = 0; i < 7; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-15.5f + i*0.2, 24.6f+j*0.19);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
  }
   }
     /// Now we form a catenary curve which hangs from the sphere, which on hitting falls down and completes the road. 
	{
    //Forming a catenary curve
    /*b2PolygonShape shape;                                 // Creating a shape for the platform.                                        
      shape.SetAsBox(5.5f, 0.25f);                           // Defining the dimensions of the platform  
      b2BodyDef bd;
      bd.position.Set(35.0f, 30.0f);                     // Setting position of the platform.  
      bd.type = b2_dynamicBody;                           // Setting type to dynamic
      ba = m_world->CreateBody(&bd);
      ba->CreateFixture(&shape, 0.2f);*/
    b2Vec2 w;
    int num=200;
    b2Vec2 v[num];
    w.Set(38.57f,30.0f);
    for(int i=0;i<1;i++)
      {
       float size=2.5f;
       b2ChainShape chain;
       catenary(v,w,size,num);
       chain.CreateChain(v,num);
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       ba = m_world->CreateBody(&bd);
       ba->CreateFixture(&chain, 0.2f);
       //cout<<"Bodywertgfdsa"<<endl;
       w=v[num-1];
       }
	/// The UserData corresponding to this Catenary curve is set to a non - null value so that it can be accessed in Contact Listener.
       ba->SetUserData(&numb);
    //Small spheres... used for joint
     b2BodyDef waste1;
     b2BodyDef waste2;
     
     b2CircleShape circ;
     circ.m_radius=0.5;
     waste1.position.Set(41.185f, 40.0f);
     waste2.position.Set(33.8f, 40.0f);
     mb1= m_world->CreateBody(&waste1);
     //mb2 = m_world->CreateBody(&waste2);
     mb1->CreateFixture(&circ, 0.0f);
     //mb2->CreateFixture(&circ, 0.0f);
     
    /// A new distance joint is defined connecting the Sphere to this Catenary curve.  (This is the joint which breaks) 
    //if (flag==0){
    dj1 = new b2DistanceJointDef();
    dj2 = new b2DistanceJointDef();
    b2Vec2 worldAnchorOnBody1(28.2f, 40.0f);
    b2Vec2 worldAnchorOnBody2(33.8f, 40.0f);
    float a =2.5f;
    float alpha=a*asinh(a/2);
    float beta=-1*a*(1-cosh(asinh(a/2)));
    b2Vec2 wv(41.185f, 30.0f+catenary_equation(0.2f, 2.5f, alpha, beta));
    b2Vec2 wv1(33.8f, 30.0f);
	{
		b2Body* bm;
	b2PolygonShape shape;                                                                  
      shape.SetAsBox(0.3f, 0.1f);                       
      b2BodyDef bd;
      bd.position.Set(43.4f, 0.0f);				//two pillars                                         
      bm = m_world->CreateBody(&bd);
      bm->CreateFixture(&shape, 0.2f);

		b2Body* bm1;
      shape.SetAsBox(0.3f, 0.1f);                       
      bd.position.Set(38.8f, 0.0f);				//two pillars                                         
      bm1 = m_world->CreateBody(&bd);
      bm1->CreateFixture(&shape, 0.2f);

	}

    //b2Vec2 wv(31.2f, 30.0f+catenary_equation(0.2f, 2.5f, alpha, beta));
    //b2Vec2 wv1(33.8f, 30.0f+catenary_equation(3.8f, 2.5f, alpha, beta));
    dj1->Initialize(ba, mb1, wv, mb1->GetWorldCenter());
    //dj2->Initialize(ba, mb2, ba->GetWorldCenter(), mb2->GetWorldCenter());
    dj1->collideConnected=true;
    //dj2->collideConnected=true;
    m_world->CreateJoint(dj1);
    //m_world->CreateJoint(dj2);

    //b2JointEdge* je = ba->GetJointList();
    //b2Joint* j1 = je->joint;
    //m_world->DestroyJoint(joint);
   //}
	b2Body* bm;
	b2PolygonShape shape;                                 // Creating a shape for the platform.                                        
      shape.SetAsBox(3.5f, 0.25f);                           // Defining the dimensions of the platform  
      b2BodyDef bd;
      bd.position.Set(31.8f, 32.0f);                     // Setting position of the platform.  
     // bd.type = b2_dynamicBody;                           // Setting type to dynamic
      bm = m_world->CreateBody(&bd);
      bm->CreateFixture(&shape, 0.2f);
/*
      
     //b2Body* mb4;
     b2BodyDef m23;
     b2CircleShape circ1;
     circ1.m_radius=0.4f;
     m23.position.Set(34.5f, 35.5f);
	m23.type = b2_dynamicBody;
     //waste2.position.Set(33.8f, 40.0f);
     mb4= m_world->CreateBody(&m23);
     mb4->CreateFixture(&circ1, 0.0f);*/
     
      // Creating an invisible hinge to rotate the platform: Static object.
      b2PolygonShape shape2;                              
      shape2.SetAsBox(0.00f, 0.0f);                         //Dimensions of the hinge.
      b2BodyDef bd2;
      bd2.position.Set(34.5f, 35.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);           //Static Object,so default fixture.
      body2->CreateFixture(&shape2, 0.0f);
      //b2Body* body3 = m_world->CreateBody(&bd2);           //Static Object,so default fixture.
        
      // Revolute joint between the plank and the hinge.    
      b2RevoluteJointDef jointDef;                         // Creating a revolute joint to rotate the platform 
      jointDef.bodyA = bm;                               // by joining the bar and the hinge.
      jointDef.bodyB = body2;   
      jointDef.localAnchorA.Set(0,0);                      // Setting Local Anchor
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;                   // The bodies will not collide with each other
      m_world->CreateJoint(&jointDef); 
      
 }
 
  }


	
/// Mouse Down :: On mouse click object becomes dragable and function parameter is b2Vec2
	void dominos_t::mouse_down(const b2Vec2& p)
    {
    	
	b2Vec2 vec= sbody-> GetPosition();		
	float32 dist = sqrtf((p.x-vec.x)*(p.x-vec.x) + (p.y-vec.y)*(p.y-vec.y));
	if(dist<10.0f){	
	b2AABB aabb;
	aabb.lowerBound.Set(p.x - 0.001, p.y - 0.001);
	aabb.upperBound.Set(p.x + 0.001, p.y + 0.001);
	
	b2MouseJointDef def;
	def.bodyA = b1;
	def.bodyB =sbody;
	def.target = p;
	def.collideConnected = true;
	def.maxForce = 1000*sbody->GetMass();
	def.dampingRatio = 0;
	mouse_joint = (b2MouseJoint*)m_world->CreateJoint(&def);
	sbody->SetAwake(true);
	}
    }
///Mouse Up :: On mouse release the joint is release and is destroyed
	void dominos_t::mouse_up(const b2Vec2& p)
    {
	if (mouse_joint!=NULL){    	
	m_world->DestroyJoint(mouse_joint);
	mouse_joint=NULL;
	}
	}
/// Mouse_move:  On Mouse movement the object is moved with joint
	 void dominos_t::mouse_move(const b2Vec2& p)
	{
	    if (mouse_joint !=NULL)		
		mouse_joint->SetTarget(p);

	}
	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);				//Name written on top is "Dominos" and dominos_t::create will
}										//create a dominos_t object with the base_sim_t pointer
										//as it is  base class
