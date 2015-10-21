/* 
 * Group 14: CS251 Box2D project 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */

#ifndef _CALLBACKS_HPP_
#define _CALLBACKS_HPP_

/*! 
  Callback functions are called when a event occurs due to the press of a key, mouse button, or a GUI element like a menu item is clicked or scroller is dragged. Some callbacks are triggered after a fixed intervals when a timer elapses.  
*/

#include "render.hpp"
#include "cs251_base.hpp"

namespace cs251
{
  class callbacks_t
  {
  public:
  //! Helper function for coordinates system conversion
  static b2Vec2 convert_screen_to_world(int32 x, int32 y);
  
  //! GLUT resize callback. This is set via GLUI. 
  //! This gets called when the window is resized. Passed as parameters
  //! are the current window width and height.
  static void resize_cb(int32 w, int32 h);
  
  //! GLUT keyboard callback
  //! This gets called whenever a key is pressed
  static void keyboard_cb(unsigned char key, int x, int y);
  
  //! GLUT keyboard callback for keys with special keycodes
  static void keyboard_special_cb(int key, int x, int y);
  

  static void keyboard_up_cb(unsigned char key, int x, int y);
  
  //! GLUT mouse callback
  //! This is called when a mouse button is pressed
  static void mouse_cb(int32 button, int32 state, int32 x, int32 y);
  
  //! GLUT Mouse motion callback
  //! This is called when the mouse is moved/dragged
  static void mouse_motion_cb(int32 x, int32 y);
  
  //! GLUT timer callback.
  //! This is used to control the frame rate - figure out how by reading the GLUT manual.
  static void timer_cb(int);
  
  //! GLUT display callback
  //! This draws every time a draw event is posted. 
  //! It also executes the main simulation loop by calling test->step(...)
  static void display_cb(void);
  
  //! GLUI callback - Called when the restart button is pressed
  static void restart_cb(int);
  
  //! GLUI callback - Called when the pause button is pressed
  static void pause_cb(int);
  
  //! GLUI callback - Called when the single-step button is pressed
  static void single_step_cb(int);
  
  //! GLUI callback - Called when the exit button is pressed
  static void exit_cb(int code);
  };
};

#endif
