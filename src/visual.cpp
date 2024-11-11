#include <SFML/Graphics.hpp>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>

#include "environment.hpp"

#include "point_set.hpp"
#include "simple_point_set.hpp"
#include "bin_point_set.hpp"

#include "collision_engine.hpp"
#include "dda_collision_engine.hpp"

#include "rrt_base.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "quick_rrt_star.hpp"
#include "rrt_x.hpp"
#include "sst.hpp"
#include "sst_x.hpp"
#include "rrt_star_fn.hpp"

void draw_environment(sf::RenderWindow * window, environment * env) {
   sf::RectangleShape rect;
   
   /* draw border */
   rect.setSize(sf::Vector2f(10*env->xsize,10*env->ysize));
   rect.setPosition(0,0);
   rect.setFillColor(sf::Color(0,0,0,0));
   rect.setOutlineColor(sf::Color(255,255,255,255));
   rect.setOutlineThickness(1);
   window->draw(rect);

   /* draw obstacles */
   rect.setSize(sf::Vector2f(10,10));
   rect.setFillColor(sf::Color(255,255,255,255));
   for (int i = 0; i < env->xsize; ++i) {
      for (int j = 0; j < env->ysize; ++j) {
         if (env->occupancy[i][j]) {
            rect.setPosition(i*10,j*10);
            window->draw(rect);
         }
      }
   }
}

void draw_path(sf::Vector2f * mouse, sf::RenderWindow * window, point_set * p) {
   edge * start = p->closest(vertex(mouse->x/10,mouse->y/10));
   sf::Vertex point[2];
   point[1].color = sf::Color::Red;
   point[0].color = sf::Color::Red;
   while (start) {
      point[1].position = sf::Vector2f(start->to.x*10,start->to.y*10);
      point[0].position = sf::Vector2f(start->from.x*10,start->from.y*10);
      window->draw(point,2,sf::Lines);
      start = start->parent;
   }
}

int main(int args, char ** argv) {

   srand(time(NULL)*clock());

   /* SFML display types */
   sf::RenderWindow window;
   sf::View view = sf::View();
   sf::Event event;

   /* SFML/GUI window and status tracking */
   double winScale;
   double startX, startY, posX, posY;

   bool   leftMouseDown = false;
   bool   rightMouseDown = false;
   bool   display_path = false;
   bool   focus_path = false;
   bool   paused;
   int    lastKey;
   int    xpos, ypos;
   double rate;
   double scale;
   sf::Vector2f mouse;

   /* Variables for the RRT process */
   vertex start = vertex(100,50);
   environment env = environment(200,100);
   point_set * points = new bin_point_set(5);
   collision_engine * ce = new dda_collision_engine(&env);
   rrt_base * rrt_algo = new rrt(start,&env,points,ce);

   window.create(sf::VideoMode(1280,720),"viewer");
   window.setFramerateLimit(30);
   xpos = (env.xsize*10)/2;
   ypos = (env.ysize*10)/2;
   scale = 1.2;
   paused = true;
   rate = 1;

   while (window.isOpen()) {

      
      if (!paused)
         rrt_algo->generate_next(rate);

      mouse = window.mapPixelToCoords(sf::Mouse::getPosition(window));

      if (leftMouseDown) {
         vertex mpos = vertex(mouse.x/10,mouse.y/10);
         if (mpos.x > 0 && mpos.y > 0 && mpos.x < env.xsize && mpos.y < env.ysize) {
            env.set((int)floor(mpos.x),(int)floor(mpos.y));
            env.set((int)ceil(mpos.x),(int)ceil(mpos.y));
            env.set((int)floor(mpos.x),(int)ceil(mpos.y));
            env.set((int)floor(mpos.x),(int)ceil(mpos.y));
         }
         rrt_algo->notify_obstacle(mpos);
      }

      if (rightMouseDown) {
         vertex mpos = vertex(mouse.x/10,mouse.y/10);
         if (mpos.x > 0 && mpos.y > 0 && mpos.x < env.xsize && mpos.y < env.ysize) {
            env.unset((int)floor(mpos.x),(int)floor(mpos.y));
            env.unset((int)ceil(mpos.x),(int)ceil(mpos.y));
            env.unset((int)floor(mpos.x),(int)ceil(mpos.y));
            env.unset((int)ceil(mpos.x),(int)floor(mpos.y));
         }
      }

      while (window.pollEvent(event)) {
         if (event.type == sf::Event::Closed) {
            window.close();
         }
         else if (event.type == sf::Event::MouseWheelScrolled) {
            if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
               rate = (1 > rate + 10*event.mouseWheelScroll.delta ? 1 : rate + 10*event.mouseWheelScroll.delta);
         }
         else if (event.type == sf::Event::MouseButtonPressed) {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Middle))
               rate = 1;
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
               leftMouseDown = true;
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
               rightMouseDown = true;
         }
         else if (event.type == sf::Event::MouseButtonReleased) {
            if (leftMouseDown)
               leftMouseDown = false;
            if (rightMouseDown)
               rightMouseDown = false;
         }
         else if (event.type == sf::Event::KeyPressed) {
            lastKey = event.key.code;
            if (lastKey == sf::Keyboard::Q) {
               window.close();
            }
            switch (lastKey) {
               case sf::Keyboard::L:
               case sf::Keyboard::Left:
                  xpos += 10;
                  break;
               case sf::Keyboard::H:
               case sf::Keyboard::Right:
                  xpos -= 10;
                  break;
               case sf::Keyboard::J:
               case sf::Keyboard::Down:
                  ypos += 10;
                  break;
               case sf::Keyboard::K:
               case sf::Keyboard::Up:
                  ypos -= 10;
                  break;
               case sf::Keyboard::I:
                  scale *= 0.909091;
                  break;
               case sf::Keyboard::O:
                  scale *= 1.1;
                  break;
               case sf::Keyboard::Space:
                  paused = !paused;
                  break;
               case sf::Keyboard::C:
                  rrt_algo->restart(start);
                  break;
               case sf::Keyboard::E:
                  env.clear();
                  break;
               case sf::Keyboard::X:
                  display_path = !display_path;
                  break;
               case sf::Keyboard::S:
                  start = vertex(mouse.x/10,mouse.y/10);
                  rrt_algo->restart(start);
                  break;
               case sf::Keyboard::F:
                  focus_path = !focus_path;
                  break;
               case sf::Keyboard::Num0:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new rrt(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num1:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new rrt_star(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num2:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new quick_rrt_star(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num3:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new rrt_x(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num4:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new sst(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num5:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new sst_x(start,&env,points,ce);
                  break;
               case sf::Keyboard::Num6:
                  delete rrt_algo;
                  points->reset();
                  rrt_algo = new rrt_star_fn(start,&env,points,ce);
                  break;
            }
         }
      }

      window.clear(sf::Color(0,0,0,0));
      window.setView(window.getDefaultView());
      view.setCenter(xpos,ypos);
      view.setSize(window.getSize().x,window.getSize().y);
      view.zoom(scale);
      window.setView(view);

      /* drawing operations */

      draw_environment(&window,&env);
      if (!focus_path)
         points->draw(&window);
      if (display_path || focus_path)
         draw_path(&mouse,&window,points);

      /* end drawings here */

      window.display();
   }
}
