
#include <SFML/Graphics.hpp>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>

#include "environment.hpp"

#include "point_set.hpp"
#include "simple_point_set.hpp"

#include "collision_engine.hpp"
#include "simple_collision_engine.hpp"

#include "rrt_base.hpp"
#include "rrt.hpp"

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
   bool   paused;
   int    lastKey;
   int    xpos, ypos;
   double rate;
   double scale;

   /* Variables for the RRT process */
   environment env = environment(200,100);
   point_set * points = new simple_point_set();
   collision_engine * ce = new simple_collision_engine(&env);
   rrt_base * rrt_algo = new rrt(vertex(100,50),&env,points,ce);

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

      if (leftMouseDown) {
         sf::Vector2f pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
         vertex mpos = vertex(pos.x/10,pos.y/10);
         if (mpos.x > 0 && mpos.y > 0 && mpos.x < env.xsize && mpos.y < env.ysize)
            env.set((int)floor(mpos.x),(int)floor(mpos.y));
      }

      if (rightMouseDown) {
         sf::Vector2f pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
         vertex mpos = vertex(pos.x/10,pos.y/10);
         if (mpos.x > 0 && mpos.y > 0 && mpos.x < env.xsize && mpos.y < env.ysize)
            env.unset((int)floor(mpos.x),(int)floor(mpos.y));
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
                  points->reset();
                  break;
               case sf::Keyboard::E:
                  env.clear();
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
      points->draw(&window);

      /* end drawings here */

      window.display();
   }
}
