
#include <SFML/Graphics.hpp>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>

int main(int args, char ** argv) {

   sf::RenderWindow window;
   sf::View view = sf::View();
   sf::Event event;

   double winScale;
   double startX, startY, posX, posY;

   bool   mouseDown;
   bool   paused;
   int    lastKey;
   int    xpos, ypos;
   double scale;

   window.create(sf::VideoMode(1280,720),"viewer");
   window.setFramerateLimit(30);
   xpos = window.getSize().x/2;
   ypos = window.getSize().y/2;
   scale = 1.0;

   while (window.isOpen()) {
      while (window.pollEvent(event)) {
         if (event.type == sf::Event::Closed) {
            window.close();
         }
         else if (event.type == sf::Event::MouseButtonPressed) {
            sf::Vector2f pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            sf::Vector2u ws = window.getSize();
            mouseDown = true;
         }
         else if (event.type == sf::Event::MouseButtonReleased) {
            mouseDown = false;
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
            }
         }
      }

      window.clear(sf::Color(0,0,0,0));
      window.setView(window.getDefaultView());

      /* put drawings here */

      /* end drawings here */

      view.setCenter(xpos,ypos);
      view.setSize(window.getSize().x,window.getSize().y);
      view.zoom(scale);
      window.setView(view);
      window.display();
   }
}
