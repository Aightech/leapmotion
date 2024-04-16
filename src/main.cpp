#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "LeapC.h"
#include "leapmotion.hpp"
#include <SFML/Graphics.hpp>
#define W_WIDTH 800
#define W_HEIGHT 600

int64_t lastFrameID = 0; //The last frame received

int
main(int argc, char **argv)
{
    sf::RenderWindow window(sf::VideoMode(W_WIDTH, W_HEIGHT), "My window");
    window.clear(sf::Color::White);
    // //set the framerate to 30 frames per second
    window.setFramerateLimit(30);

    sf::CircleShape circle(10);
    circle.setFillColor(sf::Color::Red);
    sf::RectangleShape rectangle(sf::Vector2f(20, 20));
    rectangle.setFillColor(sf::Color::Blue);

    float origin[3] = {0, 0, 0};
    float current[3] = {0, 0, 0};
    float position[3] = {0, 0, 0};
    bool isGrabbing = false;
    bool justGrabbed = false;
    LEAP_HAND *hand = NULL;
    OpenConnection();
    while(!IsConnected)
        millisleep(100); //wait a bit to let the connection complete

    printf("Connected.");
    LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
    if(deviceProps)
        printf("Using device %s.\n", deviceProps->serial);

    bool running = true;
    while(running)
    {
        window.clear(sf::Color::White);
        //sfml event handling
        sf::Event event;
        while(window.pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
            {
                window.close();
                running = false;
            }
        }

        LEAP_TRACKING_EVENT *frame = GetFrame();
        if(frame && (frame->tracking_frame_id > lastFrameID))
        {
            lastFrameID = frame->tracking_frame_id;
            // printf("Frame %lli with %i hands.\n",
            //        (long long int)frame->tracking_frame_id, frame->nHands);
            for(uint32_t h = 0; h < frame->nHands; h++)
            {
                hand = &frame->pHands[h];
                if(hand->type == eLeapHandType_Right)
                {
                    justGrabbed = !isGrabbing && (hand->grab_strength > 0.8);
                    isGrabbing = hand->grab_strength > 0.8;
                    current[0] = hand->palm.position.x;
                    current[1] = hand->palm.position.y;
                    current[2] = hand->palm.position.z;
                    //std::cout << "x: " << current[0] - origin[0] << " y: " << current[1] - origin[1] << " z: " << current[2] - origin[2] << std::endl;
                }
            }
        }

        //if hand is not null, draw the hand has a circle using x and y coordinates
        if(hand != NULL)
        {
            if(justGrabbed)
            {
                origin[0] = current[0] - position[0];
                origin[1] = current[1] - position[1];
                origin[2] = current[2] - position[2];
            }
            if(isGrabbing)
            {
                position[0] = current[0] - origin[0];
                position[1] = current[1] - origin[1];
                position[2] = current[2] - origin[2];
            }

            float x = position[0] + W_WIDTH / 2;
            float y = position[2] + W_HEIGHT / 2;
            float z = position[1] + W_HEIGHT / 2;
            //std::cout << "x: " << x << " y: " << y << std::endl;
            circle.setPosition(x, y);
            window.draw(circle);
            rectangle.setPosition(W_WIDTH * 0.95, z);
            window.draw(rectangle);
        }
        window.display();

    } //ctrl-c to exit
    return 0;
}
