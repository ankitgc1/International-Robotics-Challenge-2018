# International-Robotics-Challenge-2018
#### IRC-2018(IIT-Bombay, December 2018)

The solution of IRC-2018 held at IIT-Bombay in December. We took part in IRC and reached the final round.

In the IRC there are two bots one is Manual, and the other one is Autonomous bot. The problem statement is down below.

[IRC_problem_statement](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/IRC_problem_statement.pdf)

## Description 
### Autonomous bot:-
  In the autonomous bot, there are two-part, one for image processing, and the other one for maze solving. In the maze solving part, the bot should solve the maze without manual control. The ultrasonic sensors, Arduino Uno, dc motors, dc motor driver, and the lipo battery ware used. Three ultrasonic sensors ware placed like one in the forward direction, second in the left side, and third in the right side to navigate the walls and the open path. Using these three ultrasonic sensors the bot can navigate in three direction and able to solve the maze.
  
   In the image preprocessing part, the bot should detect colored boxes and stop in front of colored boxes until manual bot come and pick up that box. The raspberry pi and pi cam ware used. For color detection HSV color range ware used. The raspberry pi and The arduino uno ware connected through USB cable. Whenever the box detected the raspberry pi send a signal to arduino to stop for a while, and the manual bot comes and pick up the detected box.

### Manual bot:-
  In the manual bot, the bot should be able to pick up a box and keep the box a particular place as described in the problem statement. The bot has to complete some task. Arduino Uno, dc motors, motor drivers, servos, lipo battery, RC transmitter, and receiver module ware used to make the manual bot.
  

### Connections:-
##### Autonomous_bot_connections:- 
In the below picture the Autonomous bot connections are defined. Feel free to use code's connection.
![alt text](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/images/Autonomous_bot_connections.jpg)

##### manual_bot_connections:-
In the below picture the Manual bot connections are defined. Feel free to use code's connection.
![alt text](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/images/manual_bot_connections.png)

#### Autonomous Bot
![alt text](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/images/autonomous_bot2.jpg)
![alt text](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/images/autonomous_bot.jpg)

#### Manual Bot
![alt text](https://github.com/ankitgc1/International-Robotics-Challenge-2018/blob/master/images/Manual_bot.jpg)
