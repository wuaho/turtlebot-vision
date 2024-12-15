<h1 align="center" >
  <img src="media/hikorea-mascot.png" alt="HiKorea Mascot" width="200" />
  <br>
  Turtlebot Vision
  <br>
</h1>
<h4 align="center">A study of the efficiency in the communications of turtlebot with vision processed on a local server.<img src="https://cdn-icons-png.flaticon.com/512/197/197582.png" width="13"/> <a href="https://www.hikorea.go.kr/Main.pt" target="_blank"> Hi Korea Immigration Portal</a>.
<p align="center">

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)

# turtlebot-vision
Implementation of nodes for the ROS turtlebot that make use of the images taken by the Microsoft Kinect to evaluate the communication efficiency between a local server and a turtlebot.
## Purpose
This project was developed in my last year of university for the Final Degree Project (you can check it here). The purpose of it was to study of the efficiency in the communications of a mobile robot with vision processed on a local server. For this project, I made use of technologies such as ROS (Robot Operating System) to deploy a turtle robot (TurtleBot), both in a real environment and in a simulated one with Gazebo and with the help of OpenCV for image processing. The results I obtained by analyzing TCP connections pointed out that ROS systems are not prepared for wireless transfer of heavy packets.
You can check the paper here but apart from the Abstract, everything is in Spanish (you can check it [here](./ROS_TFG.pdf ))
