#!/bin/bash

### Este es un modelo del script a ser lanzado ya sea en el servidor o en el robot tortuga. Ciertos parametros deberan de ser 
### reemplazados segun en la maquina en la que se este. Este script debera de ser lanzado mediante . /PATH_AL_SCRIPT/scriptVariables.sh

### Aun debe de ser mejorado ya que tenemos que conseguir que se escriba en el fichero /etc/hosts 

echo export TURTLEBOT_3D_SENSOR=kinect >> ~/.bashrc

#Por defecto esta puesto a http://localhost:11311
#EN EL SERVIDOR: cambiar localhost por la IP del robot
#EN EL ROBOT: dejarlo tal y como esta 
echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc

#Por defecto, esta variable de entorno no existe
#EN EL SERVIDOR: introducir IP ip del servidor
#EN EL ROBOT: introducir la IP del robot

echo export ROS_HOSTNAME=IP_DE_ESTA_MAQUINA >> ~/.bashrc


#Para poder hacer un roslaunch desde el servidor en el cliente mediante ssh tenemos que añadir en el .bashrc del cliente
#el comando << export DISPLAY=:0  >>
echo export DISPLAY=:0 

#Para tener el nodo de ros que hemos creado accesible en todo momento:
echo source $HOME/turtlebot-vision/vision_ws/devel/setup.bash

