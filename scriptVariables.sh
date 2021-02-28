#!/bin/bash

### Este es un modelo del script a ser lanzado ya sea en el servidor o en el robot tortuga. Ciertos parametros deberan de ser 
### reemplazados segun en la maquina en la que se este. Este script debera de ser lanzado mediante . /path_al_script/scriptVariables.sh

### Aun debe de ser mejorado este fichero ya que al ejecutarlo solo sirve para una unica bash

export TURTLEBOT_3D_SENSOR=kinect

#Por defecto esta puesto a http://localhost:11311
export ROS_MASTER_URI=http://localhost:11311 

#Por defecto esta puesto a http://localhost:11311
export ROS_HOSTNAME=IP_OF_TURTLEBOT

