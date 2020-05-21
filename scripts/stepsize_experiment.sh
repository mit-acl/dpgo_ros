#!/bin/bash

echo "Stepsize experiment begins."

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=1.0 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.5 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.25 &

sleep 70

rosnode kill --all

sleep 5


##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.125 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.0625 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.03 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.01 &

sleep 70

rosnode kill --all

sleep 5

##########################################################

roslaunch dpgo_ros DPGO.launch dataset:=$1 stepsize:=0.005 &

sleep 70

rosnode kill --all

sleep 5

echo "Stepsize experiment ends."

