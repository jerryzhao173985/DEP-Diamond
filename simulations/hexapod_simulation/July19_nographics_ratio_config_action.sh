#!/bin/bash
## on revision 
## looking for best urate and synboost 

for trial in {1..10}
do
  # number=$RANDOM
  echo TRIAL $trial/10
  echo ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.75 -terrain 1 -seed 22646 -realtimefactor 0 -nographics -terrain_coverage -no_plot
  ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.75 -terrain 1 -seed 22646 -realtimefactor 0 -nographics -terrain_coverage -no_plot -log_layer #-log

done
