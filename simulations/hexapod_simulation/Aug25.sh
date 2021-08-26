#!/bin/bash
## on revision 
## looking for best urate and synboost 

for trial in {1..10}
do
  number=$RANDOM
  echo TRIAL $trial/10
  echo ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.01 -terrain 1 -seed $number -realtimefactor 0 -nographics -terrain_coverage -no_plot
  ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.01 -terrain 1 -seed $number -realtimefactor 0 -nographics -terrain_coverage -no_plot -log_layer #-log

  printf '\7'

done
