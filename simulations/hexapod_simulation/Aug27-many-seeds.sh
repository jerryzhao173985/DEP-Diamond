#!/bin/bash
## on revision 
## looking for best urate and synboost 

rm -r data
mkdir data

for seed in {1..10}
do
  mkdir data/seed$seed
  number=$RANDOM
  for trial in {1..10}
  do
    echo TRIAL $trial/10
    echo ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.01 -seed $number -realtimefactor 0 -nographics -terrain_coverage -no_plot
    ./start -diamond -layers 2 -period 2 -config $1/config_trail$trial.txt -simtime $2 -playground 0.01 -seed $number -realtimefactor 0 -nographics -terrain_coverage -no_plot -log_layer #-log

  done
  mv layer1.txt data/seed$seed
  mv layer2.txt data/seed$seed
  mv terrain_coverage.txt data/seed$seed
  printf '\7'

done
