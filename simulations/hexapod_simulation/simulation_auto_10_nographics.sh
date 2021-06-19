#!/bin/bash
## on revision 
## looking for best learning rates and lambdas 

#find two map: 13060, 25316, 169

for trial in {1..100}
do
  number=$RANDOM
  echo TRIAL $trial/100
  echo ./start -diamond -layers 2 -period 2 -config config1.txt -simtime 30 -playground 0.75 -terrain 1 -seed $number -realtimefactor 0 -topview -nographics -terrain_coverage
  ./start -diamond -layers 2 -period 2 -config config1.txt -simtime 30 -playground 0.75 -terrain 1 -seed $number -realtimefactor 0 -topview -nographics -terrain_coverage
done
