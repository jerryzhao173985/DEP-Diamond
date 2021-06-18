#!/bin/bash
## on revision 
## looking for best learning rates and lambdas 

#find two map: 13060, 25316, 169

for trial in {1..10}
do
  number=$RANDOM
  echo TRIAL $trial/10
  echo ./start -m 5 -diamond -layers 2 -period 2 -g -config config1.txt -simtime 30 -playground 0.75 -terrain 1 -seed 13060 -realtimefactor 0 -topview
  ./start -m 5 -diamond -layers 2 -period 2 -g -config config1.txt -simtime 30 -playground 0.75 -terrain 1 -seed 13060 -realtimefactor 0 -topview
done
