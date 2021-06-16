Simulation code for the humanoid simulations of the paper

R. Der and G. Martius. Novel plasticity rule can explain the development of sensorimotor intelligence. Proceedings of the National Academy of Sciences, 112(45):E6224--E6232, 2015. arXiv Preprint http://arxiv.org/abs/1505.00835.

Check out the supplementary:
http://playfulmachines.com/DEP

First make sure you have lpzrobots installed, see README.txt in the top folder.

Compile the code in this directory via
make

then you get a program called "start"

You can specify the parameters in the commandline such that you can change the parameters as ou like. There are many switched, see
./start -h
for help.
There are also many parameters of the controller and of the simulation that you can change during the runtime and you can also specify at start with the -set "{key=value key2=value2...}" option. Please see the online help of lpzrobots (http://robot.informatik.uni-leipzig.de/software/doc/html/index.html#RunExample) to get an idea how it works and how to control the software.
There is also some useful resources in our book "The Playful Machine", see pdf on playfulmachines.com,  page 303ff.

The naming of the controller parameters differs a bit from the paper:
synboost := activity level = kappa in paper
urate    := update rate (speed of synaptic dynamics)
         tau (paper) =1/(simulation_rate*urate) = 1/(50* urate)


Video 2:
Humanoid crawling:
./start -set "{epsh=0.05 synboost=1.4 indnorm=0 urate=0.005}" -m 5
one can play with the physical parameters of the robot as well, for instance:
- hip2jointlimit=1 makes the legs more apart or closer together
- anklepower=6 makes it more or less crawl

A certain controller can be saved on the console with Ctrl+C
> store 101 nameoffile.ctrl
(see also > help)

Video 4:
(A) from snapshots:
./start -set "{epsh=0.05 synboost=1.4 indnorm=0 urate=0.005}" -m 5
then Ctrl+C and
load 101 controller/Crawl_cluster_5.ctrl      -> 1 (run 1)
load 101 controller/Crawl_cluster_7.ctrl      -> 2 (run 1)
load 101 controller/Crawl2_cluster_8.ctrl     -> 3 (run 2)
load 101 controller/Crawl2_cluster_5.ctrl     -> 4 (run 2)
load 101 controller/Crawl_cluster_5.ctrl      -> 1 (run 1)
load 101 controller/Crawl_cluster_4.ctrl      -> 5 (run 1)
load 101 controller/Crawl_cluster_7.ctrl      -> 2 (run 1)
You can also try the other controllers (cluster centers).
You may need to give the robot a little push in order to get going (Ctrl+Left-Mouse)

Video 5:
(A) Humanoid at a wheel
./start -trainer -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96}"  -m 5
You can use d/D to influence the wheel, see also 'h' in the graphical window for other keys.

(B) Humanoid at two wheels
 ./start -trainer -trainer4feet -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96 anklejointlimit=0.7}"  -m 5


or with limited back/pelvis movement
./start -trainer -trainer4feet -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96 anklejointlimit=0.7 backjointlimit=0.5 pelvisjointlimit=0.3}"  -m 5

Video 6:
(A) two humanoids sitting
./start -trainer2 -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=1}"  -m 5

(B) two humanoids standing
./start -trainer2 -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 powerfactor=1.5 synboost=1.1}"  -m 5
then press 'X' to remove the stools


You can use -f 1 to record all the parameters etc. There are tools to process these files, such as selectcolumns.pl. (just call it without arguments for help)
You can also use -g 1 to a visual tools to look at all control parameters online with gnuplot.
Use ./start --help to get an overview.
