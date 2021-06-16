/***************************************************************************
 *   Copyright (C) 2015 by Robot Group Leipzig                             *
 *    georg.martius@web.de                                                 *
 *    ralfder@mis.mpg.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/
#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <selforg/sinecontroller.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/motornoisewiring.h>
#include <selforg/copywiring.h>
#include <selforg/ringbuffer.h>
#include <selforg/motorbabbler.h>

// #include <ode_robots/skeleton.h>
#include "skeleton.h"

#include <ode_robots/joint.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/relativepositionsensor.h>

#include <ode_robots/operators.h>

#include "dep.h"

#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

#define ROBOTSTOREFILE "humanoid_initial.rob"


enum SimType { Normal, TwoNormal, Bungee, TwoBungee, Trainer, TwoTrainer};
string typeToString(SimType t){
  switch(t){
  case Normal:
    return "Normal";
  case TwoNormal:
    return "TwoNormal";
  case Bungee:
    return "Bungee";
  case TwoBungee:
    return "TwoBungee";
  case Trainer:
    return "Trainer";
  case TwoTrainer:
    return "TwoTrainer";
  }
  return "unknown";
}

double tilt        = 0.0;
double powerfactor = .6;;
string name        = "";
double noisecontrol= 0;
double feedbackstrength = 0;
double wheelmass        = 70;
char*  controllerfile[2]={0,0};
bool beside      = false;
bool fixBungee   = false;
bool exterioception = false;
bool trainer4feet  = false;
bool trainersameside  = false;
bool trainer4feetsameside  = false;
bool noEigenvalues = false;
int useSine        = false;
int babbling=0;


class ThisSim : public Simulation {
public:
  SimType type;
  Env env;

  int  fixedSegm;
  Joint* fixator;
  Joint* leftHandJoint;
  Joint* rightHandJoint;

  Playground* playground;
  //  AbstractObstacle* playground;
  double hardness;
  Substance s;

  Joint* connector[4];
  matrix::Matrix conn_forces;

  ThisSim(SimType type)
    : type(type){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
    setCaption("DEP - Differential Extrinsic Plasticity (Der & Martius)");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(2.60209, 6.00217, 2.42803),  Pos(162.466, -13.1846, 0));

    setCameraMode(Static);

    int humanoids = 1;

    bool fixedInAir = false;
    fixator=0;
    fixedSegm  = -1;
    env.type=Env::Normal;
    global.odeConfig.setParam("noise",0);
    //    global.odeConfig.setParam("realtimefactor",0);
    global.odeConfig.setParam("simstepsize",0.01);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("gravity", -9.81);

    switch(type){
    case Normal:
      env.roughness  = 2.5;
      break;
    case TwoNormal:
      env.roughness  = 2.5;
      humanoids      = 2;
      break;
    case Bungee:
      break;
    case TwoBungee:
      humanoids       = 2;
      break;
    case Trainer:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withTrainer= true;
      env.withTrainer4Feet = trainer4feet;
      env.wheelOpposite = !trainersameside;
      env.wheel4FeetOpposite = !trainer4feetsameside;
      env.roughness  = 2.5;
      env.wheelmass  = wheelmass;
      break;
    case TwoTrainer:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withStool2 = true;
      env.withTrainer2 = true;
      env.wheelOpposite = !trainersameside;
      env.stool2Pose = TRANSM(Pos(1.0, -0.85, 0.2+0.4));
      env.roughness  = 2.5;
      env.wheelmass  = wheelmass;
      humanoids      = 2;
      break;
    }

    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0;
    env.numBoxes    = 0;
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);

    leftHandJoint = rightHandJoint = 0;

    for (int i=0; i< humanoids; i++){ //Several humanoids
      // for some reason the humanoid robot is called Skeleton
      SkeletonConf conf = Skeleton::getDefaultConfVelServos();

      OsgHandle skelOsgHandle=osgHandle.changeColorSet(i);
      double initHeight=0.8;

      conf.useBackJoint = true;
      conf.powerFactor = powerfactor;
      conf.dampingFactor = .0;
      conf.pelvisJointLimit = M_PI/6; // new
      conf.elbowJointLimit = 1.6;     // new

      conf.backSideBend = true;

      switch(type){
      case Normal:
      case TwoNormal:
        initHeight = 0.45;
        break;
      case Bungee:
      case TwoBungee:
        initHeight = 0.1;
        conf.powerFactor = 0.3;
        break;
      case Trainer:
      case TwoTrainer:
        initHeight = 0.1;
        conf.handsRotating = false;
        conf.useBackJoint = true;
        conf.backSideBend = true;
        fixedInAir = true;
        fixedSegm = Skeleton::Hip;
        break;
      }

      OdeHandle skelHandle=odeHandle;

      Skeleton* human = new Skeleton(skelHandle, skelOsgHandle,conf,
                                     "Humanoid" + itos(i) + "_" + typeToString(type) + "_" + ::name);
      if(exterioception){
        human->addSensor(std::make_shared<SpeedSensor>(10.0, SpeedSensor::RotationalRel, Sensor::X),
                         Attachment(Skeleton::Hip));
      }

      if(type==TwoBungee || type==TwoNormal){
        human->place( ROTM(M_PI_2,1,0,0) * ROTM( M_PI,0,0,1) * TRANSM(2*i, 0, initHeight));
        // human->place( ROTM(M_PI_2,1,0,0) * TRANSM(1, 0, initHeight) * ROTM( i*M_PI,0,0,1)); // symmetric
      }else if((i==0 && env.withStool) || (i==1 && env.withStool2)) {
        human->place( ROTM(M_PI_2,1,0,0) * ROTM( M_PI,0,0,1)
                      * TRANSM(0, 0.15,-0.62)*(i==0 ? env.stoolPose : env.stool2Pose));
      } else{
        human->place( ROTM(M_PI_2,1,0,0)*ROTM( i%2==0 ? M_PI : 0,0,0,1)
                      * ROTM( tilt ,1,0,0)
                      * TRANSM(1*i, 0.10*i, initHeight));
      }
      if(env.wheel4hands) human->addInspectable(env.wheel4hands);
      if(env.wheel4feet)  human->addInspectable(env.wheel4feet);

      if( fixedInAir){
        human->fixate(global, fixedSegm, 0);
      }

      AbstractWiring* wiring;
      if(noisecontrol>0){
        wiring = new MotorNoiseWiring(new ColorUniformNoise(noisecontrol),1.0);
      }else
        wiring = new One2OneWiring(new ColorUniformNoise(0.1));

      AbstractController* controller;

      if(useSine){
        controller = new SineController();
        controller->setParam("amplitude",0.8);
      }else{
        DEPConf pc = DEP::getDefaultConf();
        pc.calcEigenvalues = !noEigenvalues;
        if(babbling) pc.initModel=false;

        controller = new DEP(pc);
        controller->setParam("epsM", 0.0);
        controller->setParam("urate",0.05);
        controller->setParam("synboost",1.4);
        controller->setParam("s4avg",1);
        controller->setParam("evinterval",10);
        controller->setParam("epsh", 0);
      }
      if(noisecontrol>0){
        controller = new SineController();
        controller->setParam("amplitude",0);
        // this will set the controller output to zero and the noise comes from the MotorNoiseWiring
      }

      OdeAgent* agent = new OdeAgent(global);

      agent->init(controller, human, wiring);

      if(babbling){
        MotorBabbler* babbler=new MotorBabbler();
        babbler->setParam("minperiod",15);
        babbler->setParam("maxperiod",200);
        babbler->setParam("amplitude",1);
        babbler->setParam("resample",200);
        global.configs.push_back(babbler);
        agent->startMotorBabblingMode(babbling,babbler);
        agent->fixateRobot(global,-1,babbling/50);
      }

      switch(type){
      case Normal:
      case TwoNormal:
        agent->setParam("backjointlimit",0.3);
        agent->setParam("hip2jointlimit",1);
        agent->setParam("armpower",20);
        agent->setParam("anklepower",6);
        agent->addOperator(new LimitOrientationOperator(Axis(0,0,-1), Axis(0,0,1),
                                                        M_PI/2.0, 60));
        break;
      case Bungee:
      case TwoBungee: {
        controller->setParam("synboost",0.9);
        Pos p = human->getPosition();
        p.z()=5;
        agent->addOperator(new PullToPointOperator(p,25,true,
                                                   fixBungee ?
                                                   PullToPointOperator::XYZ :
                                                   PullToPointOperator::Z,
                                                   0, 0.1, true));
        break;
      }
      case Trainer: {
        OdeHandle myHandle(odeHandle);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Right_Hand], myHandle,0);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Left_Hand], myHandle,1);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Right_Forearm], myHandle,0);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Left_Forearm], myHandle,1);
        if(env.wheel4feet){
          env.wheel4feet->registerHand(human->getAllPrimitives()[Skeleton::Right_Shin], myHandle,0);
          env.wheel4feet->registerHand(human->getAllPrimitives()[Skeleton::Left_Shin], myHandle,1);

        }
        break;
      }
      default:
        break;
      }

      if(controllerfile[i]){
        if(controller->restoreFromFile(controllerfile[i])){
          printf("restoring controller: %s\n", controllerfile[i]);
        } else{
          fprintf(stderr,"restoring of controller failed!\n");
          exit(1);
        }
      }

      // save robot
      if(i==0) human->storeToFile(ROBOTSTOREFILE);

      global.configs.push_back(agent);
      global.agents.push_back(agent);
    }// Several humanoids end
  };


  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
    env.update();
    if(type==Trainer || type==TwoTrainer){
      // move hands to crank (if not already there, handled in attract)
      Skeleton* h = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
      if(h){
        env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Right_Hand],0, 0,
                                 odeHandle, osgHandle, global.time);
        env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Left_Hand], 1, 0,
                                 odeHandle, osgHandle, global.time);
        if(env.wheel4feet){
          env.wheel4feet->attract(h->getAllPrimitives()[Skeleton::Right_Foot],0, 0,
                                  odeHandle, osgHandle, global.time);
          env.wheel4feet->attract(h->getAllPrimitives()[Skeleton::Left_Foot], 1, 0,
                                  odeHandle, osgHandle, global.time);

        }
      }
      if(type==TwoTrainer){
        h = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
        if(h){
          env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Right_Hand],0, 1, odeHandle, osgHandle, global.time);
          env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Left_Hand], 1, 1, odeHandle, osgHandle, global.time);
        }
      }
    }
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: d/D","turn upper wheel with external force");
    au.addKeyboardMouseBinding("Sim: k/K","turn lower wheel with external force");
    au.addKeyboardMouseBinding("Sim: x","release/fixate robot");
    au.addKeyboardMouseBinding("Sim: X","Remove stool(s) and fix robots at left leg");
    au.addKeyboardMouseBinding("Sim: l","reset robot to initial position");
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'x':{
          OdeAgent* agent = getWatchedAgent();
          if(agent) {
            if(!agent->unfixateRobot(global)){
              agent->fixateRobot(global, fixedSegm, 0);
            }
          }
          return true;
        }
        case 'X':
          env.removeStools(global);
          for(auto& a: global.agents){
            a->fixateRobot(global, Skeleton::Left_Foot, 0);
          }
          return true;
        case 'l':
          {
            globalData.agents[0]->getRobot()->restoreFromFile(ROBOTSTOREFILE);
            globalData.agents[0]->getWiring()->reset();
          }
          return true;
        case 'd':
        case 'D':
          if(env.wheel4hands){
            env.wheel4hands->wheel->applyTorque(((char)key == 'D' ? -1 : 1)*env.wheel4hands->mass
                                                *(env.wheel4hands->forTwo? 3 : 1),0,0);
            global.addTmpObject(new TmpDisplayItem(new OSGCapsule(env.wheel4hands->cranklength/4,
                                                                  env.wheel4hands->width*1.5),
                                                   env.wheel4hands->wheel->getPose(),
                                                   Color(((char)key == 'D' ? 0 : 1),
                                                         ((char)key == 'D' ? 1 : 0),0)),
                                0.5);
          }
          return true;
        case 'k':
        case 'K':
          if(env.wheel4feet){
            env.wheel4feet->wheel->applyTorque(((char)key == 'K' ? -1 : 1)*env.wheel4feet->mass,0,0);
            global.addTmpObject(new TmpDisplayItem(new OSGCapsule(env.wheel4feet->cranklength/4,
                                                                  env.wheel4feet->width*1.5),
                                                   env.wheel4feet->wheel->getPose(),
                                                   Color(((char)key == 'K' ? 0 : 1),
                                                         ((char)key == 'K' ? 1 : 0),0)),
                                0.5);
          }
          return true;
        default:
          return false;
        }
    }

    return false;
  }

  virtual void usage() const {
    printf("  --------------- Specific settings for this simulation ------------\n");
    printf("    -trainer\trobot in front of a wheel\n");
    printf("    -trainer2\ttwo robots in front of a wheel\n");
    printf("    -trainer4feet\tadd also a wheel for the feet\n");
    printf("    -trainer4feet\tadd also a wheel for the feet\n");
    printf("    -trainersameside\tcranks for hands go to same side\n");
    printf("    -trainer4feetsameside\tcranks for feet go to same side\n");
    printf("    -name NAME\tname of experiment for logfiles etc(def: )\n");
    printf("    -wheelmass MASS \tmass of wheel(s) (def: 70) \n");
    printf("  --------------- Misc stuff ------------\n");
    printf("    -two\ttwo robots on ground\n");
    printf("    -bungee\ta weak humanoid attached to bungee\n");
    printf("    -2bungee\t2 weak humanoids attached to bungee\n");
    printf("    -noeigenvalues\tdon't calculate the eigenvalues\n");
    printf("    -tilt angle\tangle in rad by which to tilt the robot initialially (def: 0.0)\n");
    printf("    -powerfactor factor\tpowerfactor of robot (def: 0.6)\n");
    printf("    -loadcontroler file \tload the controller at startup\n");
    printf("    -loadcontroler1 file \tload the controller for second humanoid at startup\n");
    printf("    -noisecontrol tau \tuse noise as motor commands with correlation length 1/tau \n");
    printf("    -babbling STEPS \tstart simulation by motor babbling for given STEPS (15000)\n");
    printf("    -sine\tuse a sine-wave has control output\n");
  };

};

int main (int argc, char **argv)
{
  SimType type=Normal;
  if (Simulation::contains(argv, argc, "-two")) {
    type= TwoNormal;
  }
  if (Simulation::contains(argv, argc, "-trainer")) {
    type= Trainer;
  }
  if (Simulation::contains(argv, argc, "-beside")) {
    beside=true;
  }
  if (Simulation::contains(argv, argc, "-trainer2")) {
    type= TwoTrainer;
  }
  fixBungee      = Simulation::contains(argv, argc, "-fixbungee");
  exterioception = Simulation::contains(argv, argc, "-exterioception");
  trainer4feet   = Simulation::contains(argv, argc, "-trainer4feet");
  trainersameside   = Simulation::contains(argv, argc, "-trainersameside");
  trainer4feetsameside   = Simulation::contains(argv, argc, "-trainer4feetsameside");
  noEigenvalues  = Simulation::contains(argv, argc, "-noeigenvalues");
  if (Simulation::contains(argv, argc, "-bungee")) {
    type= Bungee;
  }
  if (Simulation::contains(argv, argc, "-2bungee")) {
    type= TwoBungee;
  }
  int index = Simulation::contains(argv, argc, "-name");
  if (index>0 && index<argc) {
    name=string(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-tilt");
  if (index>0 && index<argc) {
    tilt=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-powerfactor");
  if (index>0 && index<argc) {
    powerfactor=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-noisecontrol");
  if (index>0 && index<argc) {
    noisecontrol=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-loadcontroller");
  if (index>0 && index<argc) {
    controllerfile[0]=argv[index];
  }
  index = Simulation::contains(argv, argc, "-loadcontroller1");
  if (index>0 && index<argc) {
    controllerfile[1]=argv[index];
  }
  index = Simulation::contains(argv, argc, "-wheelmass");
  if (index>0 && index<argc) {
    wheelmass=atof(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-babbling");
  if(index >0 && argc>index){
    babbling=atoi(argv[index]);
  }
  useSine  = Simulation::contains(argv, argc, "-sine");

  ThisSim sim(type);
  return sim.run(argc, argv) ? 0 : 1;

}
