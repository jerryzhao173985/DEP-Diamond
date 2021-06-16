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
 *                                                                 *
 ***************************************************************************/

#ifndef __ENVIRONMENT_H
#define __ENVIRONMENT_H

#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>
#include <ode_robots/seesaw.h>
#include <ode_robots/boxpile.h>

using namespace lpzrobots;
using namespace std;

class WheelTrainer : public Inspectable {
public:
  WheelTrainer(bool forTwo=false, bool upperWheel=true)
    : forTwo(forTwo), upperWheel(upperWheel) {
    radius = forTwo ? 0.3 : .2;
    width  = .1;
    height = 1.3;
    cranklength = radius;
    opposite = true;
    mass   = 1;
    handlelength = forTwo ? 0.8 : 0.2;

    crank[0]=0;
    crank[1]=0;
    crankT[0]=0;
    crankT[1]=0;
    for(int k=0; k<4; k++){
      grip[k]=0;
    }
    addInspectableValue(string("wheelangle")    + (upperWheel ? "-hand" : "-feet"), &angle, "angle of wheel");
    addInspectableValue(string("wheelanglevel") + (upperWheel ? "-hand" : "-feet"), &anglevelocity, "angle-velocity of wheel");
    stepsize = 0.01;
  }

  virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                    GlobalData& global, const osg::Matrix& pose){
    // stand
    OsgHandle standHandle = osgHandle.changeAlpha(0.5);
    if(upperWheel){
      stand = new PassiveBox(odeHandle, standHandle, osg::Vec3(1.6*width,width,height+width),0);
      stand->setTexture("Images/dusty.rgb");
      stand->setPose(TRANSM(0, radius*1.5, (height+width)/2)*pose);
    }
    // holding arms
    AbstractObstacle* o;
    o = new PassiveBox(odeHandle, standHandle, osg::Vec3(.2*width,radius*1.5,width*2),0);
    o->setTexture("Images/dusty.rgb");
    o->setPose(TRANSM(width*0.7, (radius*1.5-width)/2, height)*pose);
    global.obstacles.push_back(o);
    o = new PassiveBox(odeHandle, standHandle, osg::Vec3(.2*width,radius*1.5,width*2),0);
    o->setTexture("Images/dusty.rgb");
    o->setPose(TRANSM(-width*0.7, (radius*1.5-width)/2, height)*pose);
    global.obstacles.push_back(o);


    wheel = new Cylinder(radius, width);
    wheel->setTexture("Images/stripes.rgb");
    wheel->init(odeHandle, mass, osgHandle);
    wheel->setPose(ROTM(M_PI_2, 0, 1, 0)*TRANSM(0, 0, height)*pose);

    OsgHandle h = osgHandle.changeColor("Chrom");
    // attach cranks
    Transform* t;
    Primitive* p;
    for(int k=0; k<2; k++){
      int sign = (k==1 && opposite) ? -1 : 1;
      p = new Capsule(cranklength/10,cranklength);
      t = new Transform(wheel, p,
                        TRANSM(0,0,cranklength/2)*ROTM(sign * M_PI_2,0,1,0)*TRANSM(0,0,k==0?width:-width));
      t->init(odeHandle, 0, h);
      crank[k] = new Capsule(cranklength/10,handlelength);
      t = new Transform(wheel, crank[k],
                        TRANSM(0,0,handlelength/2)*ROTM(k*M_PI,0,1,0)*TRANSM(sign*cranklength,0,k==0?width:-width));
      t->init(odeHandle, 0, h);
      crankT[k]=t;
    }

    joint=new HingeJoint(wheel, global.environment, wheel->getPosition(), Axis(1,0,0) * pose);
    // joint=new FixedJoint(wheel, global.environment, wheel->getPosition());
    joint->init(odeHandle,h, true, width*2);
  }

  virtual void update(){
    wheel->update();
    joint->update();
    HingeJoint* j = dynamic_cast<HingeJoint*>(joint);
    if(j){
      angle = j->getPosition1();
      anglevelocity = j->getPosition1Rate();
    }
  }

  virtual void registerHand(Primitive* handorfeet, OdeHandle& odeHandle,int leftorright){
    odeHandle.addIgnoredPair(handorfeet,crankT[leftorright]);
  }

  virtual void attract(Primitive* handorfeet, int leftorright, int robot,
                       const OdeHandle& odeHandle, const OsgHandle& osgHandle, double time){
    leftorright = max(0,min(1,leftorright));
    robot       = max(0,min(1,robot));
    double posOnArm = 0;
    int lr      = leftorright;
    if(forTwo){
      posOnArm  = leftorright ? -0.3 : 0.3 ;
      lr        = robot ? 0 : 1;
    }
    int index=leftorright + 2*robot;
    auto& g = grip[index];
    if(crank[lr] && g==0){
      Pos pcorig(wheel->toGlobal(crank[lr]->getPosition()+Pos(0,0,posOnArm)));
      Pos pc = handorfeet->toGlobal(handorfeet->toLocal(pcorig)-offset[index]);
      Pos ph = handorfeet->getPosition();
      Pos diff = ph-pc;
      stepsize*=1.01;
      if(diff.length() > stepsize){      // move in steps of stepsize
        handorfeet->setPosition(ph - diff/(diff.length()+0.000001)*stepsize);
      }else{
        handorfeet->setPosition(pc);
        //        g = new FixedJoint(handorfeet,wheel, pcorig);
        g = new BallJoint(handorfeet,wheel, pcorig);
        g->init(odeHandle, osgHandle,false);
        printf("Fixed hand/foot %i, robot %i\n", leftorright, robot);
      }
    }
  }

  double radius;
  double width;
  double cranklength;
  double height;
  double mass;
  double handlelength;
  bool opposite;
  bool forTwo;
  bool upperWheel;

  Primitive* wheel;
  Primitive* crank[2];
  Transform* crankT[2];
  Joint*     grip[4];
  AbstractObstacle* stand;
  Joint* joint;
  Pos offset[4];
  double angle;
  double anglevelocity;
  double stepsize;
};


class Env {
public:
  enum EnvType { None, Normal };

  Env(EnvType t = None){
    type         = t;
    widthground  = 32; // 25.85;
    height       = 0.8;
    roughness    = 2;
    hardness     = 10;
    useColorSchema = false;

    withStool    = false;
    withStool2   = false;
    withTrainer  = false;
    withTrainer2 = false;
    withTrainer4Feet = false;
    wheelOpposite  = true;
    wheel4FeetOpposite = true;

    stool2Pose   = TRANSM(Pos(0, -1.85, 0.2+0.4))*ROTM(M_PI,0,0,1);
    stoolPose    = TRANSM(Pos(0, -0.85, 0.2+0.4));

    numSpheres  = 0;
    numBoxes    = 0;
    numCapsules = 0;
    numSeeSaws  = 0;
    numBoxPiles = 0;

    stool1 = 0;
    stool2 = 0;
    wheel4hands = 0;
    wheel4feet  = 0;
  }

  EnvType type;
  std::list<AbstractObstacle*> obstacles;
  PassiveBox* stool1;
  PassiveBox* stool2;
  WheelTrainer* wheel4hands;
  WheelTrainer* wheel4feet;

  // playground parameter
  double widthground;
  double height;
  double roughness;
  double hardness;

  bool withStool;
  Pose stoolPose;
  bool withStool2;
  Pose stool2Pose;
  bool withTrainer;
  bool withTrainer2;
  bool withTrainer4Feet;
  bool wheel4FeetOpposite;
  bool wheelOpposite;
  double wheelmass;

  bool useColorSchema; // playgrounds are black and white and get color from schema

  // obstacles
  int numSpheres;
  int numBoxes;
  int numCapsules;
  int numSeeSaws;
  int numBoxPiles;

  OdeHandle odeHandle;
  OsgHandle osgHandle;
  GlobalData* global ;


  /** creates the Environment
   */
  void create(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
              GlobalData& global, bool recreate=false){
    this->odeHandle=odeHandle;
    this->osgHandle=osgHandle;
    this->global=&global;
    if(recreate && !global.obstacles.empty()){
      // no recreation possible at the moment
      std::cerr << "cannot recreate environemnt in the simulation" << std::endl;
      return;
      // for(auto& o: global.obstacles){
      //   delete (o);
      // }
      // global.obstacles.clear();
    }
    AbstractGround* playground;
    switch (type){
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,
                                    osg::Vec3(widthground, 0.20, height));
        //     playground->setTexture("Images/really_white.rgb");
        //        playground->setGroundTexture("Images/yellow_velour.rgb");
        if(useColorSchema)
          playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3)); // was: wall.rgb
        playground->setGroundThickness(0.2);
        playground->setPosition(osg::Vec3(0,0,.0));
        Substance substance(roughness, 0.0, hardness, 0.95);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
      }
      if(withStool){
        // stool
        stool1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(0.35,0.25,0.8),0);
        stool1->setTexture("Images/stripes.rgb");
        stool1->setPose(stoolPose);
      }
      if(withStool2){
        // stool2
        stool2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(0.35,0.25,0.8),0);
        stool2->setTexture("Images/stripes.rgb");
        stool2->setPose(stool2Pose);
      }
      if(withTrainer){
        // wheel
        wheel4hands = new WheelTrainer();
        wheel4hands->cranklength=0.12;
        wheel4hands->opposite=wheelOpposite;
        wheel4hands->mass=wheelmass;
        //        wheel4hands->width=0.1;
        wheel4hands->init(odeHandle, osgHandle, global, TRANSM(0,-0.2,0.1));
      }
      if(withTrainer2){
        // wheel
        wheel4hands = new WheelTrainer(true);
        wheel4hands->cranklength=0.2;
        wheel4hands->opposite=wheelOpposite;
        wheel4hands->mass=wheelmass;
        //        wheel4hands->width=0.1;
        wheel4hands->init(odeHandle, osgHandle, global, TRANSM(0.5,-0.15,0.1));
      }
      if(withTrainer4Feet){
        // wheel
        wheel4feet = new WheelTrainer(false, false);
        wheel4feet->cranklength=0.15;
        wheel4feet->opposite=wheel4FeetOpposite;
        wheel4feet->mass=wheelmass*2;
        wheel4feet->offset[0]=Pos( 0.03,-0.04,0.08);
        wheel4feet->offset[1]=Pos(-0.03,-0.04,0.08);
        //        wheel4hands->width=0.1;
        wheel4feet->init(odeHandle, osgHandle, global, TRANSM(0.,-0.2,-0.78));
      }
      break;
    default:
      break;
    }
  }

  void placeObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      GlobalData& global){


    for(int i=0; i<numSeeSaws; i++){
      Seesaw* seesaw = new Seesaw(odeHandle, osgHandle);
      seesaw->setColor("wall");
      seesaw->setPose(ROTM(M_PI/2.0,0,0,1)*TRANSM(1, -i,.0));
      global.obstacles.push_back(seesaw);
    }

    for(int i=0; i<numBoxPiles; i++){
      Boxpile* boxpile = new Boxpile(odeHandle, osgHandle);
      boxpile->setColor("wall");
      boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(-5, -5-5*i,0.2));
      global.obstacles.push_back(boxpile);
    }

    for(int i=0; i<numSpheres; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle, osgHandle.changeColor("Monaco"), 0.2);
      s->setTexture("Images/dusty.rgb");
      s->setPosition(Pos(i*0.5-2, 3+i*4, 1.0));
      global.obstacles.push_back(s);
    }

    for(int i=0; i<numBoxes; i++){
      PassiveBox* b =
        new PassiveBox(odeHandle, osgHandle.changeColor("Weissgrau"),
                       osg::Vec3(0.4+i*0.1,0.4+i*0.1,0.4+i*0.1));

      b->setTexture("Images/light_chess.rgb");
      b->setPosition(Pos(i*0.5-5, i*0.5, 1.0));
      global.obstacles.push_back(b);
    }

    for(int i=0; i<numCapsules; i++){
      PassiveCapsule* c =
        new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      c->setPosition(Pos(i-1, -i, 1.0));
      global.obstacles.push_back(c);
    }

  }

  virtual void removeStools(GlobalData& global){
    if(stool1){
      delete stool1;
      cout << "removed stool 1" << endl;
      stool1=0;
    }
    if(stool2){
      delete stool2;
      cout << "removed stool 2" << endl;
      stool2=0;
    }

  }

  virtual void update(){
    if(wheel4hands) wheel4hands->update();
    if(wheel4feet)  wheel4feet->update();
  }

};


#endif
