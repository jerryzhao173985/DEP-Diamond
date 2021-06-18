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
 *
 ***************************************************************************/

#include <iostream>
//#include <string>

#include <fstream>
#include <algorithm>

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/operators.h>
#include <ode_robots/boxpile.h>
#include <ode_robots/sensor.h>
#include <ode_robots/speedsensor.h>
//#include <ode_robots/contactsensor.h>
//#include <ode_robots/relativepositionsensor.h>

#include <selforg/switchcontroller.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/motorbabbler.h>
#include <selforg/stl_adds.h>

#include <ode_robots/hexapod.h>
// Robot's controller
#include "../../controller/depdiamond.h"
#include "../../controller/diamond.h"
// not include the following for base controller but for another controller for contrast with diamond controller
#include "../../controller/dep.h"

// #include <stdio.h>
// #include <string>

#include <cmath>
#include "../../utils/mapgen/ppm.h"
#include "../../utils/mapgen/PerlinNoise.h"

#include <gsl/gsl_histogram.h>
// create terrain based on the map
#include <ode_robots/terrainground.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
bool track = false;
const char* trackfilename=0;
const char* loadcontroller=0;
int numwalls=0;
bool useSine=false;
bool useSwitch=false;
bool useDEP=false;
bool useDiamond = true;
// Diamond is the default controller
bool fix=false;
double noisecontrol=0;
bool whitenoise=false;
bool walkmodel=false;
bool tripod=false;
bool tripod_neg=false;
bool lateral_neg=false;
bool walkdelay=false;
bool legdelay=false;
bool boxpile=false;
int babbling=0;
double initHeight=1.2;
bool randomctrl=false;

bool topview=false;

int time_period;
int layers;

bool toLog;

const char* config_name = "config.txt";


double realtimefactor; //changeable speed for simulation graphics for recording images

double zsize;
int terrain;
bool playground; // use "playgournd" and terrain map with coonfig option "-zsize" or "-playground" same as previous experiments 
//Map Generator for Terrain Difficulty
string map_name = "map.ppm";
unsigned int seed = 237;

void map_gen(int seed) {
	// Define the size of the image
	unsigned int width = 256, height = 256;

	// Create an empty PPM image
	ppm image(width, height);

	// Create a PerlinNoise object with a random permutation vector generated with seed
	//unsigned int seed = 237;
	PerlinNoise pn(seed);

	unsigned int kk = 0;
	// Visit every pixel of the image and assign a color generated with Perlin noise
	for(unsigned int i = 0; i < height; ++i) {     // y
		for(unsigned int j = 0; j < width; ++j) {  // x
			double x = (double)j/((double)width);
			double y = (double)i/((double)height);

			// Typical Perlin noise
			double n = pn.noise(10 * x, 10 * y, 0.8);

			// Wood like structure
			//n = 20 * pn.noise(x, y, 0.8);
			//n = n - floor(n);

			// Map the values to the [0, 255] interval, for simplicity we use 
			// tones of grey
			image.r[kk] = floor(255 * n);
			image.g[kk] = floor(255 * n);
			image.b[kk] = floor(255 * n);
			kk++;
		}
	}

	// Save the image in a binary PPM file
	image.write(map_name);

}


/// Class to wrap a sensor and feed its delayed values.
class DelaySensor : public Sensor, public Configurable {
public:
  /// the sensor is supposed to be initialized (or gets initialized elsewhere before sense etc. is called)
  DelaySensor (std::shared_ptr<Sensor> src, int delay=1)
    : src(src), buffer(100), delay(delay), t(0) {
    assert(src);
    addParameter("delay", &this->delay, 0, 100, "delay in steps of sensor value");
    SensorMotorInfo smi=src->getBaseInfo();
    setBaseInfo(smi.changename("Delayed " + smi.name));
    setNamingFunc(src->getNamingFunc());
  }

  void init(Primitive* own, Joint* joint = 0) {
    // src->init(own, joint); // we do not want to init the sensor again. // debateable
  }

  bool sense(const GlobalData& globaldata){
    bool res =  src->sense(globaldata);
    buffer[t]=src->getList();
    t++;
    return res;
  }

  int getSensorNumber() const {
    return src->getSensorNumber();
  }

  std::list<sensor> getList() const {
    return buffer[t-1-delay];
  }

protected:
  std::shared_ptr<Sensor> src;
  RingBuffer<std::list<sensor> > buffer;
  int delay;
  long t;
};


class ThisSim : public Simulation {
public:
  StatisticTools stats;

  AbstractController* controller;
  OdeRobot* robot;

  double error;
  int cover[10][10];
  bool upside_down;
  Pos position;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");

    setCaption("DEP - DIAMOND                 ");
    //setCaption("DEP - DIAMOND (Der et al)");
    //setCaption ("Simulator by Martius et al");

  }

  // notice here pcc should pass (to function) by reference, or the DiamondConf wouldn't update
  void loadParamsintoConf(DiamondConf &pcc, const char* filename = "config.txt"){
    //read from config file first || there must be "=" in the config file!!
    // std::ifstream is RAII, i.e. no need to call close
    std::ifstream cFile (filename);  //"config.txt"
    if (cFile.is_open())
    {
      std::string line;
      while(std::getline(cFile, line)){
        line.erase(std::remove_if(line.begin(), line.end(), ::isspace),line.end());
        if( line.empty() || line[0] == '#' )
        {
            continue;
        }
        auto delimiterPos = line.find("=");
        auto name = line.substr(0, delimiterPos);
        auto value = line.substr(delimiterPos + 1);
        
        if(name=="l1_epsM"){ 
          pcc.params.l1_epsM = (double) std::stod(value);
        }else if(name=="l1_epsh"){
          pcc.params.l1_epsh = (double) std::stod(value);
        }else if(name=="l1_synboost"){
          pcc.params.l1_synboost = (double) std::stod(value); 
        }else if(name=="l1_urate"){
          pcc.params.l1_urate = (double) std::stod(value);
        }else if(name== "l1_indnorm"){
          pcc.params.l1_indnorm = (int) std::stoi(value);
        }else if(name=="l1_timedist"){
          pcc.params.l1_timedist = (int) std::stoi(value);
        }else if(name=="l2_epsM"){
          pcc.params.l2_epsM = (double) std::stod(value);
        }else if(name=="l2_epsh"){
          pcc.params.l2_epsh = (double) std::stod(value);
        }else if(name=="l2_synboost"){
          pcc.params.l2_synboost = (double) std::stod(value);
        }else if(name=="l2_urate"){
          pcc.params.l2_urate = (double) std::stod(value);
        }else if(name=="l2_indnorm"){
          pcc.params.l2_indnorm = (int) std::stoi(value);
        }else if(name=="l2_timedist"){
          pcc.params.l2_timedist = (int) std::stoi(value);
        
        }else if(name=="l1_learningrule"){
          pcc.params.l1_learningrule = (int) std::stoi(value);
        }else if(name=="l2_learningrule"){
          pcc.params.l2_learningrule = (int) std::stoi(value);
        
        }else if(name=="l1_time_average"){
          pcc.params.l1_time_average = (int) std::stoi(value);
        }else if(name=="l2_time_average"){
          pcc.params.l2_time_average = (int) std::stoi(value);
        
        }else{
          std::cout<< "some thing in the file cannot be assigned to the simulation controller." <<std::endl;
        }
        std::cout << name << " " << value << '\n';
      }
    }
    else 
    {
      std::cerr << "Couldn't open config file for reading.\n";
    }

    // set parameters get from the config file
    // pcc.params.l1_epsM = ;
    // pcc.params.l1_epsh = ;
    // pcc.params.l1_synboost = ;
    // pcc.params.l1_urate = ;
    // pcc.params.l1_indnorm = ;
    // pcc.params.l1_timedist = ;

    // pcc.params.l2_epsM = ;
    // pcc.params.l2_epsh = ;
    // pcc.params.l2_synboost = ;
    // pcc.params.l2_urate = ;
    // pcc.params.l2_indnorm = ;
    // pcc.params.l2_timedist = ;
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos// (Pos(-6.32561, 5.12705, 3.17278),  Pos(-130.771, -17.7744, 0));
    (Pos(1.6, 4.9945, 3.55472),  Pos(160.39, -25.768, 0));
    setCameraMode(Follow);

    // top view
    if(topview){
      setCameraHomePos(Pos(-1.14383, 10.1945, 42.7865),  Pos(179.991, -77.6244, 0));
      setCameraMode(Static);
    }
    //global.odeConfig.noise=0.05;
    //global.odeConfig.setParam("controlinterval", 1);
    //in "q learning simulation": global.odeConfig.setParam("controlinterval", 10);
    //NOTE: Find a best realtimefactor to capture the images-----
    // In the simulation simply set "-realtimefactor 10" then it is speed =  *10
    global.odeConfig.setParam("realtimefactor", realtimefactor);    //default/normal spped is 1.0

    global.odeConfig.setParam("noise", 0.0);
    global.odeConfig.setParam("controlinterval", 2);
    global.odeConfig.setParam("cameraspeed", 100);
    global.odeConfig.setParam("gravity", -9.81);
    //    global.odeConfig.setParam("realtimefactor", 1);
    setParam("UseQMPThread", false);

    if(boxpile){
      Boxpile* boxpile = new Boxpile(odeHandle, osgHandle,osg::Vec3(20.0, 20.0, 1.0), 100, 1,
                                     osg::Vec3(1, 1, 0.07), osg::Vec3(.4, .4, 0.02));
      boxpile->setColor("wall");
      boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(20 ,0 , 0));
      global.obstacles.push_back(boxpile);
    }

    // stacked Playgrounds
    double scale = 1;
    double heightoffset = 0.1;
    double height = 0;
    for (int i=0; i< numwalls; i++){
      auto playground = new Playground(odeHandle, osgHandle,
                                  osg::Vec3((4+4*i)*scale, .2, heightoffset +i*height), 1, false);
      //      playground->setColor(Color((i & 1) == 0,(i & 2) == 0,(i & 3) == 0,0.3f));
      // playground->setTexture("Images/really_white.rgb");
      playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
      global.obstacles.push_back(playground);
    }

    // Playground
    if (zsize >= .0) {
      double widthground = 20.;
      double heightground = 3.0;    //1.5
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
      playground->setColor(Color(1., 1., 1., .99));
      playground->setPosition(osg::Vec3(0, 0, .0));
      if (terrain == 0) {
        Substance substance;
        substance.toRubber(5);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
        double xboxes = 18;//19.0;
        double yboxes = 17;
        double boxdis = 3.;//.45;//1.6;
        for (double j = .0; j < xboxes; j++)
          for(double i = .0; i < yboxes; i++) {
            double xsize = 1.;//1.0;
            double ysize = 1.;//.25;
            PassiveBox* b =
              new PassiveBox(odeHandle,
                  osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
            b->setPosition(Pos( + boxdis*(i-(xboxes-1)/2.0), + boxdis*(j-(yboxes-1)/2.0), 0.01));
            global.obstacles.push_back(b);
          }
      }
      if (terrain == 1) {
        TerrainGround* terrainground =
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
              map_name, "../../utils/maze256c.ppm",
              20, 20, zsize, OSGHeightField::Red);
        terrainground->setPose(osg::Matrix::translate(.0, .0, .11));
        global.obstacles.push_back(terrainground);
      }
    }



    controller=0;
    global.configs.push_back(this);
    /*******  H E X A P O D  *********/

    int numhexapods = 1;
    for ( int ii = 0; ii< numhexapods; ii++){

      HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
      myHexapodConf.useTebiaMotors     = true;
      myHexapodConf.coxaPower          = 3.0;
      myHexapodConf.coxaSpeed          = 25;
      myHexapodConf.tebiaPower         = 1.8;
      myHexapodConf.coxaJointLimitV    = 0.5; // angle range for vertical dir. of legs
      myHexapodConf.coxaJointLimitH    = 0.8; // angle range for horizontal dir. of legs
      myHexapodConf.tebiaJointLimit    = 0.2;
      myHexapodConf.tebiaOffset        = 0;
      myHexapodConf.percentageBodyMass = .1;
      myHexapodConf.useBigBox          = false;
      myHexapodConf.tarsus             = true;
      myHexapodConf.numTarsusSections  = 1;
      myHexapodConf.useTarsusJoints    = true;
      myHexapodConf.legSpreading       = M_PI/10.0;
      // myHexapodConf.height default is 0.125

      OdeHandle rodeHandle = odeHandle;
      rodeHandle.substance.toRubber(20.); // sticky feet
      OdeRobot* hexapod = new Hexapod(rodeHandle, osgHandle.changeColor("Green"),
                                      myHexapodConf, "Hexapod_" + std::itos(ii));

      robot = hexapod;
      // on the top
      robot->place(osg::Matrix::rotate(-0.0*M_PI,0,1.0,0)*osg::Matrix::translate(0,0,initHeight+ 2*ii));

      // add delayed sensors
      std::list<SensorAttachment> sas = robot->getAttachedSensors();
      int k=0;
      for(auto& sa: sas){
        if(k%2==0){
          auto s = std::make_shared<DelaySensor>(sa.first,8);
          robot->addSensor(s);
          global.configs.push_back(s.get());
        }
        k++;
      }

      // one can also attach additional sensors
      // robot->addSensor(std::make_shared<SpeedSensor>(1.0, SpeedSensor::TranslationalRel, Sensor::X));
      // robot->addSensor(std::make_shared<SpeedSensor>(1.0, SpeedSensor::Translational, Sensor::Z));//World Coords


      AbstractController* sine = 0;
      if(useSine || useSwitch  ){
        // sine = new SineController(~0, SineController::Sine);
        sine = new MultiSineController(0x0FFF, SineController::Sine); // control the first 12
        sine->setParam("period", 25);
        sine->setParam("phaseshift", 0);
        if(noisecontrol>0)
          sine->setParam("amplitude",0);
        else{
          sine->setParam("amplitude",0.5);
        }
      }

      if(useDEP){
        DEPConf pc = DEP::getDefaultConf();
        pc.initFeedbackStrength=0.0;
        pc.calcEigenvalues = true;
        if(babbling) pc.initModel=false;

        DEP* dep = new DEP(pc);
        dep->setParam("epsM",0);
        dep->setParam("epsh",0.0);
        dep->setParam("synboost",2.2);
        dep->setParam("urate",0.05);
        dep->setParam("indnorm",1); // 0 is global normalization
        dep->setParam("timedist",4);

        controller=dep;
      } else if(useSine) {
        controller = sine;
      } else if(useDiamond){
        DiamondConf pc = Diamond::getDefaultConf();
        pc.time_period= time_period;
        pc.n_layers = layers;
        //if(babbling) pc.initModel=false;
        loadParamsintoConf(pc, config_name);
        // checck if DiamondConf is truly updated: 
        //std::cout<< pc.params.l1_epsM << std::endl;

        Diamond* diamond_controller = new Diamond(pc);
        // diamond_controller ->setParam("epsM",0.001);
        // diamond_controller ->setParam("epsh",0.01);
        // diamond_controller ->setParam("synboost",0.02);
        // diamond_controller ->setParam("urate",0.05);
        // diamond_controller ->setParam("indnorm",1); // 0 is global normalization
        // diamond_controller ->setParam("timedist",4);

        //diamond_controller->setParam("epsA",0.1); // model learning rate
        //diamond_controller->setParam("epsC",0.1); // controller learning rate
        //controller->setParam("rootE",3);    // model and contoller learn with square rooted error
        //controller->setParam("epsC", epsC); // 0.3
        //controller->setParam("Logarithmic", 0);
        
        //diamond_controller->setParam("regularization", 12);
        
        global.configs.push_back ( diamond_controller );
        controller = diamond_controller;

      }else{
        assert("unknown controller");
      }
      if(useSwitch){
        std::list<AbstractController*> ctrls = {controller,sine};
        SwitchController* sw = new SwitchController(ctrls);
        controller = sw;
      }

      One2OneWiring* wiring = new One2OneWiring( whitenoise ? (NoiseGenerator*) new WhiteUniformNoise() : (NoiseGenerator*) new ColorUniformNoise(0.1));
      // NOTE: noise is by default switched off (noise=0 in global parameters)

      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, robot, wiring);
      // add an operator to keep robot from falling over
      agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), M_PI*0.5, 30));
      // keep track of the robot's trajectory
      if(track) {
        TrackRobotConf trc = TrackRobot::getDefaultConf();
        if(trackfilename){
          trc.scene=trackfilename;
          trc.autoFilename = false;
        }
        agent->setTrackOptions(trc);
      }
      if(loadcontroller){
        if(agent->getController()->restoreFromFile(loadcontroller)){
          printf("loaded controller from %s\n", loadcontroller);
        }else{
          fprintf(stderr,"cannot load controller from %s\n", loadcontroller);
          simulation_time_reached=true;
        }
      }
      // learn model matrix
      if(babbling){
        MotorBabbler* babbler=new MotorBabbler();
        babbler->setParam("minperiod",20);
        babbler->setParam("maxperiod",200);
        babbler->setParam("amplitude",1);
        babbler->setParam("resample",200);
        global.configs.push_back(babbler);
        agent->startMotorBabblingMode(babbling,babbler);
        agent->fixateRobot(global,-1,babbling/50);
      }else{
        setModel(agent);
      }
      if(randomctrl) setRndController(agent);

      global.agents.push_back(agent);
      global.configs.push_back(agent);

      if(fix) robot->fixate(global);

      // if we use sine controller for manual control or a switching controller
      if(useSine || useSwitch){
        sine->setParam("phaseshift0", -1.2);
        sine->setParam("phaseshift1", 0);
        sine->setParam("phaseshift2", 0.8);
        sine->setParam("phaseshift3", 2);
        sine->setParam("phaseshift4", 0.8);
        sine->setParam("phaseshift5", 2);
        sine->setParam("phaseshift6", -1.2);
        sine->setParam("phaseshift7", 0);
        sine->setParam("phaseshift8", -1.2);
        sine->setParam("phaseshift9", 0);
        sine->setParam("phaseshift10",0.8);
        sine->setParam("phaseshift11",2);

        sine->setParam("amplitude0", 0.8);
        sine->setParam("amplitude2", 0.8);
        sine->setParam("amplitude4", 0.8);
        sine->setParam("amplitude6", 0.8);
        sine->setParam("amplitude8", 0.8);
        sine->setParam("amplitude10", 0.8);
      }
    }
  }

  virtual void setModel(OdeAgent* agent){
    /* Hexapod leg number
          \  /
           ||
      4--|~~~~|--5
         |    |
      2--|    |--3
         |    |
      0--|____|--1
      Each leg has up/down, front/back and tebia (knee)
      */
    // set connections into model
    DEP* dep = dynamic_cast<DEP*>(agent->getController());
    if(walkmodel && dep){
      Matrix M = dep->getM();
      // LC is transposed for historical reasons
      Matrix LC(M.getN(), M.getM());
      /// TRIPOD
      if(tripod){
        for(int k=0; k<2; k++){
          // leg 0: 3,4
          LC.val(3*3+k,0*3+k)=1;
          LC.val(4*3+k,0*3+k)=1;
          // leg 1: 2,5
          LC.val(2*3+k,1*3+k)=1;
          LC.val(5*3+k,1*3+k)=1;
          // leg 2: 1,5
          LC.val(1*3+k,2*3+k)=1;
          LC.val(5*3+k,2*3+k)=1;
          // leg 3: 0,4
          LC.val(0*3+k,3*3+k)=1;
          LC.val(4*3+k,3*3+k)=1;
          // leg 4: 0,3
          LC.val(0*3+k,4*3+k)=1;
          LC.val(3*3+k,4*3+k)=1;
          // leg 5: 1,2
          LC.val(1*3+k,5*3+k)=1;
          LC.val(2*3+k,5*3+k)=1;
        }
      }
      // subsequent legs on one side are negatively coupled (antiphasic)
      if(tripod_neg){
        for(int k=1; k<2; k++){ // front/back only
          // leg 0: - 2
          LC.val(2*3+k,0*3+k)=-1;
          // leg 1: - 3
          LC.val(3*3+k,1*3+k)=-1;
          // leg 2:  -4
          LC.val(4*3+k,2*3+k)=-1;
          // leg 3:  -5
          LC.val(5*3+k,3*3+k)=-1;
          // leg 4:  -5
          //LC.val(5*3+k,4*3+k)=-1;
          // leg 5:  -4
          //LC.val(4*3+k,5*3+k)=-1;
        }
      }
      // left and right leg pairs are negatively coupled (antiphasic)
      if(lateral_neg){
        for(int k=1; k<2; k++){// only front/back
          // leg 0: 1
          LC.val(1*3+k,0*3+k)=-1;
          // leg 1: 0
          LC.val(0*3+k,1*3+k)=-1;
          // leg 2: 3
          LC.val(3*3+k,2*3+k)=-1;
          // leg 3: 2
          LC.val(2*3+k,3*3+k)=-1;
          // leg 4: 5
          LC.val(5*3+k,4*3+k)=-1;
          // leg 5: 4
          LC.val(4*3+k,5*3+k)=-1;
        }
      }

      // delays (the delay sensors start with index 18 and for each leg we have 2,
      //  but we use only one for the connection
      if(walkdelay){
        for(int k=0; k<6; k++){
          LC.val(18+k*2+1,k*3)=1;
        }
      }

      if(legdelay){
        int k = 1;
        // leg 0: - 2
        LC.val(18+2*2+k,0*3+k)=1;
        // leg 1: - 3
        LC.val(18+3*2+k,1*3+k)=1;
        // leg 2:  -4
        LC.val(18+4*2+k,2*3+k)=1;
        // leg 3:  -5
        LC.val(18+5*2+k,3*3+k)=1;
      }

      std::cout << "apply Leg coupling" << std::endl;
      dep->setM(M+(LC^T));
    }
  }


  virtual void setRndController(OdeAgent* agent){
    DEP* dep = dynamic_cast<DEP*>(agent->getController());
    if(dep){
      Matrix C = dep->getC();
      dep->setC(C.map(random_minusone_to_one));
      printf("Set random controller matrix\n");
    }
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {

    if(globalData.sim_step==3000){
      const char* name = "screenshots/";
      startVideoRecording(name);
    }
    if(globalData.sim_step==4000){
      stopVideoRecording();
    }

    
    Position robot_position = robot->getPosition();
    const int playground = 20;
    const int bins = 10;
    double rx = robot_position.x + (playground / 2);
    double _bin_x = floor(rx / (playground / bins));
    double ry = robot_position.y + (playground /2);
    double _bin_y = floor(ry / (playground / bins));

    // if (((fmod(rx, playground / bins) < 0.2) && (bin_x != _bin_x))  ||
    //     ((fmod(ry, playground / bins) < 0.2) && (bin_y != _bin_y))) {
    //   bin_x = _bin_x;
    //   bin_y = _bin_y;
    //   if (cover[bin_x][bin_y] == 0)
    //     coverage++;
    //   cover[bin_x][bin_y]++;
    //   displacement++;
    // }      

    // this should be called at the end of the simulation but controller
    // already dead at ~ThisSim()
    // double error = 0.0; //controller->getXiAverage();
    
    //pp is an orientation matrix for the sensor!
    Pose pp = robot->getMainPrimitive()->getPose();
    matrix::Matrix pose3d;
    pose3d.set(3,3); // change matrix to be a 3x3 matrix
    for(int ii=0; ii<3; ii++){
      for(int jj=0; jj<3; jj++){
        pose3d.val(ii,jj) = pp(ii,jj);
      }
    }
    double angle_x = atan2(pose3d.val(2,1),pose3d.val(2,2));
    double angle_y = atan2(-pose3d.val(2,0), sqrt(pose3d.val(2,1)*pose3d.val(2,1)+pose3d.val(2,2)*pose3d.val(2,2)));
    double angle_z = atan2(pose3d.val(1,0), pose3d.val(0,0));


    Pos po = robot->getMainPrimitive()->getPosition();

    
    if(globalData.sim_step%100==0){
      //std::cout <<"( " << angle_x <<" , "<< angle_y << " , "<< angle_z<<")" <<std::endl;
      
      //std::cout<< pp(0,0)<< pp(1,1) <<pp(2,2)  << std::endl;
      // when I print pp(3,3) it is always 1, only the first 3*3 matrix of the pp is very useful;
      //std::cout <<"pose is: "<<"( " << pp(0,0)<<" , "<< pp(0,1)<<" , "<< pp(0,2) <<")"<< std::endl;
      //std::cout << "position is: "<<"( " << po.x()<<" , "<< po.y()<<" , "<< po.z() <<")"<< std::endl;
    }  // initial pose: ( 1 , 0 , 0); initial position: ( 0 , 0 , 1.8) ;
    
    //logging out the position and pose txt file for behaviour analysis with '-log'
    //the first 100 steps are not stable (hexapod are dropped from a high place)
    if (toLog) {
      FILE* pFile1;
      string fileName1 = "position.txt";
      pFile1 = fopen(fileName1.c_str(), "a");
      string ss1;
      ss1 = to_string(globalData.sim_step) 
          + "\t" + to_string(po.x())
          + "\t" + to_string(po.y()) 
          + "\t" + to_string(po.z()); 
      fprintf(pFile1, "%s\n", ss1.c_str());
      fclose(pFile1);

      FILE* pFile2;
      string fileName2 = "pose.txt";
      pFile2 = fopen(fileName2.c_str(), "a");
      string ss2;
      ss2 = to_string(globalData.sim_step) 
          + "\t" + to_string(pp(0,0))
          + "\t" + to_string(pp(0,1)) 
          + "\t" + to_string(pp(0,2)) 
          + "\t" + to_string(pp(1,0))
          + "\t" + to_string(pp(1,1)) 
          + "\t" + to_string(pp(1,2))
          + "\t" + to_string(pp(2,0))
          + "\t" + to_string(pp(2,1)) 
          + "\t" + to_string(pp(2,2)); 
      fprintf(pFile2, "%s\n", ss2.c_str());
      fclose(pFile2);

      FILE* pFile3;
      string fileName3 = "angle.txt";
      pFile3 = fopen(fileName3.c_str(), "a");
      string ss3;
      ss3 = to_string(globalData.sim_step) 
          + "\t" + to_string(angle_x)
          + "\t" + to_string(angle_y) 
          + "\t" + to_string(angle_z); 
      fprintf(pFile3, "%s\n", ss3.c_str());
      fclose(pFile3);
    }


    // if (pp(0,2) < -.8) {
    //   simulation_time_reached = true;
    //   upside_down = true;
    // }

    // if ((abs(position.x() - po.x()) < 0.000001) &&
    //     (abs(position.y() - po.y()) < 0.000001) &&
    //     (abs(position.z() - po.z()) < 0.000001)) {
    //   stuckness++;
    // }
    // position = po;

    
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: m","reset model matrix");
  };

  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& global,
                       int key, bool down) {
    if(down){
      switch(key){
      case 'm' : // set model
        setModel(global.agents[0]);
        break;
      }
    }
    return false;
  };

  virtual void usage() const {
    printf("  --------------- Specific settings for this simulation ------------\n");
    printf("    -walkmodel\tinitialize model to contain additional connections, see following switches:\n");
    printf("    -walkdelay\tadd connections for leg oscillation\n");
    printf("    -tripod\tadd connections for full tripod gait\n");
    printf("    -tripod_neg\tadd only negative connections for subsequent legs for tripod\n");
    printf("    -lateral_neg\tadd negative connections for lateral leg pairs\n");
    printf("    -leg_delay\tadd connections to delay subsequent legs on each side\n");
    printf("              END OF MODEL SWITCHES\n");
    printf("    -babbling STEPS\tstart with # STEPS in motor babbling mode (robot fixed)\n");
    printf("    -boxpile\tadd a boxpile to the scene\n");
    printf("    -randomctrl\tinitialize the C matrix randomly (with urate=0 this is static)\n");
    printf("  --------------- Misc stuff ------------\n");
    printf("    -numwalls N\tadd N surrounding walls around robot\n");
    printf("    -track FILENAME\trecord trajectory of robot into file\n");
    printf("    -loadcontroler FILE \tload the controller at startup\n");
    printf("    -height HEIGHT \tset initial height of robot\n");
    printf("    -noisecontrol TAU \tuse noise as motor commands with correlation length 1/TAU \n");
    printf("    -sine\tuse sine controller\n");
    printf("    -whitenoise\tuse white noise instead of colored noise for sensor noise (only active if paramter noise!=0)\n");
    printf("  --------------- Jerry's settings ------------\n");
    printf("    -diamond \tusing the DEP-Diamond model\n");
    printf("    -config FILENAME\tconfig files of all controller layer parameters to load in\n");
    printf("    -layers NUM_of_LAYER \tusing Number_of_Layer in the diamond model with config detailed parameters\n");
    printf("    -playground BOOL \tuse the playgruond box area or not\n");
    printf("    -zsize DIFFICULTY \tthe terrian map difficulty, max height \n");
    printf("    -topview\tcamera top-down view\n");
    printf("    -realtimefactor SPEED\tuse using a speed of SPEED(or max with 0) in simulation\n");
    printf("    -period TIME_AVG \ttime-sliding window(period) for the second layer Time-loop error as input \n");
    printf("    -seed SEED \tthe terrian map seed for generation with Perlin noise\n");
  };

};

int main (int argc, char **argv)
{
  int index = Simulation::contains(argv,argc,"-numwalls");
  if(index >0 && argc>index){
    numwalls=atoi(argv[index]);
  }
  track      = Simulation::contains(argv,argc,"-track")   != 0;
  index = Simulation::contains(argv,argc,"-trackfile");
  if(index >0 && argc>index){
    trackfilename=argv[index];
  }
  index = Simulation::contains(argv,argc,"-babbling");
  if(index >0 && argc>index){
    babbling=atoi(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-loadcontroller");
  if(index >0 && argc>index){
    loadcontroller=argv[index];
  }
  index = Simulation::contains(argv,argc,"-height");
  if(index >0 && argc>index){
    initHeight=atof(argv[index]);
  }
  useSine    = Simulation::contains(argv,argc,"-sine")    != 0;
  fix        = Simulation::contains(argv,argc,"-fix")     != 0;
  useSwitch  = Simulation::contains(argv,argc,"-switch")!= 0;
  whitenoise = Simulation::contains(argv,argc,"-whitenoise")!= 0;
  walkmodel  = Simulation::contains(argv,argc,"-walkmodel")!= 0;
  walkdelay  = Simulation::contains(argv,argc,"-walkdelay")!= 0;
  tripod     = Simulation::contains(argv,argc,"-tripod")!= 0;
  tripod_neg = Simulation::contains(argv,argc,"-tripod_neg")!= 0;
  lateral_neg = Simulation::contains(argv,argc,"-lateral_neg")!= 0;
  legdelay   = Simulation::contains(argv,argc,"-legdelay")!= 0;
  boxpile    = Simulation::contains(argv,argc,"-boxpile")!= 0;
  randomctrl = Simulation::contains(argv,argc,"-randomctrl")!= 0;

  useDiamond    = Simulation::contains(argv,argc,"-diamond")    != 0;

  index = Simulation::contains(argv,argc,"-noisecontrol"); // 1/(correlation length)
  if(index >0 && argc>index){
    noisecontrol=atof(argv[index]);
    useSine=true; useDEP = false;
  }
  // DEP is default controller
  if(useSine)
    useDEP = false;


  layers = 3;
  index = Simulation::contains(argv, argc, "-layers");
  if(index) 
    if(argc > index)
      layers = atoi(argv[index]);
  
  time_period = 3;
  index = Simulation::contains(argv, argc, "-period");
  if(index) 
    if(argc > index)
      time_period = atoi(argv[index]);
  
  config_name = "config.txt";
  index = Simulation::contains(argv, argc, "-config");
  if(index)
    if(argc > index)
      config_name = argv[index];
  std::cout<< std::endl << config_name << " successfully parsed from the CML!"<< std::endl;

  toLog = false;
  toLog    = Simulation::contains(argv,argc,"-log")    != 0;
  // //logFileName = "";
  // index = Simulation::contains(argv, argc, "-log");
  // if (index) {
  //   if (argc > index) {
  //      //logFileName = argv[index];
  //      toLog = true;
  //    }
  // }

  topview = false;
  topview    = Simulation::contains(argv,argc,"-topview")    != 0;

  // Negative values cancel playground
  zsize = -1.;
  index = Simulation::contains(argv, argc, "-playground");
  if(index) 
    if(argc > index)
      zsize = atof(argv[index]);

  realtimefactor = 1.;  //default using 1 speed  // 0 is full speed
  index = Simulation::contains(argv, argc, "-realtimefactor");
  if(index) 
    if(argc > index)
      realtimefactor = atof(argv[index]);
      

  terrain = 0; // boxes
  index = Simulation::contains(argv, argc, "-terrain");
  if(index) 
    if(argc > index)
      terrain = atoi(argv[index]);

  seed = 237;
  index = Simulation::contains(argv, argc, "-seed");
  if(index) 
    if(argc > index)
      seed = atoi(argv[index]);
  
  //generate map;
  std::cout<<"The seed for map generation is: "<< seed << std::endl;
  map_gen(seed);

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}
