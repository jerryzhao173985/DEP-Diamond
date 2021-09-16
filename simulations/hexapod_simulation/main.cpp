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

// For colored output with std::cout at the command line
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

//color code example: std::cout << RED << "hello world" << RESET << std::endl;



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
// #include <ode_robots/terrainground.h>
//using own modified version of terrainground.h
#include "terrainground.h"

#include <selforg/stl_adds.h>
#include <sys/stat.h>

#include <selforg/inspectable.h>

#include <vector>

//image processing library for screenshot images Cimg Library.
#include "../../utils/CImg.h"


#include "../../utils/fft/FFT.h"
// #include <vector>


#include <cstdio>
#include <memory>
#include <vector>

#include <libff/common/double.hpp>
#include <libfqfft/evaluation_domain/get_evaluation_domain.hpp>
using namespace libfqfft;    //using library libff in MakeFile.conf add '-lff'


#include <queue>     // get N largest index from std::vector using std::priority_queue


#include "../../utils/wavelet/wavelet_all.hpp"   //wavelet transformation


using namespace cimg_library;
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
bool with_12_delayed_sensors;


int time_period;
int layers;

bool toLog;
bool terrain_coverage;

// parameters defined to use at each timestep in addCallBack function (to do a different thing); 
bool passing_coverage_to_internal_param;

string logFileName;
double stuckness;


const char* config_name = "config.txt";


double realtimefactor; //changeable speed for simulation graphics for recording images

double zsize;
int terrain;
bool playground; // use "playgournd" and terrain map with coonfig option "-zsize" or "-playground" same as previous experiments 
//Map Generator for Terrain Difficulty
string map_name = "map.ppm";
unsigned int seed = 237;


//buffer for plot
CImg<double> colli1(1, 1000, 1, 1, 0);
CImg<double> colli2(1, 1000, 1, 1, 0);
CImg<double> colli3(1, 1000, 1, 1, 0);
CImg<double> colli4(1, 1000, 1, 1, 0);
CImg<double> colli5(1, 1000, 1, 1, 0);
CImg<double> colli6(1, 1000, 1, 1, 0);
CImg<double> invcolli1(1, 1000, 1, 1, 0);
CImg<double> invcolli2(1, 1000, 1, 1, 0);
CImg<double> invcolli3(1, 1000, 1, 1, 0);
CImg<double> invcolli4(1, 1000, 1, 1, 0);
CImg<double> invcolli5(1, 1000, 1, 1, 0);
CImg<double> invcolli6(1, 1000, 1, 1, 0);
// colli(0, 0) = 0.0;
bool plot_collision = true;



//1D FFT (&&2D FFT):
typedef std::vector<double>::iterator                 vec_d_it;
typedef std::vector<double>                           vec_d;
typedef std::vector< std::vector<double> >            vec_vec_d;
typedef std::vector<std::vector<double> >::iterator   vec_vec_d_it;
// vec_d         Frequency_R;              // real part of frequency profile
// vec_d         Frequency_I;              // imaginary part of frequency
int N_FFT = 1024;              // p=10: 2^p
// FFT* fft;   // = new FFT(N_FFT, &Frequency_R, &Frequency_I);

// HI.resize(N_FFT);  //NOTE: you should put resize() function inside a constructor!

std::vector<FFT*> fftx;            /* fft structure to compute the FFT */
int nx = 10;    // maximum buffer for time-horizons
// std::vector<FFT*> ffty;            /* fft structure to compute the FFT */
vec_vec_d         Frequency_R;              /* frequency domain, real part      - [y][x] */
vec_vec_d         Frequency_I;              /* frequency domain, imaginary part - [y][x] */

vec_vec_d         hr;              /* time domain, real part      - [y][x] */
vec_vec_d         hi;              /* time domain, imaginary part - [y][x] */


//NEW LIBRARY for FFT
CImg<double> data_original_R(1, N_FFT, 1, 1, 0);   // original
std::vector<libff::Double> f(N_FFT);                      //current dynamic(time/frequency)
std::vector<double> normalized_amp(N_FFT/2);                //normalized frequency amplitude from FFT


// bool wavelet_analysis = false;
//---Wavelet Transformation---- 
float samplerate_hz(1000.);
float frequency_min = 0.1;
float frequency_max = 500.;
float bands_per_octave = 50;
// frequency_min(this, frequency_min_, 1e-12, frequency_max_),
// frequency_max(this, frequency_max_, frequency_min_, samplerate_/2.),
wavelet::Filterbank cwt1(samplerate_hz,
                frequency_min,
                frequency_max,
                bands_per_octave);
std::size_t numbands1(cwt1.size());

wavelet::Filterbank cwt2(1000. , 0.1, 500., 50);       //(samplerate_hz, frequency_min, frequency_max, bands_per_octave);
std::size_t numbands2(cwt2.size());


bool wavelet_transform = false;
bool log_collision = false;

bool log_layer = false;

DiamondConf qc;   //used for storage text log file internal data

// lpzrobots::Joint* robotfixator;  //fix robot in the air whenusing zero rule(rule 6)


/// neuron transfer function
static double g(double z)
{
  return tanh(z);
};


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

  int bin_x, bin_y, coverage, displacement;
  int cover[10][10];
  Pos position;

  int collsion1_flag = 1;  //either 1 or 0 : default is on the floor, on the floor is 1, not on the floor is 0
  int collsion2_flag = 1;  //either 1 or 0
  int collsion3_flag = 1;  //either 1 or 0
  int collsion4_flag = 1;  //either 1 or 0
  int collsion5_flag = 1;  //either 1 or 0
  int collsion6_flag = 1;  //either 1 or 0

  double angle_xx;  //global/ [In simulation] params for(row, pitch, yaw)
  double angle_yy;
  double angle_zz;


  std::vector<int> period_of_coverage;


  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");

    setCaption("");  // if not set caption, it'll go to the default Lpzrobot caption
    // setCaption("DEP - DIAMOND                 ");
    //setCaption("DEP - DIAMOND (Der et al)");
    //setCaption ("Simulator by Martius et al");

  }


  //~ is the deconstructor, operates at the end of the simulation?
  ~ThisSim() {
    stuckness = 100. * stuckness / globalData.sim_step;
    cout << "stuck percentage: " << stuckness << endl;
    cout << "terrain coverage log: " << terrain_coverage << endl;
    cout << RED<<"Final coverage achieved: " << coverage <<RESET<< endl;
    // log the terrain data and map seed data to the file "terrain_coverage.txt"
    if (terrain_coverage) {
      bool writeHeader;
      FILE* pFile;
      string fileName = /*to_string(layers) +*/ "terrain_coverage.txt";
      string header = "coverage\tzsize\tdisplacement\tseed\ttime_steps\tstuck_percentage\tentropy\tl1_learningrate\tl2_learningrate\tl1_synboost\tl2_synboost\tl1_TIME\tl2_TIME\tperiod_coverage";
      struct stat stFileInfo;
      if ((stat(fileName.c_str(), &stFileInfo) != 0) && (!header.empty())) 
        writeHeader = true;
      else 
        writeHeader = false;
      cout << "Log file name: " << fileName << "\n";
      pFile = fopen(fileName.c_str(), "a");
      string ss;
      double coverage_entropy = 0.0;
      for (int i=0; i<10; i++){
        for (int j=0; j<10; j++){
          double prob = (double) cover[i][j] / (double) 180000;
          if(cover[i][j]!=0)
            coverage_entropy += - prob * log(prob);
        }
      }  // entropy based on the robot position x,y with respect to the map for behaviour diversity
      ss = to_string(coverage)  + "\t" + to_string(zsize) 
          + "\t" + to_string(displacement) + "\t" + to_string(seed) 
          + "\t" + to_string(globalData.sim_step) 
          + "\t" + to_string(stuckness)
          + "\t" + to_string(coverage_entropy)
          + "\t" + to_string(qc.params.l1_urate)
          + "\t" + to_string(qc.params.l2_urate)
          + "\t" + to_string(qc.params.l1_synboost)
          + "\t" + to_string(qc.params.l2_synboost)
          + "\t" + to_string(qc.params.l1_Time)
          + "\t" + to_string(qc.params.l2_Time);
      
      ss += "\t(";
      for(std::vector<int>::const_iterator i = period_of_coverage.begin(); i != period_of_coverage.end(); ++i) {
        ss += to_string(*i) + " , ";
      }
      ss += to_string(coverage)+")";
      //ss += "\t" + to_string();
      if (writeHeader) {
        fprintf(pFile, "%s\n", header.c_str());
        writeHeader = false;
      }
      fprintf(pFile, "%s\n", ss.c_str());
      fclose(pFile);
      
    }

    //Free memory.
    for(int i=0 ; i<nx ; i++) delete fftx[i];
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
        }else if(name=="l1_Time"){
          pcc.params.l1_Time = (int) std::stoi(value);
        }else if(name=="l2_Time"){
          pcc.params.l2_Time = (int) std::stoi(value);
        
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

    //initialize the evaluation metric for terrain coverage:
    bin_x = 0;
    bin_y = 0;
    coverage = 0;
    // test if I can log out the parameters as a graph curve to the GuiLogger inside this main.cpp file
    //addInspectableMatrix("coverage", &coverage, false, "terrain coverage with time");
    //the above is not working because the inspectable can only be internal parameters, thus can only be added inside
    
    displacement = 0;
    stuckness = .0;
    for (int i = 0; i < 10; i++)
      for (int j = 0; j < 10; j++)
        cover[i][j] = 0;



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



      if(with_12_delayed_sensors){
        // add delayed sensors
        std::list<SensorAttachment> sas = robot->getAttachedSensors(); //18 basic sensor for now, same as action
        int k=0;
        for(auto& sa: sas){
          if(k%2==0){
            auto s = std::make_shared<DelaySensor>(sa.first,8);  //not delay tiebia(knee), only delay the leg. 12
            robot->addSensor(s);
            global.configs.push_back(s.get());
          }
          k++;
        }

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
        qc = pc;   //save the log file data to the global variable for logging out

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

        if(pc.params.l1_learningrule==6 && pc.params.l2_learningrule==6){
          // create a fixed joint to hold the robot in the air at the beginning
          robotfixator = new lpzrobots::FixedJoint(
              robot->getMainPrimitive(),
              global.environment);
          robotfixator->init(odeHandle, osgHandle, false);
        }

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

        ((OdeRobot*)robot)->place(Pos(.0, .0, 1.));

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


    //MARK: set other parameters (Initialization) in this start() function!
    // Initialize two 2D vectors(arrays) Frequency_R(_I)
    // HR.resize(nx+1); 
    // for(vec_vec_d_it it=HR.begin() ; it!=HR.end() ; it++) it->resize(ny+1);
    Frequency_R.resize(nx);
    Frequency_I.resize(nx);
    for(vec_vec_d_it it=Frequency_R.begin() ; it!=Frequency_R.end() ; it++) it->resize(N_FFT);
    for(vec_vec_d_it it=Frequency_I.begin() ; it!=Frequency_I.end() ; it++) it->resize(N_FFT);

    // fft = new FFT(N_FFT, &Frequency_R, &Frequency_I);   //1D FFT initialization
    // ffty.reserve(nx);
    // for(int i=0 ; i<nx ; i++) ffty.push_back(new FFT(ny, &HR[i], &HI[i]));
    
    fftx.reserve(nx);
    for(int i=0 ; i<nx ; i++) fftx.push_back(new FFT(N_FFT, &Frequency_R[i], &Frequency_I[i]));


    hr.resize(nx);
    hi.resize(nx);
    for(vec_vec_d_it it=hr.begin() ; it!=hr.end() ; it++) it->resize(N_FFT);
    for(vec_vec_d_it it=hi.begin() ; it!=hi.end() ; it++) it->resize(N_FFT);


    f[0] = libff::Double(0.);
    data_original_R(0,0) = 1.;


    //wavelet initialization:
    cwt1.reset(); // Reset processing
    std::cout<< "Wavelet1 frequency bandwidth: "<< cwt1.size() << std::endl;

    cwt2.reset(); // Reset processing
    std::cout<< "Wavelet2 frequency bandwidth: "<< cwt2.size() << std::endl;
    
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
  
  // interestingly, here the param GlobalData is same as the next function param global!1
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {

    //try to use this HUBmanager for screen UI manipulation (add parameters to show)
    //this->getHUDSM()
    // Collision Handling to test which gait is in:
    // Basically of the dix feet which foot is on the floor:
    
    //collision of foot to the ground detection to determine which gait, 
    // FIXME: NOW WORKING instead of the enviornment/terrain obstacles crash!
    // std::cout<<BOLDBLUE<< robot->getAllPrimitives().size() << RESET<< std::endl; //==23
    dGeomID enviornment_ID = globalData.obstacles[0]->getMainPrimitive()->getGeom();
    dGeomID foot1_ID = robot->getAllPrimitives()[3]->getGeom();
    dGeomID foot2_ID = robot->getAllPrimitives()[6]->getGeom();
    dGeomID foot3_ID = robot->getAllPrimitives()[9]->getGeom();
    dGeomID foot4_ID = robot->getAllPrimitives()[12]->getGeom();
    dGeomID foot5_ID = robot->getAllPrimitives()[15]->getGeom();
    dGeomID foot6_ID = robot->getAllPrimitives()[18]->getGeom();
    //MARK: if the foot is stuck all in the ground, then the flag will be 0 because it is the collision with the lower leg not the foot any more!
    dGeomID lowerleg1_ID = robot->getAllPrimitives()[2]->getGeom();
    dGeomID lowerleg2_ID = robot->getAllPrimitives()[5]->getGeom();
    dGeomID lowerleg3_ID = robot->getAllPrimitives()[8]->getGeom();
    dGeomID lowerleg4_ID = robot->getAllPrimitives()[11]->getGeom();
    dGeomID lowerleg5_ID = robot->getAllPrimitives()[14]->getGeom();
    dGeomID lowerleg6_ID = robot->getAllPrimitives()[17]->getGeom();

    // //std::cout << BOLDBLUE << enviornment_ID << RESET <<std::endl;
    // int collision;
    const int N = 10;
    dContact contact1[N];
    dContact contact2[N];
    dContact contact3[N];
    dContact contact4[N];
    dContact contact5[N];
    dContact contact6[N];

    int collision1 = dCollide (enviornment_ID, foot1_ID,N,&contact1[0].geom,sizeof(dContact));
    int collision2 = dCollide (enviornment_ID, foot2_ID,N,&contact2[0].geom,sizeof(dContact));
    int collision3 = dCollide (enviornment_ID, foot3_ID,N,&contact3[0].geom,sizeof(dContact));
    int collision4 = dCollide (enviornment_ID, foot4_ID,N,&contact4[0].geom,sizeof(dContact));
    int collision5 = dCollide (enviornment_ID, foot5_ID,N,&contact5[0].geom,sizeof(dContact));
    int collision6 = dCollide (enviornment_ID, foot6_ID,N,&contact6[0].geom,sizeof(dContact));
    
    dContact lowerlegcontact1[N];
    dContact lowerlegcontact2[N];
    dContact lowerlegcontact3[N];
    dContact lowerlegcontact4[N];
    dContact lowerlegcontact5[N];
    dContact lowerlegcontact6[N];
    int lowerlegcollision1 = dCollide (enviornment_ID, lowerleg1_ID,N,&lowerlegcontact1[0].geom,sizeof(dContact));
    int lowerlegcollision2 = dCollide (enviornment_ID, lowerleg2_ID,N,&lowerlegcontact2[0].geom,sizeof(dContact));
    int lowerlegcollision3 = dCollide (enviornment_ID, lowerleg3_ID,N,&lowerlegcontact3[0].geom,sizeof(dContact));
    int lowerlegcollision4 = dCollide (enviornment_ID, lowerleg4_ID,N,&lowerlegcontact4[0].geom,sizeof(dContact));
    int lowerlegcollision5 = dCollide (enviornment_ID, lowerleg5_ID,N,&lowerlegcontact5[0].geom,sizeof(dContact));
    int lowerlegcollision6 = dCollide (enviornment_ID, lowerleg6_ID,N,&lowerlegcontact6[0].geom,sizeof(dContact));
    // std::cout<< "(" << collision1<<" , "<< collision2<<" , "<< collision3<<" , "<< collision4<<" , "<< collision5<<" , "<< collision6<<")" << std::endl;
    
    // Visualization of which foot is touching the ground! (Visualize this on the upper leg joint)
    if(collsion1_flag == 1 && collision1==0 && lowerlegcollision1==0){
      robot->getAllPrimitives()[1]->setColor(Color(1.,1.,1.,.99));    //(1,1,1) is white!!
      collsion1_flag = 0;
    }else if(collsion1_flag == 0 && (collision1>0 || lowerlegcollision1>0)){
      robot->getAllPrimitives()[1]->setColor(Color(0.,0.,0.,.99));    //(0,0,0) is black!!
      collsion1_flag = 1;
    }
    if(collsion2_flag == 1 && collision2==0 && lowerlegcollision2==0){
      robot->getAllPrimitives()[4]->setColor(Color(1.,1.,1.,.99));
      collsion2_flag = 0;
    }else if(collsion2_flag == 0 && (collision2>0 || lowerlegcollision2>0)){
      robot->getAllPrimitives()[4]->setColor(Color(0.,0.,0.,.99));
      collsion2_flag = 1;
    }
    if(collsion3_flag == 1 && collision3==0 && lowerlegcollision3==0){
      robot->getAllPrimitives()[7]->setColor(Color(1.,1.,1.,.99));
      collsion3_flag = 0;
    }else if(collsion3_flag == 0 && (collision3>0 || lowerlegcollision3>0)){
      robot->getAllPrimitives()[7]->setColor(Color(0.,0.,0.,.99));
      collsion3_flag = 1;
    }
    if(collsion4_flag == 1 && collision4==0 && lowerlegcollision4==0){
      robot->getAllPrimitives()[10]->setColor(Color(1.,1.,1.,.99));
      collsion4_flag = 0;
    }else if(collsion4_flag == 0 && (collision4>0 || lowerlegcollision4>0)){
      robot->getAllPrimitives()[10]->setColor(Color(0.,0.,0.,.99));
      collsion4_flag = 1;
    }
    if(collsion5_flag == 1 && collision5==0 && lowerlegcollision5==0){
      robot->getAllPrimitives()[13]->setColor(Color(1.,1.,1.,.99));
      collsion5_flag = 0;
    }else if(collsion5_flag == 0 && (collision5>0 || lowerlegcollision5>0)){
      robot->getAllPrimitives()[13]->setColor(Color(0.,0.,0.,.99));
      collsion5_flag = 1;
    }
    if(collsion6_flag == 1 && collision6==0 && lowerlegcollision6==0){
      robot->getAllPrimitives()[16]->setColor(Color(1.,1.,1.,.99));
      collsion6_flag = 0;
    }else if(collsion6_flag == 0 && (collision6>0 || lowerlegcollision6>0)){
      robot->getAllPrimitives()[16]->setColor(Color(0.,0.,0.,.99));
      collsion6_flag = 1;
    }
    // if(collsion1_flag==0){if(collision1>0){collsion1_flag = 1;}
    // }else{if(collision1==0){collsion1_flag = 0;}}

    // if(collsion1_flag==0){
    //   if(collision1>0 || lowerlegcollision1>0){
    //     collsion1_flag = 1;
    //   }
    // }else{
    //   if(collision1==0){
    //     if(lowerlegcollision1==0){
    //       collsion1_flag = 0;
    //     }
    //   }
    // }

    // std::cout<<  collsion1_flag <<" ,"<<collsion2_flag<<" ," << collsion3_flag<<" ,"<< collsion4_flag<<" ,"<< collsion5_flag<<" ,"<< collsion6_flag  <<std::endl;
    

    if(log_collision){
      FILE* collFile;
      string collfileName = "collision.txt";
      collFile = fopen(collfileName.c_str(), "a");
      string collss;
      collss = to_string(collision1) 
          + "\t" + to_string(collision2)
          + "\t" + to_string(collision3) 
          + "\t" + to_string(collision4)
          + "\t" + to_string(collision5)
          + "\t" + to_string(collision6); 
      fprintf(collFile, "%s\n", collss.c_str());
      fclose(collFile);
    }




    // LOG: Assertion `globalData.obstacles[0]->getMainPrimitive()!=NULL' failed.
    // std::cout <<  globalData.obstacles.size() <<std::endl;
    // std::cout << ((Primitive*) globalData.obstacles[0])->getPosition().x() << std::endl;
    // assert((TerrainGround*) globalData.obstacles[0] != NULL);


    if(passing_coverage_to_internal_param){
      matrix::Matrix param_coverage;
      param_coverage.set(1,1);
      param_coverage.val(0,0) = (double) coverage;
      Diamond* dia = dynamic_cast<Diamond*>(globalData.agents[0]->getController());
      dia->set_coverage(param_coverage);
    }
    // if(passing_coverage_to_internal_param)

    


    // print out the terrain coverage status every 5 minutes to show development of the controller:
    if(globalData.sim_step%((int) 6000*5) == 0){
      int t_cover = coverage;
      std::cout <<YELLOW<<"Coverage over " << globalData.sim_step/6000 << " minutes is: "<< t_cover  <<RESET<< std::endl;
      period_of_coverage.push_back(t_cover);
    }

    // Screenshot some pictures for 30-40s!! To check if the robot move at all!
    // if(globalData.sim_step==3000){
    //   const char* name = "screenshots/";
    //   startVideoRecording(name);
    // }
    // if(globalData.sim_step==4000){
    //   stopVideoRecording();
    // }

    
    // Position robot_position = robot->getPosition();

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

    angle_xx = angle_x;  //send these (yaw, pitcg, row) values to global params
    angle_yy = angle_y;
    angle_zz = angle_z;


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


    
    // 2D terrain coverage same as Simon's on four-wheeled robot:
    if (terrain_coverage) {
      Position robot_position = robot->getPosition();
      const int playground = 20;
      const int bins = 10;
      double rx = robot_position.x + (playground / 2);
      double _bin_x = floor(rx / (playground / bins));
      double ry = robot_position.y + (playground /2);
      double _bin_y = floor(ry / (playground / bins));

      if (((fmod(rx, playground / bins) < 0.2) && (bin_x != _bin_x))  ||
          ((fmod(ry, playground / bins) < 0.2) && (bin_y != _bin_y))) {
        bin_x = _bin_x;
        bin_y = _bin_y;
        if (cover[bin_x][bin_y] == 0)
          coverage++;
        cover[bin_x][bin_y]++;
        displacement++;
      }

    }

    
    //set title containing inportant information
    Diamond* diamond_now = dynamic_cast<Diamond*>(globalData.agents[0]->getController());
    double boost_layer1 = diamond_now->get_internal_layers()[0]->get_synboost();
    double boost_layer2 = diamond_now->get_internal_layers()[1]->get_synboost();
    int Time_layer1 = diamond_now->get_internal_layers()[0]->get_Time();
    int Time_layer2 = diamond_now->get_internal_layers()[1]->get_Time();
    position = robot->getPosition();
    char config_chars[150] = {0};    
    sprintf(config_chars, "Syn:(%.1f,%.1f) T:(%d,%d), Gait(%d,%d,%d,%d,%d,%d), Ang:(%.1f,%.1f,%.1f), Bin:(%d,%d), Z: %.2f", boost_layer1, boost_layer2, Time_layer1, Time_layer2, \
      collsion1_flag, collsion2_flag, collsion3_flag, collsion4_flag, collsion5_flag, collsion6_flag, angle_xx, angle_yy, angle_zz, bin_x, bin_y, po.z());
    std::string config_string(config_chars);
    setTitle("Cov: " + to_string(coverage) + config_string);



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


    //adding the steps collision information to the buffer
    if(globalData.sim_step%10==0){
      colli1(0, (globalData.sim_step/10)%1000) = (double) collsion1_flag+8.;
      colli2(0, (globalData.sim_step/10)%1000) = (double) collsion2_flag+6.5;
      colli3(0, (globalData.sim_step/10)%1000) = (double) collsion3_flag+5.;     //*3.5;
      colli4(0, (globalData.sim_step/10)%1000) = (double) collsion4_flag+3.5;
      colli5(0, (globalData.sim_step/10)%1000) = (double) collsion5_flag+2.;
      colli6(0, (globalData.sim_step/10)%1000) = (double) collsion6_flag+0.5;
      //inverse of the signal, see spike (when the feet leaves terrain more clearly!)
      invcolli1(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion1_flag))+8.;
      invcolli2(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion2_flag))+6.5;
      invcolli3(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion3_flag))+5.;     //*3.5;
      invcolli4(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion4_flag))+3.5;
      invcolli5(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion5_flag))+2.;
      invcolli6(0, (globalData.sim_step/10)%1000) = ((double) (1-collsion6_flag))+0.5;
    }
    if(plot_collision){
      if(globalData.sim_step%10000==0){  //after 100 seconds
        std::cout << BOLDBLACK<< "------Plot collision------" <<RESET<< std::endl;
        // CImgDisplay dispcoll1, dispcoll2;   // This is for simple vector plot display!
        // colli1.display_graph(dispcoll1, 1, 1, "X Axis", 1., 1000., "Y Axis");
        // colli2.display_graph(dispcoll2, 1, 1, "X Axis", 1., 1000., "Y Axis");

        
        CImg<unsigned char> visu(3000,1500,1,3,0), inv_visu(3000,1500,1,3,0);
        const unsigned char red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 }, yellow[] = {250,241,107}, purple[] = {221, 139, 222}, white[] = {255,255,255} ;
        CImgDisplay draw_disp(visu,"feet gait visualization"), inv_draw_disp(inv_visu,"gait visualization");
        
        while (!draw_disp.is_closed() && !inv_draw_disp.is_closed()) {
          visu.fill(0).draw_graph(colli1, white, 1 ,1 ,0., 9.5, 0);
          visu.draw_graph(colli2, purple, 1 ,1 ,0., 9.5, 0);
          visu.draw_graph(colli3, yellow, 1 ,1 ,0., 9.5, 0);
          visu.draw_graph(colli4, blue, 1 ,1 ,0., 9.5, 0);
          visu.draw_graph(colli5, green, 1 ,1 ,0., 9.5, 0);
          visu.draw_graph(colli6, red, 1 ,1 ,0., 9.5, 0).display(draw_disp); //,false,0,true);

          inv_visu.fill(0).draw_graph(invcolli1, white, 1 ,1 ,0., 9.5, 0);
          inv_visu.draw_graph(invcolli2, purple, 1 ,1 ,0., 9.5, 0);
          inv_visu.draw_graph(invcolli3, yellow, 1 ,1 ,0., 9.5, 0);
          inv_visu.draw_graph(invcolli4, blue, 1 ,1 ,0., 9.5, 0);
          inv_visu.draw_graph(invcolli5, green, 1 ,1 ,0., 9.5, 0);
          inv_visu.draw_graph(invcolli6, red, 1 ,1 ,0., 9.5, 0).display(inv_draw_disp); //,false,0,true);
        }

        CImg<double> snap, inv_snap;
        draw_disp.snapshot(snap);

        char szFileName[50] = {0};
        sprintf(szFileName, "visu/collision_flag_visualization_%d.bmp", globalData.sim_step/10000);
        // char* ss_name = "collision_flag_visualization_"+to_string(globalData.sim_step/10000)+".bmp";
        // const char* filename_visu = "collision_flag_visualization_"+to_string(globalData.sim_step/10000)+".bmp";
        snap.save_bmp(szFileName);


        inv_draw_disp.snapshot(inv_snap);
        char inv_szFileName[50] = {0};
        sprintf(inv_szFileName, "visu/inv_collision_flag_visualization_%d.bmp", globalData.sim_step/10000);
        inv_snap.save_bmp(inv_szFileName);
      }
    }

    //MARK: //COMMENT OUT THIS PREVIOUS FFT ATTEMPT
    // //adding the FFT for position (x,y) information buffer
    // // if(globalData.sim_step%10==0){
    // Diamond* diamond_fft = dynamic_cast<Diamond*>(globalData.agents[0]->getController());
    // int N1 = globalData.sim_step / N_FFT; 
    // Frequency_R[N1][globalData.sim_step%N_FFT] = diamond_fft->get_internal_layers()[0]->getLastMotorValues().val(10,0);
    // Frequency_I[N1][globalData.sim_step%N_FFT] = 0.0;
    // // std::cout<< Frequency_R[N1][globalData.sim_step%N_FFT] << std::endl;
    // // }
    // /*Direct FFT transform. The algorithm computes the spectrum for the time-domain signal 
    // in the real and imag vectors, and stores the result in these same vectors. -->direct()*/
    // bool plot_FFT = false; //plot_collision;

    // if(plot_FFT){
    //   if(globalData.sim_step%(N_FFT/* *10 */)==0){  //after FFT/10 seconds
    //     //store the time-domain data plot first before doing FFT direct()
    //     int x;
    //     vec_d_it it;
    //     for(it=hr[N1].begin(), x=0 ; it!=hr[N1].end() ; it++, x++) *it = Frequency_R[N1][x];
    //     for(it=hi[N1].begin(), x=0 ; it!=hi[N1].end() ; it++, x++) *it = Frequency_I[N1][x]; 
        
    //     fftx[N1]->direct();
        
    //     std::vector<double>* ss; 
    //     std::vector<double>* sss;
    //     ss =  fftx[N1]->get_real();
    //     sss =  fftx[N1]->get_imag();
        
        
    //     std::cout << BOLDBLACK<< "------Plot FFT------" <<RESET<< std::endl;        
    //     CImg<unsigned char> visu_fft(500,400,1,3,0);
    //     const unsigned char red1[] = { 255,0,0 }, blue1[] = {0,0,255};
    //     CImgDisplay draw_disp_fft(visu_fft,"Frequency plot visualization");
        
    //     //buffer for plot
    //     CImg<double> data_fft_R(1, N_FFT, 1, 1, 0); 
    //     CImg<double> data_fft_I(1, N_FFT, 1, 1, 0);
        
    //     for(int i=0; i<N_FFT; i++){
    //       data_fft_R(0,i) = (double) ss->at(i);  //Frequency_R[N1][i];  //hr[N1][i];    //Frequency_R[N1][i];
    //       // std::cout<< Frequency_R[N1][i] << std::endl;
    //       //std::cout<< hr[N1][i] << std::endl;
    //       std::cout<< ss->at(i) <<std::endl;
    //       data_fft_I(0,i) = (double) sss->at(i);  //Frequency_I[N1][i];  //hi[N1][i];    //Frequency_I[N1][i];
    //     }
    //     while (!draw_disp_fft.is_closed()) {
    //       visu_fft.fill(0).draw_graph(data_fft_R, red1, 1 ,1 ,-1., 1., 0).draw_graph(data_fft_I, blue1, 1 ,1 ,-1., 1., 0).display(draw_disp_fft); //,false,0,true);
    //     }
        
    //     //clear the vector buffer and set to 0 again
    //     // Frequency_R.clear();
    //     // Frequency_I.clear();
    //     // Frequency_R.resize(N_FFT);
    //     // Frequency_I.resize(N_FFT);

    //   }
    // } 
    

    int N1 = globalData.sim_step / N_FFT; 
    
    // f.push_back(libff::Double(last_motor_value));   //if here use push_back, later on should use clear()

    // if (globalData.sim_step%10==0) std::cout<< diamond_fft->get_internal_layers()[0]->getLastMotorValues().val(0,0) <<std::endl;
    // if (globalData.sim_step%10==0) std::cout<< Frequency_R[N1][globalData.sim_step%N_FFT] <<std::endl;
    bool plot_FFT = plot_collision;
    if(plot_FFT){
      if(globalData.sim_step%(N_FFT)==0){  //after FFT/10 seconds
        
        // MARK: Wrap the following wavelet Transform into CML options 'w/W'
        // CImg<double> wave_amp(1, cwt.size(), 1, 1, 0);
        // CImg<unsigned char> wave_visu(2048,1024,1,3,0);
        // const unsigned char red1[] = { 255,0,0 }, blue1[] = {0,0,255};
        // CImgDisplay wave_disp(wave_visu,"wave plot visualization");  
        
        // double wave_sum = 0.0;
        // std::cout <<"(";
        // for (unsigned int band=0; band<numbands; band++) {
        //   std::complex<double> result = cwt.result_complex[band];
        //   double amp = std::sqrt(result.real() * result.real() + result.imag() * result.imag());
        //   std::cout << amp << " , ";
        //   wave_amp(0, band) = amp;
        //   wave_sum += amp;
        // }
        // std::cout <<")" << std::endl<< std::endl;

        // for (unsigned int band=0; band<numbands; band++) { wave_amp(0, band) = wave_amp(0, band) / wave_sum; }

        // while (!wave_disp.is_closed()) {
        //   wave_visu.fill(255).draw_graph(wave_amp, red1, 1 ,1 ,0., 0.05, 0).display(wave_disp);  
        // }

        


        //store the time-domain data plot first before doing FFT direct()
        // f.resize(N_FFT);
        // for(int i=0; i<N_FFT; i++){
        //   data_original_R(0,i) = Frequency_R[N1][i];  //(1 , 3)
        //   f[i]=libff::Double(Frequency_R[N1][i]);
        // }
        //FFT using NEW libff library
        size_t m = N_FFT;
        /* Get evaluation domain */
        std::shared_ptr<evaluation_domain<libff::Double> > domain = get_evaluation_domain<libff::Double>(m);
        /* FFT */
        domain->FFT(f);
        // int Ii=1000;   //for test value
        // printf("Last Magnitude: %0.1f, (%0.1f, %0.1f)\n", std::sqrt(f[Ii].val.real() *f[Ii].val.real() +f[Ii].val.imag()* f[Ii].val.imag()), f[Ii].val.real(), f[Ii].val.imag());
        
        //MARK: Calculate some data evaluation metrics before doing the plot!
        //Calculate the mean frequency to determine a rough frequency of this 10s MAYBE can also calculater the frequency entropy!
        int MEDIUM_SAMPLE = 500;
        int SAMPLE_START = (int) (N_FFT - MEDIUM_SAMPLE)/2;
        std::vector<double> medium_freq(MEDIUM_SAMPLE);
        for(int i=0; i<MEDIUM_SAMPLE; i++)  medium_freq[i] = std::sqrt(f[SAMPLE_START+i].val.real() *f[SAMPLE_START+i].val.real() +f[SAMPLE_START+i].val.imag()* f[SAMPLE_START+i].val.imag()) * .1;
        double average = std::accumulate( medium_freq.begin(), medium_freq.end(), 0.0) / (double) medium_freq.size(); 
        std::cout << "The Mean Medium Frequency is: " << average << std::endl;
        
        // double LogEntropy = 0.0;
        double weighted_freq = 0.0;
        double amp_sum = std::accumulate( medium_freq.begin(), medium_freq.end(), 0.0);
        for(int i=0; i<MEDIUM_SAMPLE; i++) {
          weighted_freq += (medium_freq[i]/amp_sum) * ((double) (SAMPLE_START+i));
          // LogEntropy += (medium_freq[i]/amp_sum) * std::log((medium_freq[i]/amp_sum));
        }
        std::cout << "The LOG Entropy for Medium Frequency is: " << weighted_freq << std::endl;    
        
        // GET N largest Frequenmcy in the middle!
        int N_LARGEST = 10;    // number of indices we need
        std::priority_queue<std::pair<double, int> > q;
        for (int i = 0; i < medium_freq.size(); ++i) {
          q.push(std::pair<double, int>(medium_freq[i], i));
        }
        for (int i = 0; i < N_LARGEST; ++i) {
          int ki = q.top().second;
          std::cout << "index[" << i << "] = " << ki << std::endl;
          q.pop();
        }

        //cluster these top areas from i~(200, 300), with each section a window of 10 within calculate the mean
        double window_sum;
        for(int i=0; i<10; i++){
          vec_d_it first = medium_freq.begin() + 200+i*10;         //typedef std::vector<double>::iterator  vec_d_it;
          vec_d_it last = medium_freq.begin() + 200+(i+1)*10 + 1;  //get the subvector start and end pointer
          std::vector<double> subvector_freq(first, last);
          window_sum =   std::accumulate( subvector_freq.begin(), subvector_freq.end(), 0.0);
          std::cout << "SUM of window ("<<SAMPLE_START+200+i*10<<", "<<SAMPLE_START+200+(i+1)*10<<") is: "<< window_sum  <<std::endl;
        }


      
        std::cout << BOLDBLACK<< "------Plot FFT------" <<RESET<< std::endl;        
        CImg<unsigned char> visu_fft(2048,1024,1,3,0);
        const unsigned char red1[] = { 255,0,0 }, blue1[] = {0,0,255};
        CImgDisplay draw_disp_fft(visu_fft,"Frequency plot visualization");  
        //buffer for plot
        CImg<double> data_fft_R(1, N_FFT, 1, 1, 0); 
        CImg<double> data_fft_I(1, N_FFT, 1, 1, 0);
        CImg<double> data_fft_amp(1, N_FFT, 1, 1, 0);  //Sufficient for analysis: Amplitute for conplex vector if you only care about intensity
        
        // double check_summation_normalization = 0.0;
        for(int i=0; i<N_FFT; i++){
          data_fft_R(0,i) =  f[i].val.real();   
          data_fft_I(0,i) =  f[i].val.imag();
          if(i<N_FFT/2){
            data_fft_amp(0,i) =  std::sqrt(f[i].val.real() *f[i].val.real() +f[i].val.imag()* f[i].val.imag()) ;
            data_fft_amp(0,i) =  (data_fft_amp(0,i)/ (double) N_FFT ); //* 2.;  //*= 0.1   //Normalize the amplitude to (0,1), then enlarge to (0, 2) for visualization!
            normalized_amp[i] =  data_fft_amp(0,i);  // store this normalized value in order to calculate frequency entropy later 
            // check_summation_normalization += data_fft_amp(0,i); //Correct! // The summation before *2 is summed up to approx 1;
            data_fft_amp(0,i) =  data_fft_amp(0,i) * 10.;  // Purely for visualization
          }
          if(i>=N_FFT/2)  data_fft_amp(0,i) = 0.;
        }

        std::cout<<"Accumulation of normalized frequncy amplitude: " <<   std::accumulate( normalized_amp.begin(), normalized_amp.end(), 0.0)   <<std::endl;
        double LogEntropy = 0.0;
        for(int i=0; i<N_FFT/2; i++) {
          LogEntropy += normalized_amp[i] * std::log(normalized_amp[i]);
        }
        LogEntropy = -LogEntropy;   //Negative LOG entropy is positive value;
        std::cout<<"LogEntropy of normalized frequncy amplitude: " <<   LogEntropy   <<std::endl;
        

        while (!draw_disp_fft.is_closed()) {
          visu_fft.fill(255).draw_graph(data_original_R, red1, 1 ,1 ,0., 2., 0).draw_graph(data_fft_amp, blue1, 1 ,1 ,0., 2., 0).display(draw_disp_fft);  //NOTE: the y_min should be greater than 0.0!!
          //draw_graph(data_fft_R, red1, 1 ,1 ,-5., 5., 0).draw_graph(data_fft_I, blue1, 1 ,1 ,-5., 5., 0).display(draw_disp_fft); //,false,0,true);
        }

        // f.clear(); 
        
        
        

      }
    }

    //Make assigning the value after the plot! To assign new values to 0 location.
    Diamond* diamond_fft = dynamic_cast<Diamond*>(globalData.agents[0]->getController());
    double last_motor_value = diamond_fft->get_internal_layers()[0]->getLastMotorValues().val(0,0);
    // Frequency_R[N1][globalData.sim_step%N_FFT] = last_motor_value;
    data_original_R(0,globalData.sim_step%N_FFT) = last_motor_value + 1.0;   // Here +1.0 is only for visualization of the original time-domain sin curve 
    f[globalData.sim_step%N_FFT] = libff::Double(last_motor_value);

   /* std::cout<<  diamond_fft->get_internal_layers()[0]->getLastMotorValues().val(0,0) << " , "<< diamond_fft->get_internal_layers()[1]->getLastMotorValues().val(0,0) 
      <<" , "<< ((diamond_fft->get_internal_layers()[0]->getC() * diamond_fft->get_internal_layers()[0]->get_x()+diamond_fft->get_internal_layers()[0]->geth()).map(g)).val(0,0) <<" , "  <<std::endl;
   */


    
    
    if(wavelet_transform){  //DEBUG: The following takes too much RAM and make the sim speed down even to 0.3
      double value = diamond_fft->get_internal_layers()[0]->getLastMotorValues().val(0,0);// Get data from a stream
      cwt1.update(value);
      cwt2.update( diamond_fft->get_internal_layers()[1]->getLastMotorValues().val(0,0) );
    }



    if(globalData.sim_step >= 30000){  //after 5 minutes record the sensor values!
      //logging out layer informations (actions/sensors) for layer 1 and layer 2
      if (log_layer) {
        FILE* pFilelayer1;
        string fileNamelayer1 = "layer1.txt";
        pFilelayer1 = fopen(fileNamelayer1.c_str(), "a");
        string sslayer1;
        sslayer1 =  "";
        
        for(int aa = 0; aa<diamond_fft->getMotorNumber(); aa++){
          sslayer1 += "\t" + to_string(diamond_fft->get_internal_layers()[0]->get_x().val(aa,0));  // x or y is a colum vector
        }
        fprintf(pFilelayer1, "%s\n", sslayer1.c_str());
        fclose(pFilelayer1);

        
        FILE* pFilelayer2;
        string fileNamelayer2 = "layer2.txt";
        pFilelayer2 = fopen(fileNamelayer2.c_str(), "a");
        string sslayer2 = "";
        for(int aa = 0; aa<diamond_fft->getMotorNumber(); aa++){
          sslayer2 += "\t" + to_string(diamond_fft->get_internal_layers()[1]->get_x().val(aa,0));  // x or y is a colum vector
        }
        fprintf(pFilelayer2, "%s\n", sslayer2.c_str());
        fclose(pFilelayer2);
      }
    }  
 



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
      
      case 'p' : { // print out the model matrix (or the controller matrix) for every layer
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        
        Matrix M1 = diamond->get_internal_layers()[0]->getM();
        Matrix M2 = diamond->get_internal_layers()[1]->getM();
        
        std::cout << "M1: "<< std::endl << M1 << std::endl;
        std::cout << "M2: "<< std::endl << M1 << std::endl;
        break; }  //the aim of the '{}' is to keep the local variables inside the case to be private!

      case 'e' :  //print out Euler Angles
        std::cout <<"( " << angle_xx <<" , "<< angle_yy << " , "<< angle_zz<<")" <<std::endl;
        break;
      
      case 'i':{
        std::cout << BOLDBLACK<< "------Plot M1------" <<RESET<< std::endl;
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix M1 = diamond->get_internal_layers()[0]->getM();
        int all_items = (M1.getM()) * (M1.getN());

        CImg<double> values1(1, all_items, 1, 1, 0);

        for (int i1 = 0; i1 < all_items; ++i1)
        {
          int j = i1%(M1.getM());  
          values1(0, i1) = M1.val((i1-j)/M1.getM() ,j);
        }

        const char *const formula = "M1 matrix all items";
        const float x0 = (double) 1. ;
        const float x1 = (double) all_items;
        const int resolution = all_items;
        const unsigned int nresolution = resolution>1 ? resolution : 5000;
        const unsigned int plot_type = 1;
        const unsigned int vertex_type = 1;

        CImgDisplay disp1;
        CImg<double> values2;
        values1.display_graph(disp1, plot_type, vertex_type, "X Axis", x0, x1, "Y Axis");
        disp1.snapshot(values2);
        values2.save_bmp("Matrix_M.bmp");
        
        break;}


        case 'I':{
        std::cout << BOLDBLACK<< "------Plot C1------" <<RESET<< std::endl;
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix M1 = diamond->get_internal_layers()[0]->getC();
        int all_items = (M1.getM()) * (M1.getN());
        CImg<double> values1(1, all_items, 1, 1, 0);

        for (int i1 = 0; i1 < all_items; ++i1)
        {
          int j = i1%(M1.getM());  
          values1(0, i1) = M1.val((i1-j)/M1.getM() ,j);
        }

        const char *const formula = "C1 matrix all items";
        const float x0 = (double) 1. ;
        const float x1 = (double) all_items;
        const int resolution = all_items;
        const unsigned int nresolution = resolution>1 ? resolution : 5000;
        const unsigned int plot_type = 1;
        const unsigned int vertex_type = 1;

        CImgDisplay disp1;
        CImg<double> values2;
        values1.display_graph(disp1, plot_type, vertex_type, "X Axis", x0, x1, "Y Axis");
        disp1.snapshot(values2);
        values2.save_bmp("matrix_C.bmp");
        
        break;}


        case 'l':{
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix C1 = diamond->get_internal_layers()[0]->getC_update();
        
        Matrix LC(C1.getM(), C1.getN());
        for(int i=0; i<C1.getM(); i++){
          for(int j=0; j<C1.getN(); j++){
            LC.val(i,j) = 1.0; //C1.val(i,j) * 2.0;
            //MARK: Here cannot set *2 times, because after normalize it is still the same 
          }
        }
        diamond->get_internal_layers()[0]->setC_update(LC);
        std::cout << BOLDMAGENTA<< "C1(update) enlarged by 2.0" <<RESET<< std::endl;
        break;}

        case 'L':{
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix C1 = diamond->get_internal_layers()[0]->getM();
        
        Matrix LC(C1.getM(), C1.getN());
        for(int i=0; i<C1.getM(); i++){
          for(int j=0; j<C1.getN(); j++){
            LC.val(i,j) = C1.val(i,j) * 2.0;
          }
        }
        diamond->get_internal_layers()[0]->setM(LC);
        std::cout << BOLDMAGENTA<< "M1 enlarged by 2.0" << RESET << std::endl;
        break;}


        /* Hexapod leg number
            \  /
              ||
        4--|~~~~|--5
            |    |
        2--|    |--3
            |    |
        0--|____|--1
        Each leg has up/down, front/back and tebia (knee)*/  
        // set connections into model      
        case 't':{
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix M = diamond->get_internal_layers()[0]->getM();
        // LC is transposed for historical reasons
        Matrix LC(M.getN(), M.getM());
        /// TRIPOD
        for(int k=0; k<2; k++){    //   k=0  ,   k=1
          // leg 0: 3,4
          LC.val(3*3+k,0*3+k)=1.;   // (9, 0) , (10, 1)
          LC.val(4*3+k,0*3+k)=1.;   // (12, 0), (13, 1)
          // leg 1: 2,5
          LC.val(2*3+k,1*3+k)=-1.;   // (6 ,3) , (7, 4)
          LC.val(5*3+k,1*3+k)=-1.;   // (15,3) , (16,4)
          // leg 2: 1,5
          LC.val(1*3+k,2*3+k)=1.;   // (3, 6) , (4, 7)
          LC.val(5*3+k,2*3+k)=1.;   // (15,6) , (16,7)
          // leg 3: 0,4
          LC.val(0*3+k,3*3+k)=-1.;   // (0, 9) , (1, 10)
          LC.val(4*3+k,3*3+k)=-1.;   // (12,9) , (13,10)
          // leg 4: 0,3
          LC.val(0*3+k,4*3+k)=1.;   // (0,12) , (1, 13)
          LC.val(3*3+k,4*3+k)=1.;   // (9,12) , (10,13)
          // leg 5: 1,2
          LC.val(1*3+k,5*3+k)=-1.;   // (3,15) , (4, 16)
          LC.val(2*3+k,5*3+k)=-1.;   // (6,15) , (7, 16)
        }
      
        diamond->get_internal_layers()[0]->setM((LC^T));  //setM(M+(LC^T));
        break;}

        case 'T':{   //second layer
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        Matrix M = diamond->get_internal_layers()[1]->getM();
        Matrix LC(M.getN(), M.getM());
        for(int k=0; k<2; k++){    //   k=0  ,   k=1
          LC.val(3*3+k,0*3+k)=.5;   // (9, 0) , (10, 1)
          LC.val(4*3+k,0*3+k)=.5;   // (12, 0), (13, 1)
          LC.val(2*3+k,1*3+k)=.5;   // (6 ,3) , (7, 4)
          LC.val(5*3+k,1*3+k)=.5;   // (15,3) , (16,4)
          LC.val(1*3+k,2*3+k)=.5;   // (3, 6) , (4, 7)
          LC.val(5*3+k,2*3+k)=.5;   // (15,6) , (16,7)
          LC.val(0*3+k,3*3+k)=.5;   // (0, 9) , (1, 10)
          LC.val(4*3+k,3*3+k)=.5;   // (12,9) , (13,10)
          LC.val(0*3+k,4*3+k)=.5;   // (0,12) , (1, 13)
          LC.val(3*3+k,4*3+k)=.5;   // (9,12) , (10,13)
          LC.val(1*3+k,5*3+k)=.5;   // (3,15) , (4, 16)
          LC.val(2*3+k,5*3+k)=.5;   // (6,15) , (7, 16)
        }
        diamond->get_internal_layers()[1]->setM((LC^T));  //setM(M+(LC^T));
        break;}

        case 'x':{
        if (robotfixator) {
          std::cout << "dropping robot" << std::endl;
          delete robotfixator;
          robotfixator = NULL;
        }
        break;}

        case 'b':{  //increase the synboost
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double boost = diamond->get_internal_layers()[0]->get_synboost();
        diamond->get_internal_layers()[0]->set_synboost(boost+0.05);
        std::cout<< BLUE << "Now the synboost for "<<RED<<"layer 1" << BLUE<<" is: " << boost+0.05 << RESET<< std::endl;
        break;}

        case 'B':{
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double boost = diamond->get_internal_layers()[1]->get_synboost();
        diamond->get_internal_layers()[1]->set_synboost(boost+0.05);
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 2" << BLUE<<" is: " << boost+0.05 << RESET<< std::endl;
        break;}

        case 'a':{  // both layers increase the time period T (Time)
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        int TTime = diamond->get_internal_layers()[0]->get_Time();
        diamond->get_internal_layers()[0]->set_Time(TTime+5);
        diamond->get_internal_layers()[1]->set_Time(TTime+5);
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 1" << BLUE<<" is: " << TTime+5 << RESET<< std::endl;
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 2" << BLUE<<" is: " << TTime+5 << RESET<< std::endl;
        break;}

        case 'A':{  // both layers decrease the time period T (Time)
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        int TTime = diamond->get_internal_layers()[1]->get_Time();
        diamond->get_internal_layers()[0]->set_Time(TTime-5);
        diamond->get_internal_layers()[1]->set_Time(TTime-5);
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 1" << BLUE<<" is: " << TTime-5 << RESET<< std::endl;
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 2" << BLUE<<" is: " << TTime-5 << RESET<< std::endl;
        break;}

        case 'd':{  //decrease the synboost
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double boost = diamond->get_internal_layers()[0]->get_synboost();
        diamond->get_internal_layers()[0]->set_synboost(boost-0.05);
        std::cout<< BLUE << "Now the synboost for "<<RED<<"layer 1" << BLUE<<" is: " << boost-0.05 << RESET<< std::endl;
        break;}

        case 'D':{
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double boost = diamond->get_internal_layers()[1]->get_synboost();
        diamond->get_internal_layers()[1]->set_synboost(boost-0.05);
        std::cout<< BLUE << "Now the synboost for "<<RED<< "layer 2" << BLUE<<" is: " << boost-0.05 << RESET<< std::endl;
        break;}

        case 'c':{  //Lower case for layer 1: print out the config
        std::cout << BOLDCYAN << "This is the config for "<<RED<< "Layer 1: " <<RESET << std::endl;
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        diamond->get_internal_layers()[0]->printConf();
        break;}
        case 'C':{  //Capital case for layer 2: print out the config
        std::cout << BOLDCYAN << "This is the config for "<<RED<< "Layer 2: " <<RESET << std::endl;
        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        diamond->get_internal_layers()[1]->printConf();
        break;}


        case 'w':{    //Lower case for layer 1: do the Wavelet Transformation
        
        CImg<double> wave_amp(1, cwt1.size(), 1, 1, 0);
        CImg<unsigned char> wave_visu(2048,1024,1,3,0);
        const unsigned char red1[] = { 255,0,0 }, blue1[] = {0,0,255};
        CImgDisplay wave_disp(wave_visu,"wave plot visualization");  
        
        double wave_sum = 0.0;
        // std::cout <<"(";
        for (unsigned int band=0; band<numbands1; band++) {
          std::complex<double> result = cwt1.result_complex[band];
          double amp = std::sqrt(result.real() * result.real() + result.imag() * result.imag());
          // std::cout << amp << " , ";
          wave_amp(0, band) = amp;
          wave_sum += amp;
        }
        // std::cout <<")" << std::endl<< std::endl;

        for (unsigned int band=0; band<numbands1; band++) { wave_amp(0, band) = wave_amp(0, band) / wave_sum; }

        while (!wave_disp.is_closed()) {
          wave_visu.fill(255).draw_graph(wave_amp, red1, 1 ,1 ,0., 0.05, 0).display(wave_disp);  
        }

        CImg<double> snap;
        wave_disp.snapshot(snap);
        char szFileName[50] = {0};

        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double layer1_boost = diamond->get_internal_layers()[0]->get_synboost();
        
        sprintf(szFileName, "wave/boost_%.2f_level1_%d.bmp", layer1_boost, global.sim_step);
        snap.save_bmp(szFileName);

        break;}

        case 'W':{
        CImg<double> wave_amp(1, cwt2.size(), 1, 1, 0);
        CImg<unsigned char> wave_visu(2048,1024,1,3,0);
        const unsigned char red1[] = { 255,0,0 }, blue1[] = {0,0,255};
        CImgDisplay wave_disp(wave_visu,"wave plot visualization");  
        
        double wave_sum = 0.0;
        // std::cout <<"(";
        for (unsigned int band=0; band<numbands2; band++) {
          std::complex<double> result = cwt2.result_complex[band];
          double amp = std::sqrt(result.real() * result.real() + result.imag() * result.imag());
          // std::cout << amp << " , ";
          wave_amp(0, band) = amp;
          wave_sum += amp;
        }
        // std::cout <<")" << std::endl<< std::endl;

        for (unsigned int band=0; band<numbands2; band++) { wave_amp(0, band) = wave_amp(0, band) / wave_sum; }

        while (!wave_disp.is_closed()) {
          wave_visu.fill(255).draw_graph(wave_amp, red1, 1 ,1 ,0., 0.05, 0).display(wave_disp);  
        }

        CImg<double> snap;
        wave_disp.snapshot(snap);
        char szFileName[50] = {0};

        Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        double layer2_boost = diamond->get_internal_layers()[1]->get_synboost();
        
        sprintf(szFileName, "wave/boost_%.2f_level2_%d.bmp", layer2_boost, global.sim_step);
        snap.save_bmp(szFileName);

        break;}


        case 'g':{
        float grav = global.odeConfig.getParam("gravity");
        global.odeConfig.setParam("gravity", grav+0.5);
        break;}

        case 'G':{
        float grav = global.odeConfig.getParam("gravity");
        global.odeConfig.setParam("gravity", grav-0.5);
        break;}

        case 's':{
        global.odeConfig.setParam("realtimefactor", 0);    //default/normal spped is 1.0
        std::cout << "Simulation speed is: " << global.odeConfig.getParam("realtimefactor") <<std::endl;
        break;}

        case 'S':{
        global.odeConfig.setParam("realtimefactor", 1.0);    //default/normal spped is 1.0
        std::cout << "Simulation speed is: " << global.odeConfig.getParam("realtimefactor") <<std::endl;
        break;}

        case 'v':{
        int con_interval = global.odeConfig.getParam("controlinterval");
        global.odeConfig.setParam("controlinterval", con_interval+1);
        std::cout << "ControlInterval is: " << con_interval+1 << std::endl;
        break;}
        case 'V':{
        int con_interval = global.odeConfig.getParam("controlinterval");
        global.odeConfig.setParam("controlinterval", con_interval-1);
        std::cout << "ControlInterval is: " << con_interval-1 << std::endl;
        break;}


        // case 'T':{
        // Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        // Matrix M = diamond->get_internal_layers()[0]->getM();
        
        // diamond->get_internal_layers()[0]->setM(LC);
        // break;}


        // case 'l':{
        // Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        // Matrix M = diamond->get_internal_layers()[0]->getM();
        
        // diamond->get_internal_layers()[0]->setM(LC);
        // break;}


        // case 'L':{
        // Diamond* diamond = dynamic_cast<Diamond*>(global.agents[0]->getController());
        // Matrix M = diamond->get_internal_layers()[0]->getM();
        
        // diamond->get_internal_layers()[0]->setM(LC);
        // break;}




      //   if(walkmodel && dep){
      //   Matrix M = dep->getM();
      //   // LC is transposed for historical reasons
      //   Matrix LC(M.getN(), M.getM());
      //   /// TRIPOD
      //   if(tripod){
          
      //   }
      //   // subsequent legs on one side are negatively coupled (antiphasic)
      //   if(tripod_neg){
      //     for(int k=1; k<2; k++){ // front/back only
      //       // leg 0: - 2
      //       LC.val(2*3+k,0*3+k)=-1;
      //       // leg 1: - 3
      //       LC.val(3*3+k,1*3+k)=-1;
      //       // leg 2:  -4
      //       LC.val(4*3+k,2*3+k)=-1;
      //       // leg 3:  -5
      //       LC.val(5*3+k,3*3+k)=-1;
      //       // leg 4:  -5
      //       //LC.val(5*3+k,4*3+k)=-1;
      //       // leg 5:  -4
      //       //LC.val(4*3+k,5*3+k)=-1;
      //     }
      //   }
      //   // left and right leg pairs are negatively coupled (antiphasic)
      //   if(lateral_neg){
      //     for(int k=1; k<2; k++){// only front/back
      //       // leg 0: 1
      //       LC.val(1*3+k,0*3+k)=-1;
      //       // leg 1: 0
      //       LC.val(0*3+k,1*3+k)=-1;
      //       // leg 2: 3
      //       LC.val(3*3+k,2*3+k)=-1;
      //       // leg 3: 2
      //       LC.val(2*3+k,3*3+k)=-1;
      //       // leg 4: 5
      //       LC.val(5*3+k,4*3+k)=-1;
      //       // leg 5: 4
      //       LC.val(4*3+k,5*3+k)=-1;
      //     }
      //   }

      //   // delays (the delay sensors start with index 18 and for each leg we have 2,
      //   //  but we use only one for the connection
      //   if(walkdelay){
      //     for(int k=0; k<6; k++){
      //       LC.val(18+k*2+1,k*3)=1;
      //     }
      //   }

      //   if(legdelay){
      //     int k = 1;
      //     // leg 0: - 2
      //     LC.val(18+2*2+k,0*3+k)=1;
      //     // leg 1: - 3
      //     LC.val(18+3*2+k,1*3+k)=1;
      //     // leg 2:  -4
      //     LC.val(18+4*2+k,2*3+k)=1;
      //     // leg 3:  -5
      //     LC.val(18+5*2+k,3*3+k)=1;
      //   }

      //   std::cout << "apply Leg coupling" << std::endl;
      //   dep->setM(M+(LC^T));
      // }

      
      
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

  protected:
    lpzrobots::Joint* robotfixator;

};

int main (int argc, char **argv)
{
  //------------------test third-party library code-----------------------------
  // std::complex<double> aa = std::complex<double>( 1., 2. );
  /* Polynomial Evaluation */
  /* Domain size */
  size_t m = 4;
  /* Evaluation vector */
  //std::complex<double> z4 = 1. + 2i, z5 = 1. - 2i; // conjugates C++14 directly using 'i'
  std::vector<libff::Double> f = { std::complex<double>( 1., 2. ), std::complex<double>( -1., 2. ), std::complex<double>( 1., 2. ), std::complex<double>( -1., 2. )}; //2. + 2i, 5. - 2i, 3, 8 };
  /* Get evaluation domain */
  std::shared_ptr<evaluation_domain<libff::Double> > domain = get_evaluation_domain<libff::Double>(m);
  /* FFT */
  domain->FFT(f);
  for (size_t i = 0; i < f.size(); i++)
  {
    printf("%ld: %ld\n", i, f[i].as_ulong());     //as_long(){round(var.real())}  unsigned long Double::as_ulong() const{return round(val.real());}
    // std::cout<< f[i].val<<std::endl;
    printf("Magnitude: %0.1f, (%0.1f, %0.1f)\n", i, std::sqrt(f[i].val.real() *f[i].val.real() +f[i].val.imag()* f[i].val.imag()), f[i].val.real(), f[i].val.imag());
  }


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
  passing_coverage_to_internal_param = false;
  passing_coverage_to_internal_param = Simulation::contains(argv,argc,"-plot_coverage")    != 0;

  with_12_delayed_sensors=false;
  with_12_delayed_sensors = Simulation::contains(argv,argc,"-delayed_sensor")    != 0;

  plot_collision = true;
  plot_collision = Simulation::contains(argv,argc,"-no_plot")    == 0;

  wavelet_transform = false;
  wavelet_transform = Simulation::contains(argv,argc,"-wavelet_transform")    != 0;

  log_collision = false;
  log_collision = Simulation::contains(argv,argc,"-log_collision")  != 0;

  log_layer = false;
  log_layer = Simulation::contains(argv,argc,"-log_layer")  != 0;


  terrain_coverage = false;
  terrain_coverage    = Simulation::contains(argv,argc,"-terrain_coverage")    != 0;

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
