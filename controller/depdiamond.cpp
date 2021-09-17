/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

#include "depdiamond.h"
#include <selforg/matrixutils.h>
using namespace matrix;
using namespace std;



DEPDiamond::DEPDiamond(const DEPDiamondConf& conf)
  : AbstractController("DEPDiamond", "1.0"),
    conf(conf)
{
  t=0;

  addParameterDef("epsh", &epsh, 0.0,     0,5, "learning rate of the controller bias");
  addParameterDef("epsM", &epsM, 0.0,     0,5, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,     1, buffersize-1, "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, 1, buffersize-1, "delay  (number of steps)");

  addParameterDef("learningrule",  (int*)(&this->conf.learningRule),
                  false,              string("which learning rule to use: ") +
                  std::accumulate(conf.LearningRuleNames.begin(),conf.LearningRuleNames.end(),
                                  std::string(),[](std::string a, std::pair<DEPDiamondConf::LearningRule,std::string> lr){return a + itos((int)lr.first) + ": " + lr.second + ", ";}));
  addParameterDef("timedist", &timedist, 1,     0,10, "time distance of product terms in learning rule");
  addParameterDef("synboost", &synboost, 5,     0,1,  "booster for synapses during motor signal creation");
  addParameterDef("urate", &urate, .1,          0,5,  "update rate ");

  //  addParameterDef("maxspeed", &maxSpeed, 0.5,   0,2, "maximal speed for motors");
  addParameterDef("indnorm", &indnorm,     1,   0,2, "individual normalization for each motor");
  addParameterDef("regularization", &regularization, 12, 0, 15, "exponent of regularization 10^{-regularization}");

  addInspectableMatrix("M", &M, false, "inverse-model matrix");

  addInspectableMatrix("h",  &h, false,   "acting controller bias");
  addInspectableMatrix("C", &C, false, "acting controller matrix");

  addInspectableMatrix("C_avg", &C_avg, false, "6*6 average of C feet controller matrix");

  // additional parameters for time-averaging ADEP rule for every layer
  addParameterDef("time_average", &time_average,     1,   0,100, "time-averaging factor for time period in vector outer product in ADEP rule for every layer");
  // additional parameters for including-Time DIAMONDDEP rule for every layer
  addParameterDef("Time", &Time,     1,   0,100, "time-averaging factor for time period in vector outer product in DIAMONDDEP rule for every layer");


  if(conf.calcEigenvalues){
    addInspectableMatrix("EvRe", &eigenvaluesLRe, false, "Eigenvalues of L (Re)");
    addInspectableMatrix("EvIm", &eigenvaluesLIm, false, "Eigenvalues of L (Im)");
    addInspectableMatrix("EVs",  &eigenvectors,   false, "Eigenvectors of L (Re)");
    addInspectableValue("proj1",  &proj_ev1,  "projection of x on first Eigenvector (Re)");
    addInspectableValue("proj2",  &proj_ev2,  "projection of x on second Eigenvector (Re)");
    addParameterDef("evinterval", &calcEVInterval, 1,          0,1000,  "interval to update eigenvalues etc (0: never) ");
  }

  addInspectableValue("norming", &norming, "Normalization");
  addInspectableMatrix("normmor", &normmot, false, "individual motor normalization factor");

  _internWithLearning=false; // used in step to enable learning in stepNoLearning and have only one function
  
  intern_isTeaching = false;

};

DEPDiamond::~DEPDiamond(){
}


void DEPDiamond::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


  number_sensors= sensornumber;
  number_motors = motornumber;
  M.set(number_motors, number_sensors);
  C.set(number_motors, number_sensors);
  C_update.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  eigenvectors.set(number_sensors,number_sensors);
  eigenvaluesLRe.set(number_sensors,1);
  eigenvaluesLIm.set(number_sensors,1);
  normmot.set(number_motors, 1);

  L.set(number_sensors, number_sensors);
  
  C_avg.set((int) (number_motors/3), (int) (number_sensors/3));
  
  if(conf.initModel){
    // special model initialization for delay sensor
    //  (two ID matrices besides each other)
    if(number_sensors>=2*number_motors){
      Matrix M1(number_motors, number_sensors/2);
      M1.toId();
      M=M1.beside(M1);
    }else
      M.toId(); // set a to identity matrix;
  }

  C_update.toId();
  C_update*=conf.initFeedbackStrength;

  C_avg.toId();

  x_smooth.set(number_sensors,1);
  //y_smooth.set(number_motors,1);

  x_buffer.init(buffersize, Matrix(number_sensors,1));
  y_buffer.init(buffersize, Matrix(number_motors,1));

  M_buffer.init(buffersize, Matrix(number_motors, number_sensors));

  y_teaching.set(number_motors, 1);

  //For visualization for every layer
  x_temporal.set(number_sensors,1);
  y_temporal.set(number_motors,1);


  x_derivitives.init(buffersize, Matrix(number_sensors,1));
  x_derivitives_averages.init(buffersize, Matrix(number_sensors,1));
  x_derivitive_average.set(number_sensors, 1);

}





// DEPDiamond::DEPDiamond(const DEPDiamondConf& conf)
//   : AbstractController("DEPDiamond", "1.1"),
//     conf(conf)
// {
//   constructor();
// }


// DEPDiamond::DEPDiamond(double init_feedback_strength, bool useExtendedModel, bool useTeaching )
//   : AbstractController("DEPDiamond", "1.1"),
//     conf(getDefaultConf()){

//   conf.initFeedbackStrength = init_feedback_strength;
//   conf.useExtendedModel     = useExtendedModel;
//   conf.useTeaching          = useTeaching;
//   constructor();
// }

// void DEPDiamond::constructor(){
//   t=0;

//   addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
//   addParameterDef("epsC", &epsC, 0.1,     0,5, "learning rate of the controller");
//   addParameterDef("epsA", &epsA, 0.1,     0,5, "learning rate of the model");
//   addParameterDef("sense",  &sense,    1, 0.2,5,      "sensibility");
//   addParameterDef("creativity", &creativity, 0, 0, 1, "creativity term (0: disabled) ");
//   addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
//   addParameterDef("causeaware", &causeaware, conf.useExtendedModel ? 0.01 : 0 , 0,0.1,
//                   "awarness of controller influences");
//   addParameterDef("harmony",    &harmony,    0, 0,0.1,
//                   "dynamical harmony between internal and external world");
//   addParameterDef("pseudo",   &pseudo   , 0  ,
//     "type of pseudo inverse: 0 moore penrose, 1 sensor space, 2 motor space, 3 special");

//   if(!conf.onlyMainParameters){
//     addParameter("s4avg", &conf.steps4Averaging, 1, buffersize-1,
//                     "smoothing (number of steps)");
//     addParameter("s4delay", &conf.steps4Delay,   1, buffersize-1,
//                     "delay  (number of steps)");
//     addParameter("factorS", &conf.factorS,  0, 2,
//                     "factor for learning rate for S");
//     addParameter("factorb", &conf.factorb,  0, 2,
//                     "factor for learning rate for b");
//     addParameter("factorh", &conf.factorh,  0, 2,
//                     "factor for learning rate for h");
//   }

//   gamma=0;
//   if(conf.useTeaching){
//     addParameterDef("gamma",  &gamma,    0.01, 0, 1, "guidance factor (teaching)");
//     addInspectableMatrix("y_G", &y_teaching, "teaching signal at motor neurons");
//   }

//   addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
//   if(conf.useExtendedModel)
//     addInspectableMatrix("S", &S, conf.someInternalParams, "model matrix (sensor branch)");
//   addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
//   addInspectableMatrix("L", &L, conf.someInternalParams, "Jacobi matrix");
//   addInspectableMatrix("h", &h, conf.someInternalParams, "controller bias");
//   addInspectableMatrix("b", &b, conf.someInternalParams, "model bias");
//   addInspectableMatrix("R", &R, conf.someInternalParams, "linear response matrix");

//   addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");

//   intern_isTeaching = false;

// };

// DEPDiamond::~DEPDiamond(){
// }


// void DEPDiamond::init(int sensornumber, int motornumber, RandGen* randGen){
//   if(!randGen) randGen = new RandGen(); // this gives a small memory leak

//   number_sensors= sensornumber;
//   number_motors = motornumber;
//   A.set(number_sensors, number_motors);
//   S.set(number_sensors, number_sensors);
//   C.set(number_motors, number_sensors);
//   b.set(number_sensors, 1);
//   h.set(number_motors, 1);
//   L.set(number_sensors, number_sensors);
//   v_avg.set(number_sensors, 1);
//   A_native.set(number_sensors, number_motors);
//   C_native.set(number_motors, number_sensors);

//   R.set(number_sensors, number_sensors);

//   A.toId(); // set a to identity matrix;
//   C.toId(); // set a to identity matrix;
//   C*=conf.initFeedbackStrength;

//   S.toId();
//   S*=0.05;

//   // if motor babbling is used then this is overwritten
//   A_native.toId();
//   C_native.toId();
//   C_native*=1.2;

//   y_teaching.set(number_motors, 1);

//   x.set(number_sensors,1);
//   x_smooth.set(number_sensors,1);
//   for (unsigned int k = 0; k < buffersize; k++) {
//     x_buffer[k].set(number_sensors,1);
//     y_buffer[k].set(number_motors,1);

//   }
// }


matrix::Matrix DEPDiamond::getPredictionState(int motor_smooth_time_period) {
  /** Call it after step() */
  
  matrix::Matrix yt = y_buffer[t%buffersize]; //buffersize is 200 MARK: should this here be a smoothed_y? Let's try
  //matrix::Matrix y_smoothed_over_time_period_for_inner_layer = yt;
  matrix::Matrix y = yt;
  for(int i=0; i<motor_smooth_time_period; i++){
    y += ( y_buffer[(t-1-i)%buffersize] - y) * 0.1;   //0.02
  }
  
  // matrix::Matrix M_prime = M;
  // matrix::Matrix x_hat = (M_prime.map(limit_regular_one_over)) * y ;
  
  // matrix::Matrix x_hat = (M + 0.01).pseudoInverse() * y ;
  // matrix::Matrix x_hat = (M ^T) * y ;
  matrix::Matrix x_hat = M.mapP(5.0, clip) * x_smooth - y;    // take the prediction error of the action!!
  // std::cout<< x_hat << std::endl;
  // or return x_har.sqrt()
  return x_hat * 0.5;   // or 0.25?
}
// sensor* DEPDiamond::getPredictionState() {
//   /** Call it after step() */
//   sensor* x_ = new sensor[number_motors];
//   matrix::Matrix y = y_buffer[t%buffersize];
//   matrix::Matrix x_hat = A * y + b + S * x;
//   x_hat.convertToBuffer(x_, number_motors);
//   //cout << x_ << endl;
//   return x_;
// }


// matrix::Matrix DEPDiamond::getA(){
//   return A;
// }

// matrix::Matrix* DEPDiamond::getpA(){
//   return &A;
// }

// void DEPDiamond::setA(const matrix::Matrix& _A){
//   assert(A.getM() == _A.getM() && A.getN() == _A.getN());
//   A=_A;
// }

// matrix::Matrix DEPDiamond::getC(){
//   return C;
// }

// void DEPDiamond::setC(const matrix::Matrix& _C){
//   assert(C.getM() == _C.getM() && C.getN() == _C.getN());
//   C=_C;
// }

// matrix::Matrix DEPDiamond::geth(){
//   return h;
// }

// matrix::Matrix DEPDiamond::getb(){
//   return b;
// }

// void DEPDiamond::seth(const matrix::Matrix& _h){
//   assert(h.getM() == _h.getM() && h.getN() == _h.getN());
//   h=_h;
// }





//-----------------------------------------------------------------------------------------
// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DEPDiamond::step(const sensor* x_, int number_sensors,
                       motor* y_, int number_motors){
  
  _internWithLearning=true;
  stepNoLearning(x_, number_sensors, y_, number_motors);
  
  //stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  //if(epsC!=0 || epsA!=0)
  //  learn();

  // update step counter
  t++;

  _internWithLearning=false;
  
  x_temporal = getLastSensorValues();
  y_temporal = getLastMotorValues();

};

// // performs one step without learning. Calulates motor commands from sensor inputs.
// void DEPDiamond::stepNoLearning(const sensor* x_, int number_sensors,
//                                  motor* y_, int number_motors){
//   assert((unsigned)number_sensors <= this->number_sensors
//          && (unsigned)number_motors <= this->number_motors);

//   x.set(number_sensors,1,x_); // store sensor values

//   // averaging over the last s4avg values of x_buffer
//   conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
//   if(conf.steps4Averaging > 1)
//     x_smooth += (x - x_smooth)*(1.0/conf.steps4Averaging);
//   else
//     x_smooth = x;

//   x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value

//   // calculate controller values based on current input values (smoothed)
//   Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);

//   // Put new output vector in ring buffer y_buffer
//   y_buffer[t%buffersize] = y;

//   // convert y to motor*
//   y.convertToBuffer(y_, number_motors);

//   // update step counter
//   t++;
// };


void DEPDiamond::stepNoLearning(const sensor* x_, int number_sensors_robot,
                         motor* y_, int number_motors_){
  assert((unsigned)number_sensors_robot <= this->number_sensors
         && (unsigned)number_motors_ <= this->number_motors);

  Matrix xrobot(number_sensors_robot,1,x_); // store sensor values

  // averaging over the last s4avg values of x_buffer
  if(s4avg > 1)
    x_smooth += (xrobot - x_smooth)*(1.0/s4avg);
  else
    x_smooth = xrobot;

  // std::cout << t << "  "<< x_buffer[t-1].val(0,0);
  x_buffer[t] = x_smooth;

  // std::cout<< "periodic simulation time step t w.r.t. buffersize" << buffersize << ",  " << t << "  another: "<< (buffersize +t-2)%buffersize <<",  " << x_buffer[1] ;
  // x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value
  x_derivitives[t] = x_buffer[t] - x_buffer[t-2];

  
  // 2. calculate x_derivitives MOVING averages and stored in another RingBuffer
  // if (t%10000 ==100){                                            // 100 seconds do a hard update!
  //   std::cout<< "hard update on the derivitives!"<< std::endl;
  //   x_derivitives_averages[t] = x_derivitives[t];
  // }else{
  //   // soft update
  //   double update_avg = 0.1; 
  //   x_derivitives_averages[t] += (x_derivitives[t] - x_derivitives_averages[t]) * update_avg;
  // }

  // do not need hard update, initialize x_derivitives_averages to zero, the value is gradually moving
  x_derivitives_averages[t] += (x_derivitives[t] - x_derivitives_averages[t]) * 0.1;



  if(_internWithLearning)
    learnController();

  // Controller function
  Matrix y =   (C*x_smooth  + h  ).map(g);
  // Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);

  y_buffer[t] = y;
  // Put new output vector in ring buffer y_buffer
  //y_buffer[t%buffersize] = y;


  if(_internWithLearning && epsM!=0)
    learnModel(epsM);

  y.convertToBuffer(y_, number_motors);
  // update step counter
  t++;
};


// Diamond Main Variant step
void DEPDiamond::stepMV(const sensor* x_, int number_sensors,
                       motor* y_, int number_motors, DEPDiamond* sox_l1){
  
  _internWithLearning=true;
  stepNoLearningMV(x_, number_sensors, y_, number_motors, sox_l1);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  //if(epsC!=0 || epsA!=0)
  //  learn();

  // update step counter
  t++;

  _internWithLearning=false;

  x_temporal = getLastSensorValues();
  y_temporal = getLastMotorValues();
};



// // Diamond Main variant
// void DEPDiamond::stepNoLearningMV(const sensor* x_, int number_sensors,
//                                  motor* y_, int number_motors, DEPDiamond* sox_l1){
//   assert((unsigned)number_sensors <= this->number_sensors
//          && (unsigned)number_motors <= this->number_motors);

//   x.set(number_sensors,1,x_); // store sensor values

//   // averaging over the last s4avg values of x_buffer
//   conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
//   if(conf.steps4Averaging > 1)
//     x_smooth += (x - x_smooth)*(1.0/conf.steps4Averaging);
//   else
//     x_smooth = x;

//   x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value

//   // calculate controller values based on current input values (smoothed)
//   Matrix y =   (C*(
//                    x_smooth + (v_avg*creativity) + // x_0
//                    ((C^T)*((sox_l1->getLastMotorValues()).map(g_inv) -h)) * 0.15 // x^tilde_1
//                   )
//                    //* 0.5)
//                    //).map(sqrt_norm)
//                 + h).map(g);
//   //std::cout << "in " << y << std::endl;
//   // Put new output vector in ring buffer y_buffer
//   y_buffer[t%buffersize] = y;

//   // convert y to motor*
//   y.convertToBuffer(y_, number_motors);

//   // update step counter
//   t++;
// };



void DEPDiamond::stepNoLearningMV(const sensor* x_, int number_sensors_robot,
                         motor* y_, int number_motors_, DEPDiamond* dep_l1){
  assert((unsigned)number_sensors_robot <= this->number_sensors
         && (unsigned)number_motors_ <= this->number_motors);

  Matrix xrobot(number_sensors_robot,1,x_); // store sensor values

  // averaging over the last s4avg values of x_buffer
  if(s4avg > 1){
    std::cout << "DANGEROUS!! Average is calculating for the sensor values!!" <<std::endl;
    x_smooth += (xrobot - x_smooth)*(1.0/s4avg);
  }else{
    // std::cout << "SAFE!! Average is NOT calculating for the sensor values!!" <<std::endl;
    x_smooth = xrobot;
  }

  x_buffer[t] = x_smooth;
  //x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value

  // 1. calculate x_derivitives and stored in a RingBuffer
  x_derivitives[t] = x_buffer[t] - x_buffer[t-2];

  // 2. calculate x_derivitives MOVING averages and stored in another RingBuffer
  // do not need hard update, initialize x_derivitives_averages to zero, the value is gradually moving
  x_derivitives_averages[t] += (x_derivitives[t] - x_derivitives_averages[t]) * 0.1;
  
  // if (t%10000 ==100){                                            // 100 seconds do a hard update!
  //   std::cout<< "hard update on the derivitives!"<< std::endl;
  //   x_derivitives_averages[t] = x_derivitives[t];
  // }else{
  //   // soft update
  //   double update_avg = 0.1; 
  //   x_derivitives_averages[t] += (x_derivitives[t] - x_derivitives_averages[t]) * update_avg;
  // }


  if(_internWithLearning)
    learnController();

  // Controller function
  // Matrix y =   (C*x_smooth  + h  ).map(g);
  // Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);
  // TODO: change this! calculate controller values based on current input values (smoothed)
  
  
  // Matrix y =   (C*(
  //                  x_smooth /* * 0.85 */ +  // (v_avg*creativity) + // x_0
  //                  ((C.pseudoInverse())*((dep_l1->getLastMotorValues()).map(g_inv) -h)) * 0.15 // x^tilde_1
  //                 //  ( (dep_l1->getM()).pseudoInverse() * (dep_l1->getLastMotorValues()) ) *0.15  // time-loop error
  //                 )
  //                  //* 0.5)
  //                  //).map(sqrt_norm)
  //               + h).map(g);  //+ dep_l1->getLastMotorValues()   
  
  Matrix y = dep_l1->getLastMotorValues() + (C * x_temporal/*x_smooth*/+ h).map(g); 

  y_buffer[t] = y;
  // Put new output vector in ring buffer y_buffer
  //y_buffer[t%buffersize] = y;


  if(_internWithLearning && epsM!=0)
    learnModel(epsM);

  y.convertToBuffer(y_, number_motors);
  // update step counter
  t++;
};




void DEPDiamond::learnController(){
  // store the M matrix here for use later in the DIAMONDDEP model
  M_buffer[t] = M;

  ///////////////// START of Controller Learning / Update  ////////////////
  int diff = 1;
  Matrix mu;
  Matrix v;

  Matrix updateC;

  switch(conf.learningRule){
  case DEPDiamondConf::DEP: { ////////////////////////////
    Matrix chi  = x_buffer[t] - x_buffer[t - diff];
    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    mu = (M * chi);
    // const Matrix& 
    updateC =   ( mu ) * (v^T);
    break;
  } // with the '{}' after 'case', scope of 'chi' ends here
  case DEPDiamondConf::DHL:{  ////////////////////////////
    mu  = y_buffer[t -   diff] - y_buffer[t - 2*diff];
    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    updateC =   ( mu ) * (v^T);
    break;
  }
  case DEPDiamondConf::HL:{  ////////////////////////////
    mu = y_buffer[t -   diff];
    v  = x_buffer[t - diff];
    updateC =   ( mu ) * (v^T);
    break;
  }
  case DEPDiamondConf::ADEP: {
    // std::cout << "time_average  " <<time_average <<std::endl;
    // int time_average = 5;
    //std::cout << "Actually using the time-averaging ADEP rule, with time_average=" <<time_average <<std::endl;
    Matrix ch  = x_buffer[t] - x_buffer[t - diff];
    for(int i=0; i<time_average; i++){
      ch += (x_buffer[t-i-1] - x_buffer[t - diff-i-1] - ch) * 0.1;
    }

    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    for(int i=0; i<time_average; i++){
      v += (x_buffer[t - timedist-i-1] - x_buffer[t - timedist - diff-i-1] -v) * 0.1;
    }

    mu = (M * ch);
    updateC =   ( mu ) * (v^T);
    break;
  } // with the '{}' after 'case', scope of 'ch' ends here

  case DEPDiamondConf::DIAMONDDEP: { ////////////////////////////
    // std::cout << "Time  " <<Time <<std::endl;
    Matrix chis[100];
    Matrix vs[100];
    Matrix mus[100];

    for(int i=0; i<Time; i++){
      chis[i] = x_buffer[t-i] - x_buffer[t - diff -i];
      vs[i] = x_buffer[t - timedist -i] - x_buffer[t - timedist - diff -i];
      mus[i] = (M_buffer[i] * chis[i]);
    }
    // Matrix chi  = x_buffer[t] - x_buffer[t - diff];
    // v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    // mu = (M * chi);

    updateC.set(number_motors, number_sensors);
    updateC.toZero();
    // C.set(number_motors, number_sensors);
    // std::cout << (1./Time) << std::endl;
    for(int i=0; i<Time; i++){
      updateC += ( ( mus[i] ) * ((vs[i])^T) ) * (1./Time);  //average vector outer product
    }
    break;
  } // with the '{}' after 'case', scope of 'chi' ends here

  
  
  case DEPDiamondConf::DEPCTM: { // DEP with cross time mapping

    Matrix chis[500];
    Matrix vs[500];
    Matrix mus[500];
    // Matrix vs_bar[500];
    // Matrix v_bar;

    for(int i=0; i<Time; i++){
      chis[i] = x_buffer[t-i] - x_buffer[t - diff -i];
      vs[i] = x_buffer[t - timedist -i] - x_buffer[t - timedist - diff -i];
      mus[i] = (M_buffer[i] * chis[i]);
    }


    // v_bar +=  ( vs[t+1] - vs[t-1] ) *0.01 ;
    
    
    Matrix chi  = x_buffer[t] - x_buffer[t - diff];
    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    mu = (M * chi);
    
    updateC =   ( mu ) * (v^T);
    
    Matrix Lambda;
    Lambda.set(number_sensors, number_sensors);
    Lambda.toZero();

    for(int i=0; i<Time; i++){ 
      Lambda += ( ( vs[i] ) * ((vs[i])^T) ) * (1./Time);  //average vector outer product
    }


    updateC = updateC * Lambda.pseudoInverse();

    break;
  } // with the '{}' after 'case', scope of 'chi' ends here


  case DEPDiamondConf::DEPZERO: { // y==0

    updateC.set(number_sensors, number_motors);
    updateC.toZero(); 

    break;
  } 

  
  // here in the rules we use x_derivitives_averages
  case DEPDiamondConf::DEPNEW: { // DEP with cross time mapping

    Matrix chi;
    chi.set(number_sensors,1);
    Matrix MM;
    MM.set(number_motors, number_sensors);         
    Matrix Lambda;
    Lambda.set(number_sensors, number_sensors);

    // CHANGES: making Lambda update inside average update below
    for(int i=(t-Time); i<t; i++){ 
      Lambda += ( ( x_derivitives_averages[i-timedist] ) * ((x_derivitives_averages[i-timedist])^T) ) * (1./Time);  //average vector outer product
    }
    
    updateC.set(number_motors, number_sensors);
    
    //using averaged derivitives here in chi and v! Lambda update inside.
    for(int i=(t-Time); i<t; i++){            // 0--> T change to (t-T) --> t
      chi  = x_derivitives_averages[i];       // x_derivitives[i];          // t-i to i   //// or here it could also be i+1
      v = x_derivitives_averages[i-timedist]; // x_derivitives[i-timedist];
      MM =  M_buffer[i];
      // Lambda = ( ( x_derivitives[i-timedist] ) * ((x_derivitives[i-timedist])^T) );
      updateC += ( ((MM * chi) * (v^T) ) * Lambda.pseudoInverse()) * (1./Time);                 // time averaged on all product
    }
    
    // for(int i=0; i<Time; i++){ 
    //   Lambda += ( ( x_derivitives[t-timedist-i] ) * ((x_derivitives[t-timedist-i])^T) ) * (1./Time);  //average vector outer product
    // }
    // updateC = updateC * Lambda.pseudoInverse();

    break;
  } 



  default:
    cerr << "unkown learning rule!" << endl;
  }
  
  // const Matrix& updateC =   ( mu ) * (v^T);


  if ( t > 10){
    C_update += ((updateC   - C_update)*urate);  // matrix for controller before normalization
  }

  double reg = pow(10,-regularization);
  switch(indnorm){
  case 1: {
    //***** individual normalization for each motor neuron***************
    const Matrix& CM=C_update*(M^T);
    for (int i=0; i<number_motors; i++) {
      double normi = sqrt(CM.row(i).norm_sqr()); // norm of one row
      // for some historic reasons there is a 0.3 here
      //  which we keep for consistency with the paper
      normmot.val(i,0) = .3*synboost/( normi + reg);
    }
    C = C_update.multrowwise(normmot);
    break;
  }
  case 0: { // global
    double norm1 = sqrt((C_update*(M^T)).norm_sqr());
    C = C_update * (synboost /(norm1 + reg)); // C stays relatively constant in size
    C.toMapP(5.0,clip); // nevertheless clip C to some reasonable range
    break;
  }
  default:  { // no normalization
    C = C_update * synboost;
    break;
  }}
  if(conf.calcEigenvalues){
    if(calcEVInterval!=0 && (t%calcEVInterval==0)){
      Matrix EVImag;
      const Matrix& L=(M^T)*C;
      eigenValuesVectors(L, eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
      toPositiveSignEigenVectors(eigenvectors, EVImag);
      scaleEigenVectorsWithValue(eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
    }
    // calc overlap of sensor state with first 2 eigenvectors (this we do every step)
    proj_ev1=((eigenvectors.column(0)^T) * x_buffer[t]).val(0,0);
    proj_ev2=((eigenvectors.column(1)^T) * x_buffer[t]).val(0,0);
  }

  const Matrix& y_last = y_buffer[t-1];
  if(epsh>=0)
    h -= ( y_last *  epsh).mapP(.05, clip) + h*.001;
  else
    h*=0;

  ///////////////// End of Controller Learning ////////

  //cauculate the average of the controller matrix C for every leg 6*6 matrix to find correlation in C_avg;
  C_avg = average_matrix(C,3);  //calculate the 3*3 average of this motor-sensor matrix.
}

void DEPDiamond::learnModel(double eps){
  // learn inverse model y = F(x') = M x'
  // the effective x/y is (actual-steps4delay) element of buffer
  s4delay = ::clip(s4delay,1,buffersize-1);
  int  t_delay =  max(s4delay,1)-1;
  if(eps!=0){
    const Matrix& y = y_buffer[t - t_delay - timedist];
    const Matrix& x_fut   = x_buffer[t];
    const Matrix& xi = y - (M * x_fut);
    // M += eps xi x_fut^T
    M += (xi*(x_fut^T) * eps).mapP(0.05, clip) - M*eps*0.01; //update - damping
  }
};



void DEPDiamond::motorBabblingStep(const sensor* x_, int number_sensors_robot,
                            const motor* y_, int number_motors){
  assert((unsigned)number_motors <= this->number_motors);
  Matrix x(number_sensors_robot,1,x_); // convert to matrix
  Matrix y(number_motors,1,y_); // convert to matrix
  x_buffer[t] = x;
  y_buffer[t] = y;

  // model learning
  learnModel(1.0/sqrt(t+1));

  t++;
}


// void DEPDiamond::motorBabblingStep(const sensor* x_, int number_sensors,
//                             const motor* y_, int number_motors){
//   assert((unsigned)number_sensors <= this->number_sensors
//          && (unsigned)number_motors <= this->number_motors);
//   x.set(number_sensors,1,x_);
//   x_buffer[t%buffersize] = x;
//   Matrix y(number_motors,1,y_);
//   y_buffer[t%buffersize] = y;

//   double factor = .1; // we learn slower here
//   // learn model:
//   const Matrix& x_tm1 = x_buffer[(t - 1 + buffersize) % buffersize];
//   const Matrix& y_tm1 = y_buffer[(t - 1 + buffersize) % buffersize];
//   const Matrix& xp    = (A * y_tm1+ b + S * x_tm1);
//   const Matrix& xi   = x - xp;

//   double epsS=epsA*conf.factorS;
//   double epsb=epsA*conf.factorb;
//   A += (xi * (y_tm1^T) * (epsA * factor) + (A *  -damping) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
//   b += (xi           * (epsb * factor) + (b *  -damping) * ( epsb > 0 ? 1 : 0)).mapP(0.1, clip);
//   if(conf.useExtendedModel)
//     S += (xi * (x_tm1^T) * (epsS*factor) + (S *  -damping*10 ) * ( epsS > 0 ? 1 : 0)).mapP(0.1, clip);

//   // learn controller
//   const Matrix& z       = (C * (x_tm1) + h); // here no creativity
//   const Matrix& yp      = z.map(g);
//   const Matrix& g_prime = z.map(g_s);
//   const Matrix& delta   = (y_tm1 - yp) & g_prime;
//   C += ((delta * (x_tm1^T)) * (epsC *factor)).mapP(0.1, clip) + (C *  -damping);
//   h += (delta * (epsC *factor*conf.factorh)).mapP(0.1, clip);
//   C_native = C;
//   A_native = A;
//   t++;
// }


Matrix DEPDiamond::pseudoInvL(int pseudo, const Matrix& L, const Matrix& A, const Matrix& C){
  if(pseudo == 0){
    return L.pseudoInverse();
  }else{
    const Matrix& P = pseudo==1 || pseudo==2 ? A^T : C;
    const Matrix& Q = pseudo==1              ? C^T : A;
    return Q *((P * L * Q)^(-1)) * P;
  }
}



void DEPDiamond::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  y_teaching= teaching.mapP(0.95,clip);
  intern_isTeaching=true;
  //// MARK: after we applied teaching signal it should be switched off until new signal is given
}

void DEPDiamond::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching,
  // that belongs to the distal teaching value by the inverse model.
  y_teaching = (M.pseudoInverse() * (teaching-b)).mapP(0.95, clip);
  intern_isTeaching=true;
}

matrix::Matrix DEPDiamond::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix DEPDiamond::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}

list<Matrix> DEPDiamond::getParameters() const {
  return {C,h};
}

int DEPDiamond::setParameters(const list<Matrix>& params){
  if(params.size() == 2){
    list<Matrix>::const_iterator i = params.begin();
    const Matrix& CN = *i;
    if(C.hasSameSizeAs(CN)) C=CN;
    else return false;
    const Matrix& hN = *(++i);
    if(h.hasSameSizeAs(hN)) h=hN;
    else return false;
  } else {
    fprintf(stderr,"setParameters wrong len %i!=2\n", (int)params.size());
    return false;
  }
  return true;
}


/* stores the controller values to a given file. */
bool DEPDiamond::store(FILE* f) const{
  // save matrix values
  C_update.store(f);
  h.store(f);
  M.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool DEPDiamond::restore(FILE* f){
  // save matrix values
  C_update.restore(f);
  h.restore(f);
  Matrix Mod;
  Mod.restore(f);
  // old versions stored the model matrix in transposed form
  if(Mod.getM()==M.getM() && Mod.getN()==M.getN())
    M=Mod;
  else if(Mod.getM()==M.getN() && Mod.getN()==M.getM())
    M=Mod^T;
  else
    return false;
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

