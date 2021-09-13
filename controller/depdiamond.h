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
#ifndef __DEP_DIAMOND_H
#define __DEP_DIAMOND_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include "abstractdiamondcontroller.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/teachable.h>
#include <selforg/parametrizable.h>

#include <selforg/ringbuffer.h>


/// configuration object for DEPDiamond controller. DEPDiamond::getDefaultConf().
struct DEPDiamondConf {
#define LEARNINGRULES  \
    X(DEP,      "DEP") \
    X(DHL,      "DHL") \
    X(HL,       "HL")  \
    X(ADEP,     "ADEP") \
    X(DIAMONDDEP,     "DIAMONDDEP")\
    X(DEPCTM,     "DEPCTM")

  enum LearningRule {
#define X(Enum, String)       Enum,
    LEARNINGRULES
#undef X
  };
  std::map<LearningRule, std::string> LearningRuleNames = {
#define X(Enum, String) { Enum , String } ,
    LEARNINGRULES
#undef X
  };

  LearningRule learningRule;

  double initFeedbackStrength;  ///< initial strength of sensor to motor connection
  bool   initModel;             ///< initialize model or leave it 0 to be learned
  /// # of steps the sensors are averaged (1 means no averaging)
  int    steps4Averaging;
  /// # of steps the motor values are delayed (1 means no delay)
  int    steps4Delay;
  bool   calcEigenvalues;       ///< if true calculate the eigenvalues of L
  
  //------------additional config param from the diamond model-----------------
  bool   useExtendedModel;      ///< if true, the extended model (S matrix) is used
  /// if true the controller can be taught see teachable interface
  bool   useTeaching;

  //   bool   someInternalParams;    ///< if true only some internal parameters are exported
  //   bool   onlyMainParameters;    ///< if true only some configurable parameters are exported
  //   double factorS;             ///< factor for learning rate of S
  //   double factorb;             ///< factor for learning rate of b
  //   double factorh;             ///< factor for learning rate of h

};


/// configuration object for DEPDiamond controller. Use DEPDiamond::getDefaultConf().
// struct DEPDiamondConf {
//   double initFeedbackStrength;  ///< initial strength of sensor to motor connection
//   bool   useExtendedModel;      ///< if true, the extended model (S matrix) is used
//   /// if true the controller can be taught see teachable interface
//   bool   useTeaching;
//   /// # of steps the sensors are averaged (1 means no averaging)
//   int    steps4Averaging;
//   /// # of steps the motor values are delayed (1 means no delay)
//   int    steps4Delay;
//   bool   someInternalParams;    ///< if true only some internal parameters are exported
//   bool   onlyMainParameters;    ///< if true only some configurable parameters are exported

//   double factorS;             ///< factor for learning rate of S
//   double factorb;             ///< factor for learning rate of b
//   double factorh;             ///< factor for learning rate of h
// };



/**
 * This controller implements a new very much simplified algorihm derived from TiPI maximization
 */
class DEPDiamond : public AbstractController, public Teachable, public Parametrizable {

public:
  DEPDiamond(const DEPDiamondConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~DEPDiamond();

  static DEPDiamondConf getDefaultConf(){
    DEPDiamondConf conf;
    conf.learningRule= DEPDiamondConf::DEPCTM; // DEPDiamondConf::DIAMONDDEP;  //DEPDiamondConf::DEP;
    conf.initFeedbackStrength = 0;
    conf.steps4Averaging      = 1;
    conf.steps4Delay          = 1;
    conf.calcEigenvalues      = false;
    conf.initModel            = true;

    conf.useExtendedModel     = true;
    conf.useTeaching          = false;
    
    return conf;
  }


// class DEPDiamond : public AbstractController, public Teachable, public Parametrizable {

// public:
//   /// constructor
//   DEPDiamond(const DEPDiamond& conf = getDefaultConf());

//   /// constructor provided for convenience, use conf object to customize more
//   DEPDiamond(double init_feedback_strength, bool useExtendedModel = true,
//       bool useTeaching = false );

//   virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

//   virtual ~DEPDiamond();

//   static DEPDiamondConf getDefaultConf(){
//     DEPDiamondConf conf;
//     conf.initFeedbackStrength = 1.0;
//     conf.useExtendedModel     = true;
//     conf.useTeaching          = false;
//     conf.steps4Averaging      = 1;
//     conf.steps4Delay          = 1;
//     conf.someInternalParams   = false;
//     conf.onlyMainParameters   = true;

//     conf.factorS              = 1;
//     conf.factorb              = 1;
//     conf.factorh              = 1;
//     return conf;
//   }


  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);
  // Diamond Main Variant step
  virtual void stepMV(const sensor* , int number_sensors, motor* , int number_motors, DEPDiamond*);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  virtual void stepNoLearningMV(const sensor* , int number_sensors,
                              motor* , int number_motors, DEPDiamond*);

  /// called during babbling phase
  virtual void motorBabblingStep(const sensor* , int number_sensors,
                                 const motor* , int number_motors);

  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /* some direct access functions (unsafe!) */
  
  // virtual matrix::Matrix getA();
  // virtual matrix::Matrix* getpA();
  // virtual void setA(const matrix::Matrix& A);
  // virtual matrix::Matrix getC();
  // virtual void setC(const matrix::Matrix& C);
  // virtual matrix::Matrix geth();
  // virtual void seth(const matrix::Matrix& h);
  // virtual matrix::Matrix getb();


  // accessors to matrices
  virtual matrix::Matrix getM(){  return M; }
  virtual matrix::Matrix* getpM(){  return &M;}
  virtual matrix::Matrix getC(){  return C; }
  virtual matrix::Matrix* getpC(){  return &C;}
  virtual matrix::Matrix getC_avg(){  return C_avg; }
  virtual matrix::Matrix* getpC_avg(){  return &C_avg;}

  virtual matrix::Matrix getEvRe(){  return eigenvaluesLRe; }
  virtual matrix::Matrix* getpEvRe(){  return &eigenvaluesLRe;}
  virtual matrix::Matrix getL(){  return L; }
  virtual matrix::Matrix* getpL(){  return &L;}

  virtual matrix::Matrix get_x(){  return x_temporal; }
  virtual matrix::Matrix* get_px(){  return &x_temporal;}
  virtual matrix::Matrix get_y(){  return y_temporal; }
  virtual matrix::Matrix* get_py(){  return &y_temporal;}

  // virtual void setx_temporal(const matrix::Matrix& _x_temporal){
  //   assert(x_temporal.getM() == _x_temporal.getM() && x_temporal.getN() == _x_temporal.getN());
  //   x_temporal=_x_temporal;
  // }

  // virtual void sety_temporal(const matrix::Matrix& _y_temporal){
  //   assert(y_temporal.getM() == _y_temporal.getM() && y_temporal.getN() == _y_temporal.getN());
  //   y_temporal=_y_temporal;
  // }
  virtual matrix::Matrix geth(){  return h; }
  virtual matrix::Matrix getx_smooth(){  return x_smooth; }

  
  virtual void setM(const matrix::Matrix& _M){
    assert(M.getM() == _M.getM() && M.getN() == _M.getN());
    M=_M;
  }
  // accessors to matrices
  // virtual matrix::Matrix getC(){  return C_update; }
  
  virtual matrix::Matrix getC_update(){  return C_update; }

  virtual void setC_update(const matrix::Matrix& _C_update){
    assert(C_update.getM() == _C_update.getM() && C_update.getN() == _C_update.getN());
    C_update=_C_update;
  }


  virtual void setC(const matrix::Matrix& _C){
    assert(C.getM() == _C.getM() && C.getN() == _C.getN());
    C=_C;
  }


  virtual double get_synboost(){  return synboost;  }
  virtual void set_synboost(double _synboost){
    synboost = _synboost;
  }

  virtual void printConf(){
    std::cout<< "The synboost is: "<< synboost <<std::endl;
    std::cout<< "The learning rate is: "<< urate <<std::endl;
    std::cout<< "The epsM is: "<< epsM <<std::endl;
    std::cout<< "The epsh is: "<< epsh <<std::endl;
    
  }


  /** returns the prediction of sensors for next time step */
  //virtual sensor* getPredictionState();
  // if motor_smooth_time_period = 0 means only y at time t, else is a sliding motor window,
  // parameter is chanable with the layers : if layers go deeper then the time_period should be larger
  virtual matrix::Matrix getPredictionState(int motor_smooth_time_period=0);

  /***** TEACHABLE ****/
  virtual void setMotorTeaching(const matrix::Matrix& teaching);
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  virtual matrix::Matrix getLastMotorValues();
  virtual matrix::Matrix getLastSensorValues();

  /***** PARAMETRIZABLE ****/
  virtual std::list<matrix::Matrix> getParameters() const override;
  virtual int setParameters(const std::list<matrix::Matrix>& params) override;

protected:
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 200;

  DEPDiamondConf conf; // configuration object

  //additional elements/parameters from the soxdiamond (sox) base algorithms.
  
  // matrix::Matrix S; // Model Matrix (sensor branch)
  // matrix::Matrix R; //
  // matrix::Matrix C_native; // Controller Matrix obtained from motor babbling
  // matrix::Matrix A_native; // Model Matrix obtained from motor babbling
  
  // matrix::Matrix v_avg;
  // matrix::Matrix x;        // current sensor value vector
  
  // bool loga;

  bool intern_isTeaching;    // teaching signal available?
  matrix::Matrix y_teaching; // motor teaching  signal

  // paramval creativity;
  // paramval sense;
  // paramval harmony;
  // paramval causeaware;
  // paramint pseudo;
  // paramval epsC;
  // paramval epsA;
  // paramval damping;
  // paramval gamma;          // teaching strength


  matrix::Matrix M; // Model Matrix
  matrix::Matrix C_update; // fast changing controller matrix (function of immediate history)
  matrix::Matrix C; // Acting Controller Matrix (normalized C_update)
  matrix::Matrix C_avg;

  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix

  RingBuffer<matrix::Matrix> x_buffer; // buffer needed for delay and derivatives
  RingBuffer<matrix::Matrix> y_buffer; // buffer needed for delay and derivatives

  RingBuffer<matrix::Matrix> M_buffer; // buffer needed for delay and derivatives

  // matrix::Matrix y_buffer[buffersize]; // buffer needed for delay
  // matrix::Matrix x_buffer[buffersize]; // buffer of sensor values

  matrix::Matrix x_smooth; // time average of x values
  matrix::Matrix normmot; // factors for individual normalization
  //matrix::Matrix y_smooth;  //I define this to use for calculate the prediction action state, and prediction action error


  matrix::Matrix x_temporal;    //adding temporal x and y sequence for visualization for every layer
  matrix::Matrix y_temporal;    //adding temporal x and y sequence for visualization for every layer
  


  matrix::Matrix eigenvaluesLRe; //Eigenvalues of L matrix real part
  matrix::Matrix eigenvaluesLIm; //Eigenvalues of L matrix imaginary part
  matrix::Matrix eigenvectors; //Eigenvectors of L matrix (real part)
  double proj_ev1; // projection of x into first eigenvector
  double proj_ev2; // projection of x into second eigenvector
  int calcEVInterval;

  int t;

  paramval epsh;
  paramval epsM;
  paramval norming;
  paramint s4avg;          // # of steps the sensors are averaged (1 means no averaging)
  paramint s4delay;        // # of steps the motor values are delayed (1 means no delay)

  int      indnorm;        ///< individual normalization (1) and global normalization (0)
  int      regularization; ///< exponent of regularization 10^{-regularization}

  paramval urate;          ///<
  paramval synboost;       ///< kappa in the paper

  paramval timedist;
  bool _internWithLearning;    // learning signal available?
  
  
  int time_average;
  int Time;

  /// learn  model (M = A^T )  // DEBUG: This oringinal comment should be M = A^(-1)
  virtual void learnModel(double eps);

  /// learn controller (C,h, C_update)
  virtual void learnController();

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

  /// function that clips the second argument to the interval [-r,r]
  static double clip(double r, double x){
    return min(max(x,-r),r);
  }

  //===============additional functions useful from the soxDiamond=================
  // calculates the pseudo inverse of L in different ways, depending on pseudo
  matrix::Matrix pseudoInvL(int pseudo, const matrix::Matrix& L, const matrix::Matrix& A, const matrix::Matrix& C);

  /// learn values model and controller (A,b,C,h)
  //virtual void learn();

  /// derivative of g
  static double g_s(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
  };
  
  /// inverse of neuron activation
  static double g_inv(double z)
  {
    return atanh(clip(.99, z));
  };


  /// one over sqrt 2 norm
  static double sqrt_norm(double z)
  {
    return (z / sqrt(2.0));
  };

  /// calculates the inverse the argument (useful for Matrix::map)
  static double one_over(double x){
    return 1/x;
  }

  /// calculates the inverse the argument (useful for Matrix::map)
  static double limit_regular_one_over(double x){
    //min(max(x,-r),r)
    return min(max(1.0/(x+0.01), -5.0), 5.0);   //clip by 5
  }

  //calculate the 3*3 submatrix average for C (or do a convolution or max pooling?)
  static matrix::Matrix average_matrix(matrix::Matrix CC ,int kk=3){
    matrix::Matrix CC_avg;
    int NN = CC.getM();
    int nn = NN/kk;
    int MM = CC.getN();
    int mm = MM/kk;
    CC_avg.set(nn, mm);
    double sum=0.0;
    double num_avg = 0.0;
    int count=0;

    for(int n=0; n<nn; n++){
      for(int m=0; m<mm; m++){
        sum = 0.0;
        count = 0;
        for(int i=0; i<NN; i++){
          for(int j=0; j<MM; j++){
            
            if(i>=n*kk && i<(n+1)*kk && j>=m*kk && j<(m+1)*kk){
              sum += CC.val(i, j);
              count += 1;
            }
    
          }
        }
        // std::cout <<"count: "<< count<< " ,with sum: "<< sum <<std::endl;
        // num_avg = sum/ (double) count;
        CC_avg.val(n,m) = sum;  //num_avg;

      }
    }

    return CC_avg;
  }




};

#endif


