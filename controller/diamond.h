/***************************************************************************
 *                                                                         *
 * author: Jerry Zhao & Simón C. Smith
 * e-mail: jerryzhao173985@gmail.com & artificialsimon@gmail.com
 *
 ***************************************************************************/

#ifndef __DIAMOND_H
#define __DIAMOND_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include "abstractdiamondcontroller.h"

// base controllers
// #include <selforg/sox.h>
#include "depdiamond.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/teachable.h>
#include <selforg/parametrizable.h>

#include <selforg/ringbuffer.h>
#include <selforg/matrixutils.h>

// utils
#include <string>
#include <vector>

using namespace matrix;
using namespace std;

struct All_Params{
  double l1_epsM;
  double l1_epsh;
  double l1_synboost;
  double l1_urate;
  int l1_indnorm;
  int l1_timedist;
  int l1_learningrule;
  int l1_time_average;

  double l2_epsM;
  double l2_epsh;
  double l2_synboost;
  double l2_urate;
  int l2_indnorm;
  int l2_timedist;
  int l2_learningrule;
  int l2_time_average;

  // internal_layer[0] ->setParam("epsM",0.005);
  // internal_layer[0] ->setParam("epsh",0.000);
  // internal_layer[0] ->setParam("synboost",1.1);   // 1.1~1.5
  // internal_layer[0] ->setParam("urate",0.05);
  // internal_layer[0] ->setParam("indnorm",1); // 0 is global normalization
  // internal_layer[0] ->setParam("timedist",4);
};

/// configuration object for Diamond controller. Use Diamond::getDefaultConf().
struct DiamondConf {
  int    n_layers;  ///< number of internal layers
  int    time_period;
  string base_controller_name;
  bool someInternalParams;  // < if true only some internal parameters are exported
  All_Params params;
};


/**
 * This controller implements the standard algorihm described the the Chapter 5 (Homeokinesis)
 *  with extensions of Chapter 15 of book "The Playful Machine"
 */
class Diamond : public AbstractController, public Teachable, public Parametrizable {

public:
  DEPDiamond *ol;
  DEPDiamond oll;
  /// constructor
  Diamond(const DiamondConf& conf = getDefaultConf());

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~Diamond();

  static DiamondConf getDefaultConf(){
    DiamondConf conf;
    conf.n_layers = 2;                          // MARK: default is 2 layer, better performance
    conf.time_period = 1;                       // sliding window for motor perdiction over certain time period
    // should perhaps construct a similar one as above for the sensor sliding(smoothing) window
    conf.base_controller_name = "DEPDiamond";   // or "Sox"
    conf.someInternalParams = true;             // < if true only some internal parameters are exported
    
    //initial layer parameter setting which is typically used in diamond.cpp in where they initial every depdiamond vector element
    All_Params all_params;
    all_params.l1_epsM = 0.001;
    all_params.l1_epsh = 0.000;
    all_params.l1_synboost = 1.5;
    all_params.l1_urate = 0.05;
    all_params.l1_indnorm = 1;
    all_params.l1_timedist = 4;
    all_params.l1_learningrule = 0;
    all_params.l1_time_average = 1;

    all_params.l2_epsM = 0.005;
    all_params.l2_epsh = 0.001;
    all_params.l2_synboost = 1.1;
    all_params.l2_urate = 0.02;
    all_params.l2_indnorm = 1;
    all_params.l2_timedist = 8;
    all_params.l2_learningrule = 0;
    all_params.l2_time_average = 1;
    
    conf.params = all_params;
    
    
    return conf;
  }


  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /* some direct access functions (unsafe!) */
  // virtual matrix::Matrix getA();
  // virtual void setA(const matrix::Matrix& A);
  // virtual matrix::Matrix getC();
  // virtual void setC(const matrix::Matrix& C);
  // virtual matrix::Matrix geth();
  // virtual void seth(const matrix::Matrix& h);

  // accessors to matrices
  virtual matrix::Matrix getM(){  return M; }
  virtual void setM(const matrix::Matrix& _M){
    assert(M.getM() == _M.getM() && M.getN() == _M.getN());
    M=_M;
  }
  // accessors to matrices
  virtual matrix::Matrix getC(){  return C_update; }
  virtual void setC(const matrix::Matrix& _C){
    assert(C_update.getM() == _C.getM() && C_update.getN() == _C.getN());
    C_update=_C;
  }

  virtual vector<DEPDiamond*> get_internal_layers(){  return internal_layer; }


  /***** TEACHABLE ****/
  virtual void setMotorTeaching(const matrix::Matrix& teaching);
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  virtual matrix::Matrix getLastMotorValues();
  virtual matrix::Matrix getLastSensorValues();

  /***** PARAMETRIZABLE ****/
  virtual std::list<matrix::Matrix> getParameters() const override;
  virtual int setParameters(const std::list<matrix::Matrix>& params) override;

protected:
  vector<DEPDiamond*> internal_layer; // internal layer controller
  DEPDiamond* make_layer(string base_controller_name); // Factory for base c.
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 200;

  DiamondConf conf; ///< configuration objects

  // the following two are not very important in the depdiamond.h but also should include!
  bool intern_isTeaching;    // teaching signal available?
  matrix::Matrix y_teaching; // motor teaching  signal

  matrix::Matrix M; // Model Matrix
  matrix::Matrix C_update; // fast changing controller matrix (function of immediate history)
  matrix::Matrix C; // Acting Controller Matrix (normalized C_update)
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix

  RingBuffer<matrix::Matrix> x_buffer; // buffer needed for delay and derivatives
  RingBuffer<matrix::Matrix> y_buffer; // buffer needed for delay and derivatives
  // matrix::Matrix y_buffer[buffersize]; // buffer needed for delay
  // matrix::Matrix x_buffer[buffersize]; // buffer of sensor values
  
  matrix::Matrix x;        // current sensor value vector
  matrix::Matrix x_smooth; // time average of x values
  matrix::Matrix normmot; // factors for individual normalization

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


  bool loga;

  // paramval creativity;
  // paramval sense;
  // paramval harmony;
  // paramval causeaware;
  // paramint pseudo;
  // paramval epsC;
  // paramval epsA;
  // paramval damping;
  // paramval gamma;          // teaching strength

    
  void constructor();

  // calculates the pseudo inverse of L in different ways, depending on pseudo
  matrix::Matrix pseudoInvL(int pseudo, const matrix::Matrix& L, const matrix::Matrix& A, const matrix::Matrix& C);

  /// learn values model and controller (A,b,C,h)
  // virtual void learn();

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

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


  /// function that clips the second argument to the interval [-first,first]
  static double clip(double r, double x){
    return min(max(x,-r),r);
  }
  /// calculates the inverse the argument (useful for Matrix::map)
  static double one_over(double x){
    return 1/x;
  }


};

#endif


