#include "diamond.h"
#include "depdiamond.h"
#include <iterator>
#include <algorithm>
#include <string>
using namespace matrix;
using namespace std;

Diamond::Diamond(const DiamondConf& conf)
  : AbstractController("Diamond", "1.0"),
    conf(conf)
{
  t=0;
  
  //passed from outside ("main.cpp")
  coverage.set(1,1);
  coverage.val(0,0) = 0;
  addInspectableMatrix("coverage", &coverage, false, "terrain coverage with time");

  // this->conf = conf;


  //addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
  //if(conf.useExtendedModel)
    //addInspectableMatrix("S", &S, conf.someInternalParams, "model matrix (sensor branch)");
  //addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  //addInspectableMatrix("L", &L, conf.someInternalParams, "Jacobi matrix");
  //addInspectableMatrix("h", &h, conf.someInternalParams, "controller bias");
  //addInspectableMatrix("b", &b, conf.someInternalParams, "model bias");
  //addInspectableMatrix("M", &M, conf.someInternalParams, "inverse model matrix");
  //addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  //addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");


};

Diamond::~Diamond(){
}


void Diamond::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_sensors= sensornumber;
  number_motors = motornumber;

  for (int i = 0; i < conf.n_layers; i++) {
    // TODO still desiding how to implement Diamond layers
    //Sox* one_layer = make_layer(conf.base_controller_name);
    //one_layer->init(sensornumber, motornumber, randGen);
    internal_layer.push_back(make_layer(conf.base_controller_name));
    internal_layer[i]->init(sensornumber, motornumber, randGen);

    // internal_layer[i] ->setParam("epsM",0.01);
    // internal_layer[i] ->setParam("epsh",0.000);
    // internal_layer[i] ->setParam("synboost",1.5);   // 1.1~1.5
    // internal_layer[i] ->setParam("urate",0.05);
    // internal_layer[i] ->setParam("indnorm",1); // 0 is global normalization
    // internal_layer[i] ->setParam("timedist",4);
    //SoxConf oll_conf = Sox::getDefaultConf();
    //Sox* ol = new Sox(oll_conf);
    //oll = new Sox(Sox::getDefaultConf());
    //internal_layer.push_back(one_layer);
    addInspectableMatrix("M"+to_string(i+1), internal_layer[i]->getpM(), false, "inverse model matrix of layer"+to_string(i+1));
    addInspectableMatrix("C"+to_string(i+1), internal_layer[i]->getpC(), false, "controller matrix of layer "+to_string(i+1));
    addInspectableMatrix("C_avg"+to_string(i+1), internal_layer[i]->getpC_avg(), false,  "Average C controller matrix for feet"+to_string(i+1));
    
    // addInspectableMatrix("L"+to_string(i+1), internal_layer[i]->getpL(), false, "Jacobian matrix of layer"+to_string(i+1));
    // addInspectableMatrix("EvRe"+to_string(i+1), internal_layer[i]->getpEvRe(), false, "Eigenvalue Real Part of layer"+to_string(i+1));
    //addInspectableMatrix("A", &(ol->getA()), conf.someInternalParams, "model matrix");
    //addInspectableMatrix("A", &(oll.getA()), conf.someInternalParams, "model matrix");
  }


  internal_layer[0] ->setParam("epsM",conf.params.l1_epsM);
  internal_layer[0] ->setParam("epsh",conf.params.l1_epsh);
  internal_layer[0] ->setParam("synboost",conf.params.l1_synboost);   // 1.1~1.5
  internal_layer[0] ->setParam("urate",conf.params.l1_urate);
  internal_layer[0] ->setParam("indnorm",conf.params.l1_indnorm); // 0 is global normalization
  internal_layer[0] ->setParam("timedist",conf.params.l1_timedist);
  internal_layer[0] ->setParam("learningrule",conf.params.l1_learningrule);
  internal_layer[0] ->setParam("time_average", conf.params.l1_time_average);
  
  internal_layer[1] ->setParam("epsM",conf.params.l2_epsM);
  internal_layer[1] ->setParam("epsh",conf.params.l2_epsh);
  internal_layer[1] ->setParam("synboost",conf.params.l2_synboost);   // 1.1~1.5
  internal_layer[1] ->setParam("urate",conf.params.l2_urate);
  internal_layer[1] ->setParam("indnorm",conf.params.l2_indnorm); // 0 is global normalization
  internal_layer[1] ->setParam("timedist",conf.params.l2_timedist);
  internal_layer[1] ->setParam("learningrule",conf.params.l2_learningrule);
  internal_layer[1] ->setParam("time_average", conf.params.l2_time_average);

  //std::cout<< std::endl<< internal_layer[0]->getParam("epsM") << std::endl<< std::endl;


  //addInspectableMatrix("M", &M, false, "inverse-model matrix");
  //addInspectableMatrix("h",  &h, false,   "acting controller bias");
  //addInspectableMatrix("C", &C, false, "acting controller matrix");

  // TODO delete from here down
  //A.set(number_sensors, number_motors);
  //S.set(number_sensors, number_sensors);
  //C.set(number_motors, number_sensors);
  //b.set(number_sensors, 1);
  //h.set(number_motors, 1);
  //L.set(number_sensors, number_sensors);
  //v_avg.set(number_sensors, 1);
  //A_native.set(number_sensors, number_motors);
  //C_native.set(number_motors, number_sensors);

  //R.set(number_sensors, number_sensors);


  //C.toId(); // set a to identity matrix;
  ////C*=conf.initFeedbackStrength;

  //S.toId();
  //S*=0.05;

  //// if motor babbling is used then this is overwritten
  //A_native.toId();
  //C_native.toId();
  //C_native*=1.2;

  //y_teaching.set(number_motors, 1);

  //x.set(number_sensors,1);
  //x_smooth.set(number_sensors,1);
  //for (unsigned int k = 0; k < buffersize; k++) {
    //x_buffer[k].set(number_sensors,1);
    //y_buffer[k].set(number_motors,1);

  //}
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void Diamond::step(const sensor* x_, int number_sensors,
                       motor* y_, int number_motors){
  if (conf.n_layers == 1) {  // No Diamond, only DEP
      internal_layer[0]->step(x_, number_sensors, y_, number_motors);
  }

  //---------when there is more than one layer--------- 
  else {
    Matrix x_l;  // Sensor values to be used in stepMV
    
    for (int i = 0; i < conf.n_layers; i++) {
      if (i == 0) {
        x.set(number_sensors,1,x_); // x^prime_0 
        x_l = x;  // First layer it is copy
      }
      // besides the first layer input a copy, the higher level should be prediction or error that cannot predicted
      // or it could be features extracted from the first layer
      else {
        x.set(number_sensors,1,x_); // x^prime_0 
        x_l = internal_layer[i-1]->getPredictionState(conf.time_period * i);
        //x_l = (internal_layer[i-1]->getC()^T) * (
            //((internal_layer[i-1]->getA()^T) * (x - internal_layer[i-1]->getb())).map(g_inv) - internal_layer[i-1]->geth());
      }
      // else {
      //   // Internal world step
      //   matrix::Matrix internal_x = internal_layer[i-2]->getA() 
      //     * internal_layer[i-2]->getLastMotorValues()
      //     + internal_layer[i-2]->getb();  // x^prime_i
      //   x_l = (internal_layer[i-1]->getC()^T) *
      //     (
      //      ((internal_layer[i-1]->getA()^T) *
      //       (internal_x - internal_layer[i-1]->getb())
      //       ).map(g_inv)
      //      - internal_layer[i-1]->geth()
      //      );
      // }

      //----now cauculates the real output for every layer, move one step and learn controller-----
      sensor x_pw[number_sensors];
      x_l.convertToBuffer(x_pw, number_sensors);
      motor y_discard[number_motors];


      //MARK: after getting every layer's virtual input, every layer (besides the final layer), all have the upper
      // layer to instruct the layering for corresponding lower level, except from the last layer (no higher level)
      if (i < conf.n_layers - 1) {
        internal_layer[i]->stepMV(x_pw, number_sensors, y_discard, number_motors,
            internal_layer[i+1]);
        
        if (i == 0) {
          for (int j = 0; j < number_motors; j++)
            y_[j] = y_discard[j]; 
        }

      }
      else {
        //Last layer is only DEP for motors command
        internal_layer[i]->step(x_pw, number_sensors, y_discard, number_motors);
      }


    }
  }


  t++;
};




DEPDiamond* Diamond::make_layer(string name){
  return new DEPDiamond();
}


// performs one step without learning. Calulates motor commands from sensor inputs.
void Diamond::stepNoLearning(const sensor* x_, int number_sensors,
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors
         && (unsigned)number_motors <= this->number_motors);

  //x.set(number_sensors,1,x_); // store sensor values
  //// averaging over the last s4avg values of x_buffer
  //conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
  //if(conf.steps4Averaging > 1)
    //x_smooth += (x - x_smooth)*(1.0/conf.steps4Averaging);
  //else
    //x_smooth = x;
  //x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value
  //// calculate controller values based on current input values (smoothed)
  //Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);
  //// Put new output vector in ring buffer y_buffer
  //y_buffer[t%buffersize] = y;
  //// convert y to motor*
  //y.convertToBuffer(y_, number_motors);
  //update step counter
  //t++;
};


Matrix Diamond::pseudoInvL(int pseudo, const Matrix& L, const Matrix& A, const Matrix& C){
  if(pseudo == 0){
    return L.pseudoInverse();
  }else{
    const Matrix& P = pseudo==1 || pseudo==2 ? A^T : C;
    const Matrix& Q = pseudo==1              ? C^T : A;
    return Q *((P * L * Q)^(-1)) * P;
  }
}


// Shouldn't need any learn function here!! ===== [learn values h,C,A,b,S]
// void Diamond::learn(){
// };


void Diamond::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  //y_teaching= teaching.mapP(0.95,clip);
  //intern_isTeaching=true;
}

void Diamond::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching,
  // that belongs to the distal teaching value by the inverse model.
  //y_teaching = (A.pseudoInverse() * (teaching-b)).mapP(0.95, clip);
  //intern_isTeaching=true;
}

matrix::Matrix Diamond::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix Diamond::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}

list<Matrix> Diamond::getParameters() const {
  return {C,h};
}

int Diamond::setParameters(const list<Matrix>& params){
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
bool Diamond::store(FILE* f) const{
  // save matrix values
  C_update.store(f);
  h.store(f);
  M.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool Diamond::restore(FILE* f){
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

