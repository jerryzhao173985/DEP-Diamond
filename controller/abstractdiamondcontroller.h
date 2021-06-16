#ifndef __ABSTRACTDIAMONDCONTROLLER_H
#define __ABSTRACTDIAMONDCONTROLLER_H

#include <stdio.h>
#include <list>
#include <map>
#include <selforg/configurable.h>
#include <selforg/inspectable.h>
#include <selforg/storeable.h>
#include <selforg/randomgenerator.h>
#include <selforg/sensormotorinfo.h>
#include <selforg/abstractcontroller.h>

/**
 * Abstract class for robot controller (with some basic functionality).
 * The controller gets a number of input sensor values each timestep
 *  and has to generate a number of output motor values.
 *
 * Interface assumes the following usage:
 *  - init() is called first to initialise the dimension of sensor- and motor space
 *  - each time step
 *     either step() or stepNoLearning() is called to ask the controller for motor values.
 */
class AbstractDiamondController : public AbstractController {
public:
  typedef double sensor;
  typedef double motor;

  /// contructor (hint: use $ID$ for revision)
  AbstractDiamondController(const std::string& name, const std::string& revision)
  : AbstractController(name, revision) {}

  /** returns the prediction of sensors for next time step */
  virtual sensor* getPredictionState() = 0;
  virtual matrix::Matrix getA();
};

#endif
