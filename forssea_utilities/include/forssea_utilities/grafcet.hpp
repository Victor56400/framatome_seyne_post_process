//
// Created by david on 13/05/2020.
//

#ifndef SRC_GRAFCET_HPP
#define SRC_GRAFCET_HPP

#include <string>
#include <list>
#include <vector>
using namespace std;

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

namespace forssea_utilities {

class Grafcet;
class GrafcetStep;

class GrafcetCondition {
public:

  GrafcetCondition(const string& id, Grafcet* grafcet) : id_(id), grafcet_(grafcet) {
  }

  string getId() { return id_; }

  virtual bool eval() { return false; }

protected:

  string id_;
  Grafcet* grafcet_;

};

class GrafcetTransition {
public:

  GrafcetTransition(GrafcetCondition* condition, list<int> upstream_step_ids, list<int> downstream_step_ids);
  virtual ~GrafcetTransition();

  void setCondtion(GrafcetCondition* condition) { condition_ = condition; }

  list<int> getUpstreamStepIds() const { return upstream_step_ids_;  }
  void addUpstreamStep(GrafcetStep* step) { upstream_steps_.push_back(step); }
  list<int> getDownstreamStepIds() const { return downstream_step_ids_; }
  void addDownstreamStep(GrafcetStep* step) { downstream_steps_.push_back(step); }
  void getDownstreamSteps(list<GrafcetStep*>& downstream_steps) { downstream_steps = downstream_steps_; }

  bool isEnabled();
  bool canBeCleared();

protected:

  bool m_canBeCleared;

  ///Transition condition (aggregation)
  GrafcetCondition* condition_;

  ///Associated upstream and downstream steps
  list<int> upstream_step_ids_;
  list<GrafcetStep*> upstream_steps_;
  list<int> downstream_step_ids_;
  list<GrafcetStep*> downstream_steps_;

};

enum ActionMode {
  RISING_EDGE,
  CONTINUE,
  FALLING_EDGE
};

class Action {
public:

  Action(Grafcet* grafcet) : is_initialized_(false), grafcet_(grafcet) {}

  bool isInitialized() { return is_initialized_; }

  virtual void initialize() { is_initialized_ = true; }

  virtual bool execute() = 0;

protected:

  bool is_initialized_;
  Grafcet* grafcet_;
};

class GrafcetStep {
public:

  GrafcetStep(const int& iId,
              list<GrafcetTransition*> transitions,
              Action* action = 0,
              ActionMode mode = ActionMode::RISING_EDGE);
  virtual ~GrafcetStep();

  int getId() const { return id_; }

  bool isActive() { return active_; }
  void setActive(bool);

  bool evolve(list<GrafcetStep*>& downstream_steps);

  list<GrafcetTransition*>& getTransitions() { return transitions_; }

private:

  bool action(bool first_activation = false);

  int id_;

  bool active_;
  ActionMode mode_;

  list<GrafcetTransition*> transitions_;

  Action* action_;

};

class Grafcet {
public:

  Grafcet(const string& name, ros::NodeHandle* node_handle, const float& frequency);
  virtual ~Grafcet();

  GrafcetStep* getStep(const int& id);

  // Build the grafcet internal structure
  void setup();

  // From this point, the grafcet will evolve depending on its inputs
  void run();

  // Returns a list of the last step ids for diagnosis
  list<int> getDiagStepIds();

protected:

  void addStep(GrafcetStep* step);

  void addTransition(GrafcetTransition* transition);

private:

  virtual void specificSetup() = 0;

  void evolve(const ros::TimerEvent &event);

  void bindTransitionToStep();

  vector<int> getActiveStepIds();

  string name_;
  ros::NodeHandle* node_handle_;
  float frequency_;

  list<GrafcetStep*> steps_;
  list<GrafcetTransition*> transitions_;

  list<GrafcetStep*> active_steps_;

  list<int> diag_step_ids_;
  boost::mutex ds_lock_;
  ros::Timer grafcet_scheduler_;
};

}

#endif //SRC_GRAFCET_HPP
