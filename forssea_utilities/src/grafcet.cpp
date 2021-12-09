//
// Created by david on 14/05/2020.
//

#include <forssea_utilities/grafcet.hpp>
using namespace forssea_utilities;

//
// GrafcetTransition implementation
//
GrafcetTransition::GrafcetTransition(GrafcetCondition* condition, list<int> upstream_step_ids, list<int> downstream_step_ids)
  : condition_(condition)
{
  upstream_step_ids_ = upstream_step_ids;
  downstream_step_ids_ = downstream_step_ids;
}

GrafcetTransition::~GrafcetTransition()
{
  if (condition_ != 0) {
    delete condition_;
    condition_ = 0;
  }

  upstream_steps_.clear();
  downstream_steps_.clear();
}

bool GrafcetTransition::isEnabled()
{
  ///Check if all upstream steps are actives
  ///if one of them is not active, then transition is not enabled
  list<GrafcetStep*>::iterator sIt = upstream_steps_.begin();
  list<GrafcetStep*>::iterator sItEnd = upstream_steps_.end();
  while (sIt != sItEnd) {
    if (!(*sIt)->isActive())
      return false;
    sIt++;
  }
  return true;
}

bool GrafcetTransition::canBeCleared()
{
  if (condition_ == 0)
    return false;

  return condition_->eval();
}

//
// GrafcetStep implementation
//
GrafcetStep::GrafcetStep(const int& id, list<GrafcetTransition*> transitions, Action* action, ActionMode mode)
    : id_(id), active_(false), mode_(mode), action_(action)
{
  list<GrafcetTransition*>::iterator tIt = transitions.begin();
  list<GrafcetTransition*>::iterator tItEnd = transitions.end();
  while (tIt != tItEnd) {
    transitions_.push_back(*tIt);
    tIt++;
  }
}

GrafcetStep::~GrafcetStep()
{
  this->setActive(false);

  ///Delete action (aggregation)
  if (action_ != 0) {
    delete action_;
    action_ = 0;
  }
}

void GrafcetStep::setActive(bool state)
{
  if (active_ == state  && state) {
    if (mode_ == ActionMode::CONTINUE)
      this->action();
  }
  else {
    if (mode_ == ActionMode::RISING_EDGE && state)
      this->action();
    else if (mode_ == ActionMode::FALLING_EDGE && !state)
      this->action();
    else if (mode_ == ActionMode::CONTINUE && state)
      this->action(true);

    active_ = state;
  }
}

bool GrafcetStep::evolve(list<GrafcetStep*>& downstream_steps)
{
  // Check whether a downstream transition can be cleared and return the downstream steps
  // Note : Only one transition can be cleared at a time
  list<GrafcetTransition*>::iterator tIt = transitions_.begin();
  list<GrafcetTransition*>::iterator tItEnd = transitions_.end();

  while (tIt != tItEnd)
  {
    if ((*tIt)->isEnabled()) {
      if ((*tIt)->canBeCleared()) {
        ///Only one transition can be cleared at a time
        (*tIt)->getDownstreamSteps(downstream_steps);
        return true;
      }
    }
    tIt++;
  }

  return false;
}

bool GrafcetStep::action(bool first_activation)
{
  if (action_ == 0)
    return true;

  return action_->execute();
}

//
// Grafcet implementation
//
Grafcet::Grafcet(const string& name, ros::NodeHandle* node_handle, const float& frequency)
 : frequency_(1.0)
{
  node_handle_ = node_handle;
  name_ = name;
  if (frequency > 0.0) frequency_ = frequency;
}

Grafcet::~Grafcet()
{
  ///Clear the active step list
  active_steps_.clear();

  ///Delete all steps
  list<GrafcetStep*>::iterator sIt = steps_.begin();
  list<GrafcetStep*>::iterator sItEnd = steps_.end();
  while (sIt != sItEnd) {
    if (*sIt != 0) {
      delete *sIt;
      *sIt = 0;
    }
    sIt++;
  }
  steps_.clear();

  ///Delete all transitions
  list<GrafcetTransition*>::iterator tIt = transitions_.begin();
  list<GrafcetTransition*>::iterator tItEnd = transitions_.end();
  while (tIt != tItEnd) {
    if (*tIt != 0) {
      delete *tIt;
      *tIt = 0;
    }
    tIt++;
  }
  transitions_.clear();
}

GrafcetStep* Grafcet::getStep(const int& id)
{
  GrafcetStep* step = 0;

  list<GrafcetStep*>::iterator sIt = steps_.begin();
  list<GrafcetStep*>::iterator sItEnd = steps_.end();
  while (sIt != sItEnd) {
    if ((*sIt)->getId() == id) {
      step = *sIt;
      break;
    }

    sIt++;
  }

  return step;

}

void Grafcet::setup()
{
  //Call specificSetup implemented by specific grafcets
  this->specificSetup();

  ///Bind up/downstreamSteps to transitions
  this->bindTransitionToStep();

  return;
}

void Grafcet::run()
{
  if ( node_handle_ != nullptr ) {
    grafcet_scheduler_ = node_handle_->createTimer(ros::Duration(1 / frequency_), &Grafcet::evolve, this, false, true);
  }
}

list<int> Grafcet::getDiagStepIds()
{
  list<int> ds_ids;
  ds_lock_.lock();
  ds_ids = diag_step_ids_;
  ds_lock_.unlock();
  return ds_ids;
}

void Grafcet::addStep(GrafcetStep* step)
{
  list<GrafcetStep*>::iterator sIt = steps_.begin();
  list<GrafcetStep*>::iterator sItEnd = steps_.end();
  while (sIt != sItEnd) {
    if ((*sIt)->getId() == step->getId()) {
      // Can never happen except when designing the grafcet
      return;
    }

    sIt++;
  }

  steps_.push_back(step);

  ///If added step is active, push back it into the activeSteps_ list
  if (step->isActive())
  {
    active_steps_.push_back(step);
  }

  //Add transitions to transitions_ (memory management)
  list<GrafcetTransition*>::iterator tIt = step->getTransitions().begin();
  list<GrafcetTransition*>::iterator tItEnd = step->getTransitions().end();
  while (tIt != tItEnd)
  {
    transitions_.push_back(*tIt);
    tIt++;
  }
}

void Grafcet::evolve(const ros::TimerEvent &event)
{
  list<GrafcetStep*> completed_steps;
  list<GrafcetStep*> new_active_step_list;

  list<GrafcetStep*>::iterator sIt = active_steps_.begin();
  list<GrafcetStep*>::iterator sItEnd = active_steps_.end();
  while (sIt != sItEnd)
  {
    ///Can the current step evolve ?
    list<GrafcetStep*> downstream_steps;
    if ((*sIt)->evolve(downstream_steps))
    {
      completed_steps.push_back(*sIt);

      //Test if step exist
      list<GrafcetStep*>::iterator sItDownstreamSteps = downstream_steps.begin();
      list<GrafcetStep*>::iterator sItEndDownstreamSteps = downstream_steps.end();
      while(sItDownstreamSteps != sItEndDownstreamSteps)
      {
        bool isExist = false;
        list<GrafcetStep*>::iterator sItnewActiveStepList = new_active_step_list.begin();
        list<GrafcetStep*>::iterator sItEndnewActiveStepList = new_active_step_list.end();
        while(sItnewActiveStepList != sItEndnewActiveStepList && !isExist)
        {
          if(( *sItDownstreamSteps )->getId() == ( *sItnewActiveStepList )->getId())
            isExist = true;
          sItnewActiveStepList++;
        }

        //Add step if not exist
        if( !isExist )
          new_active_step_list.insert( new_active_step_list.end(), *sItDownstreamSteps );

        sItDownstreamSteps++;
      }

    }
    else
      new_active_step_list.push_back(*sIt);

    sIt++;
  }

  ///Desactivate completed step
  sIt = completed_steps.begin();
  sItEnd = completed_steps.end();
  while (sIt != sItEnd)
  {
    (*sIt)->setActive(false);
    sIt++;
  }

  ///Update active steps list
  active_steps_ = new_active_step_list;

  ///Activate active steps
  sIt = active_steps_.begin();
  sItEnd = active_steps_.end();
  while (sIt != sItEnd)
  {
    (*sIt)->setActive(true);
    sIt++;
  }

  ds_lock_.lock();
  diag_step_ids_.push_back(active_steps_.front()->getId());
  if ((int)diag_step_ids_.size() > 5) diag_step_ids_.pop_front();
  ds_lock_.unlock();
}

void Grafcet::bindTransitionToStep()
{
  GrafcetStep* step = 0;
  list<int> step_ids;

  list<GrafcetTransition*>::iterator tIt = transitions_.begin();
  list<GrafcetTransition*>::iterator tItEnd = transitions_.end();
  while (tIt != tItEnd)
  {
    step_ids = (*tIt)->getUpstreamStepIds();
    list<int>::iterator iIt = step_ids.begin();
    list<int>::iterator iItEnd = step_ids.end();
    while (iIt != iItEnd)
    {
      step = this->getStep(*iIt);
      if (step != 0)
        (*tIt)->addUpstreamStep(step);

      iIt++;
    }

    step_ids.clear();
    step_ids = (*tIt)->getDownstreamStepIds();
    iIt = step_ids.begin();
    iItEnd = step_ids.end();
    while (iIt != iItEnd)
    {
      step = this->getStep(*iIt);
      if (step != 0)
        (*tIt)->addDownstreamStep(step);

      iIt++;
    }

    step_ids.clear();

    tIt++;
  }
}

vector<int> Grafcet::getActiveStepIds()
{
  vector<int> active_step_ids;
  list<GrafcetStep*>::iterator gsIt = active_steps_.begin();
  list<GrafcetStep*>::iterator gsItEnd = active_steps_.end();
  while (gsIt != gsItEnd)
  {
    active_step_ids.push_back((*gsIt)->getId());
    gsIt++;
  }

  return active_step_ids;
}
