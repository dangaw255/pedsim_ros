/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/groupwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/individualwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/queueingplanner.h>
#include <pedsim_simulator/waypointplanner/shoppingplanner.h>

#include <ros/ros.h>

AgentStateMachine::AgentStateMachine(Agent* agentIn) {
  // initialize values
  agent = agentIn;
  individualPlanner = nullptr;
  queueingPlanner = nullptr;
  groupWaypointPlanner = nullptr;
  shoppingPlanner = nullptr;
  groupAttraction = nullptr;
  shallLoseAttraction = false;
  // initialize state machine
  state = StateNone;
  stateTalkingBaseTime = 6.0;
  stateWorkingBaseTime = 6.0;
  stateLiftingForksBaseTime = 6.0;
  stateLoadingBaseTime = 6.0;
  stateLoweringForksBaseTime = 6.0;
  stateTellStoryBaseTime = 6.0;
  stateGroupTalkingBaseTime = 12.0;
}

AgentStateMachine::~AgentStateMachine() {
  // clean up
  delete individualPlanner;
  delete queueingPlanner;
  delete groupWaypointPlanner;
  delete shoppingPlanner;
}

void AgentStateMachine::loseAttraction() {
  // set mark to lose attraction
  shallLoseAttraction = true;
}

void AgentStateMachine::doStateTransition() {
  // determine new state
  // → randomly get attracted by attractions
  if ((state != StateShopping) && (state != StateQueueing)) {
    double distance = INFINITY;
    AttractionArea* attraction = nullptr;
    bool hasGroupAttraction = checkGroupForAttractions(&attraction);
    if (hasGroupAttraction) {
      // inherit groups' attraction
      groupAttraction = attraction;

      normalState = state;
      activateState(StateShopping);
      return;
    } else {
      // TODO: attraction must be visible!
      attraction = SCENE.getClosestAttraction(agent->getPosition(), &distance);

      if (attraction != nullptr) {
        // check whether agent is attracted
        // NOTE: The Cumulative Geometric Distribution determines the
        //       number of Bernoulli trials needed to get one success.
        //       → CDF(X) = 1-(1-p)^k   with k = the number of trials
        double baseProbability = 0.02;
        double maxAttractionDist = 7;
        // → probability dependents on strength, distance,
        //   and whether another group member are attracted
        double probability = baseProbability * attraction->getStrength() *
                             ((distance < maxAttractionDist)
                                  ? (1 - (distance / maxAttractionDist))
                                  : 0) *
                             CONFIG.getTimeStepSize();
        std::bernoulli_distribution isAttracted(probability);

        if (isAttracted(RNG())) {
          normalState = state;
          activateState(StateShopping);
          return;
        }
      }
    }
  }


  // → randomly lose attraction
  if (state == StateShopping) {
    // check whether agent loses attraction
    // TODO: make this dependent from the distance to CoM
    double probability = 0.03;
    std::bernoulli_distribution isAttracted(probability *
                                            CONFIG.getTimeStepSize());

    if (shallLoseAttraction || isAttracted(RNG())) {
      // reactivate previous state
      activateState(normalState);

      // alreade picked a new state, so nothing to do
      return;
    }
  }


  // → operate on waypoints/destinations
  if (state == StateNone) {
    Ped::Twaypoint* destination = agent->updateDestination();
    if (destination == nullptr)
      activateState(StateWaiting);
    else
      activateState(StateWalking);
  }


  // → update destination on arrival
  if (agent->hasCompletedDestination()) {
    agent->updateDestination();
    // start working if vehicle
    if (agent->getType() == Ped::Tagent::AgentType::VEHICLE)
    {
      activateState(StateWorking);
      return;
    } else {
      activateState(StateWalking);
      return;
    }
  }


  // → do work
  if (state == StateWorking)
  {
    activateState(StateLiftingForks);
    return;
  }


  // → lift forks
  if (state == StateLiftingForks)
  {
    ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
    if (timePassed.toSec() > stateMaxDuration)
    {
      activateState(StateLoading);
    }
    return;
  }


  // → load stuff
  if (state == StateLoading)
  {
    ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
    if (timePassed.toSec() > stateMaxDuration)
    {
      activateState(StateLoweringForks);
    }
    return;
  }


  // → lower forks
  if (state == StateLoweringForks)
  {
    ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
    if (timePassed.toSec() > stateMaxDuration)
    {
      activateState(StateWalking);
    }
    return;
  }


  // ## do some checks wether to interrupt walking

  // → start telling a story sometimes
  if (state == StateWalking && agent->tellStory()) {
    activateState(StateTellStory);
    return;
  }


  // → start group talking sometimes
  if (state == StateWalking && agent->startGroupTalking()) {
    activateState(StateGroupTalking);
    return;
  }


  // → check wether someone is talking to me
  if ((state == StateWalking) && agent->someoneTalkingToMe()) {
    activateState(StateListening);
    return;
  }


  // → start talking to someone sometimes
  if ((state == StateWalking) && agent->startTalking()) {
    activateState(StateTalking);
    return;
  }


  // → tell story for some time
  if (state == StateTellStory) {
    ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
    if (timePassed.toSec() > stateMaxDuration)
    {
      activateState(StateWalking);
    }
    return;
  }


  // → talk as a group for some time
  if (state == StateGroupTalking) {
    ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
    if (timePassed.toSec() > stateMaxDuration)
    {
      activateState(StateWalking);
    }
    return;
  }


  // → listening
  if (state == StateListening) {
    // check if I am still being talked to
    if (agent->listeningToAgent != nullptr) {
      if (
        agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTellStory ||
        agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateGroupTalking ||
        agent->listeningToAgent->talkingToId == agent->getId()
      ) {
        return;
      }
    }

    activateState(StateWalking);
    return;
  }


  // → talk for some time
  if (state == StateTalking) {
    ros::WallDuration diff = ros::WallTime::now() - startTimestamp;
    if (diff.toSec() > 6.20) {
      activateState(StateWalking);
      return;
    }
  }
}

void AgentStateMachine::activateState(AgentState stateIn) {
  ROS_DEBUG("Agent %d type %d activating state '%s' (time: %f)", agent->getId(), agent->getType(),
            stateToName(stateIn).toStdString().c_str(), SCENE.getTime());

  // de-activate old state
  deactivateState(state);

  // re-activate all forces
  agent->enableAllForces();

  // set state
  state = stateIn;

  Waypoint* destination =
      dynamic_cast<Waypoint*>(agent->getCurrentDestination());

  switch (state) {
    case StateNone:
      agent->setWaypointPlanner(nullptr);
      break;
    case StateWaiting:
      agent->setWaypointPlanner(nullptr);
      break;
    case StateWalking:
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      break;
    case StateQueueing:
      if (queueingPlanner == nullptr)
        queueingPlanner = new QueueingWaypointPlanner();
      queueingPlanner->setAgent(agent);
      queueingPlanner->setDestination(destination);
      agent->setWaypointPlanner(queueingPlanner);
      break;
    case StateGroupWalking:
      if (groupWaypointPlanner == nullptr)
        groupWaypointPlanner = new GroupWaypointPlanner();
      groupWaypointPlanner->setDestination(destination);
      groupWaypointPlanner->setGroup(agent->getGroup());
      agent->setWaypointPlanner(groupWaypointPlanner);
      break;
    case StateShopping:
      {
        shallLoseAttraction = false;
        if (shoppingPlanner == nullptr) shoppingPlanner = new ShoppingPlanner();
        AttractionArea* attraction =
            SCENE.getClosestAttraction(agent->getPosition());
        shoppingPlanner->setAgent(agent);
        shoppingPlanner->setAttraction(attraction);
        agent->setWaypointPlanner(shoppingPlanner);
        agent->disableForce("GroupCoherence");
        agent->disableForce("GroupGaze");

        // keep other agents informed about the attraction
        AgentGroup* group = agent->getGroup();
        if (group != nullptr) {
          foreach (Agent* member, group->getMembers()) {
            if (member == agent) continue;

            AgentStateMachine* memberStateMachine = member->getStateMachine();
            connect(shoppingPlanner, SIGNAL(lostAttraction()), memberStateMachine,
                    SLOT(loseAttraction()));
          }
        }
      }
      break;
    case StateTalking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateTalkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateWorking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateWorkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLiftingForks:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateLiftingForksBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLoading:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateLoadingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLoweringForks:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateLoweringForksBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateTellStory:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateTellStoryBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateGroupTalking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(stateGroupTalkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->enableForce("KeepDistance");
      break;
    case StateListening:
      agent->setWaypointPlanner(nullptr);
      agent->enableForce("KeepDistance");
      agent->setForceFactorSocial(50.0);
      break;
  }

  // inform users
  emit stateChanged(state);
}

void AgentStateMachine::deactivateState(AgentState state) {
  switch (state) {
    case StateTalking:
      // reset talking to id
      agent->talkingToId = -1;
      break;
    case StateListening:
      // reset listening to id
      agent->listeningToId = -1;
      agent->listeningToAgent = nullptr;
      agent->setForceFactorSocial(2.0);
      break;
    case StateShopping:
    {
      // inform other group members
      shoppingPlanner->loseAttraction();

      // don't worry about other group members
      AgentGroup* group = agent->getGroup();
      if (group != nullptr) {
        foreach (Agent* member, group->getMembers()) {
          if (member == agent) continue;

          AgentStateMachine* memberStateMachine = member->getStateMachine();
          disconnect(shoppingPlanner, SIGNAL(lostAttraction()),
                     memberStateMachine, SLOT(loseAttraction()));
        }
      }

      break;
    }
    default:
      break;
  }
}

double AgentStateMachine::getRandomDuration(double baseTime)
{
  uniform_real_distribution<double> Distribution(0.5, 1.5);
  double durationFactor = Distribution(RNG());
  double duration = durationFactor * baseTime;
  return duration;
}

bool AgentStateMachine::checkGroupForAttractions(
    AttractionArea** attractionOut) const {
  AgentGroup* group = agent->getGroup();

  // check whether the agent is even in a group
  if (group == nullptr) {
    if (attractionOut != nullptr) *attractionOut = nullptr;
    return false;
  }

  // check all group members
  foreach (Agent* member, group->getMembers()) {
    // ignore agent himself
    if (member == agent) continue;

    // check whether the group member uses ShoppingPlanner
    WaypointPlanner* planner = member->getWaypointPlanner();
    ShoppingPlanner* typedPlanner = dynamic_cast<ShoppingPlanner*>(planner);
    if (typedPlanner != nullptr) {
      AttractionArea* attraction = typedPlanner->getAttraction();

      if (attraction != nullptr) {
        attractionOut = &attraction;
        return true;
      }
    }
  }

  // no group member is attracted to something
  if (attractionOut != nullptr) *attractionOut = nullptr;
  return false;
}

QString AgentStateMachine::stateToName(AgentState stateIn) {
  switch (stateIn) {
    case StateNone:
      return "None";
    case StateWaiting:
      return "Waiting";
    case StateWalking:
      return "Walking";
    case StateQueueing:
      return "Queueing";
    case StateGroupWalking:
      return "GroupWalking";
    case StateTalking:
      return "Talking";
    case StateShopping:
      return "Shopping";
    case StateWorking:
      return "Working";
    case StateLiftingForks:
      return "LiftingForks";
    case StateLoading:
      return "Loading";
    case StateLoweringForks:
      return "LoweringForks";
    case StateTellStory:
      return "TellStory";
    case StateGroupTalking:
      return "GroupTalking";
    case StateListening:
      return "Listening";
    default:
      return "UnknownState";
  }
}

AgentStateMachine::AgentState AgentStateMachine::getCurrentState() {
  return state;
}
