#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot

  // return a value that is always true but uses the two variables we define, so
  // we avoid compiler warnings
  return (const void *)rot != (const void *)pos;
}

void plan() {
  // construct the state space we are planning in
  auto space{std::make_shared<ob::SE3StateSpace>()};

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-10);
  bounds.setHigh(10);

  space->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
      [](const ob::State *state) { return isStateValid(state); });

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

  // this call is optional, but we put it in to get more output information
  ss.setup();
  ss.print();

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(10);

  if (solved) {
    std::cout << "Solutions printed to path.txt" << std::endl;
    // print the path to screen
    // ss.simplifySolution();
    std::ofstream fout{"path.txt"};
    ss.getSolutionPath().printAsMatrix(fout);
  } else
    std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/) {
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  plan();

  return 0;
}