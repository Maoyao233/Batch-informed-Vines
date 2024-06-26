#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BIVstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <boost/program_options.hpp>
#include <fstream>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/util/Console.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

bool isStateValid(double radiusSquared, const ob::State *state)
{
    const auto& s = *state->as<ob::RealVectorStateSpace::StateType>();
    double x = s[0], y = s[1];
    x = std::abs(x - std::floor(x));
    y = std::abs(y - std::floor(y));
    x = std::min(x, 1. - x);
    y = std::min(y, 1. - y);
    return x * x + y * y > radiusSquared;
}

int main(int argc, char **argv)
{
    int distance, gridLimit, runCount;
    double obstacleRadius, turningRadius, runtimeLimit;

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    po::options_description desc("Options");

    desc.add_options()("help", "show help message")("distance", po::value<int>(&distance)->default_value(3),
                                                     "integer grid distance between start and goal")(
        "obstacle-radius", po::value<double>(&obstacleRadius)->default_value(.45),
        "radius of obstacles")
        ("turning-radius", po::value<double>(&turningRadius)->default_value(.5),
                               "turning radius of robot (ignored for default point robot)")(
        "grid-limit", po::value<int>(&gridLimit)->default_value(10), "size of the grid")(
        "runtime-limit", po::value<double>(&runtimeLimit)->default_value(1), "time limit for every test")(
        "run-count", po::value<int>(&runCount)->default_value(10), "number of times to run each planner");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
        std::cout << desc << "\n";
        return 1;
    }


    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-.5 * gridLimit);
    bounds.setHigh(.5 * gridLimit);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    double radiusSquared = obstacleRadius * obstacleRadius;
    ss.setStateValidityChecker([radiusSquared](const ob::State *state) { return isStateValid(radiusSquared, state); });

    // define start & goal states
    ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
    start[0] = 0.;
    start[1] = 0.5;
    goal[0] = 0;
    goal[1] = (double)distance + .5;
    ss.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 0.05 (absolute)
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.05 / gridLimit);
    ss.getProblemDefinition()->setOptimizationObjective(
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation()));
    
    ss.setPlanner(std::make_shared<og::BIVstar>(ss.getSpaceInformation()));

    if (ss.solve(0.5)) {
        std::ofstream fout{"path.txt"};
        ss.getSolutionPath().printAsMatrix(fout);
    }

    /*
    // by default, use the Benchmark class
    double memoryLimit = 4096;
    ot::Benchmark::Request request(runtimeLimit, memoryLimit, runCount);
    ot::Benchmark b(ss, "CircleGrid");

    b.addPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<og::BITstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<og::BIVstar>(ss.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile("circleGrid.log");*/

    exit(0);
}