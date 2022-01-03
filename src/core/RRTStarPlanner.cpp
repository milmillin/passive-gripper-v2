#include "RRTStarPlanner.h"

#include <thread>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>

#include "../Constants.h"
#include "CostFunctions.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace psg {
namespace core {

bool RRTStarPlanner::Optimize(const PassiveGripper& psg, Trajectory& out_trajectory) {
  using State = ob::ScopedState<ob::RealVectorStateSpace>;
  auto space(std::make_shared<ob::RealVectorStateSpace>(kNumDOFs));
  ob::RealVectorBounds bounds(kNumDOFs);
  State start(space);
  State goal(space);
  for (size_t i = 0; i < kNumDOFs; i++) {
    bounds.low[i] = psg.GetTrajectory().front()(i) -
                    psg.GetOptSettings().trajectory_wiggle[i];
    bounds.high[i] = psg.GetTrajectory().front()(i) +
                     psg.GetOptSettings().trajectory_wiggle[i];
    start[i] = psg.GetTrajectory().front()(i);
    goal[i] = psg.GetTrajectory().back()(i);
  }
  space->setBounds(bounds);

  auto validity_checker = [&psg](const ob::State* state) -> bool {
    const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
    Pose p;
    std::copy_n(s->values, kNumDOFs, p.data());
    double distance = MinDistanceAtPose(psg.GetFingers(),
                                        psg.GetFingerTransInv(),
                                        psg.GetMDR(),
                                        psg.GetSettings(),
                                        p);
    // std::cout << "validity_checker" << std::endl;
    // std::cout << p << std::endl;
    // std::cout << distance << std::endl;
    return distance > -1e-5;
  };

  ob::SpaceInformationPtr si_space(
      std::make_shared<ob::SpaceInformation>(space));
  si_space->setStateValidityChecker(validity_checker);
  si_space->setup();

  ob::ProblemDefinitionPtr pdef =
      std::make_shared<ob::ProblemDefinition>(si_space);
  pdef->setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<og::pRRT>(si_space);
  planner->setProblemDefinition(pdef);
  planner->setRange(kDegToRad * 5.);
  planner->setThreadCount(std::thread::hardware_concurrency());
  planner->setup();

  std::cout << "Testttt" << std::endl;
  // ob::CostConvergenceTerminationCondition condition(pdef, 0, 1);
  ob::PlannerStatus solved = planner->solve(ob::exactSolnPlannerTerminationCondition(pdef));
  if (solved) {
    ob::PathPtr path = pdef->getSolutionPath();
    path->print(std::cout);
    const auto& states = path->as<og::PathGeometric>()->getStates();
    out_trajectory.resize(states.size());
    for (size_t i = 0; i < states.size(); i++) {
      const auto* s = states[i]->as<ob::RealVectorStateSpace::StateType>();
      std::copy_n(s->values, kNumDOFs, out_trajectory[i].data());
    }
    return true;
  } else {
    std::cout << "Solved Failed" << std::endl;
    return false;
  }
}
}  // namespace core
}  // namespace psg