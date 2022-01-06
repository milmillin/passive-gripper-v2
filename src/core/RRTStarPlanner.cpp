#include "RRTStarPlanner.h"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <thread>

#include "../Constants.h"
#include "CostFunctions.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace psg {
namespace core {

bool RRTStarPlanner::Optimize(const PassiveGripper& psg,
                              Trajectory& out_trajectory) {
  using State = ob::ScopedState<ob::RealVectorStateSpace>;
  auto space(std::make_shared<ob::RealVectorStateSpace>(kNumDOFs));
  ob::RealVectorBounds bounds(kNumDOFs);
  State start(space);
  State goal(space);

  Pose p_min = psg.GetTrajectory().front().min(psg.GetTrajectory().back());
  Pose p_max = psg.GetTrajectory().front().max(psg.GetTrajectory().back());

  for (size_t i = 0; i < kNumDOFs; i++) {
    bounds.low[i] = p_min(i) - psg.GetOptSettings().trajectory_wiggle[i];
    bounds.high[i] = p_max(i) + psg.GetOptSettings().trajectory_wiggle[i];
    start[i] = psg.GetTrajectory().front()(i);
    goal[i] = psg.GetTrajectory().back()(i);
  }
  space->setBounds(bounds);

  const auto& fingers = psg.GetFingers();
  Eigen::MatrixXd cps(fingers.size(), 3);
  for (size_t i = 0; i < fingers.size(); i++) {
    cps.row(i) = fingers[i].row(0);
  }
  cps = (psg.GetFingerTransInv() * cps.transpose().colwise().homogeneous())
            .transpose();

  auto validity_checker = [&psg, &cps](const ob::State* state) -> bool {
    const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
    Pose p;
    std::copy_n(s->values, kNumDOFs, p.data());
    Eigen::Affine3d trans = robots::Forward(p);
    double distance = std::numeric_limits<double>::max();
    const auto& mdr = psg.GetMDR();
    Eigen::RowVector3d c;  // unused;
    for (size_t i = 0; i < cps.rows(); i++) {
      double cur_dist = GetDist(
          trans * (Eigen::Vector3d)cps.row(i), psg.GetCostSettings(), mdr, c);
      distance = std::min(distance, cur_dist);
    }

    // double distance = MinDistanceAtPose(psg.GetFingers(),
    // psg.GetFingerTransInv(),
    // psg.GetMDR(),
    // psg.GetSettings(),
    // p);
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

  std::cout << "hardware_concurrency: " << std::thread::hardware_concurrency()
            << std::endl;

  auto planner = std::make_shared<og::pSBL>(si_space);
  planner->setProblemDefinition(pdef);
  planner->setRange(kDegToRad * 0.1);
  planner->setThreadCount(std::thread::hardware_concurrency());
  planner->setup();

  std::cout << "Testttt" << std::endl;
  // ob::CostConvergenceTerminationCondition condition(pdef, 0, 1);
  ob::PlannerStatus solved =
      planner->solve(ob::exactSolnPlannerTerminationCondition(pdef));
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