#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "../Constants.h"
#include "Debugger.h"
#include "models/GripperParams.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {
namespace core {

using namespace models;

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr,
              Eigen::RowVector3d& out_dc_dp);

double ComputeCost(const GripperParams& params,
                   const GripperParams& init_params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr,
                   GripperParams& out_dCost_dParam,
                   Debugger* const debugger);

double ComputeCost1(const GripperParams& params,
                    const GripperParams& init_params,
                    const GripperSettings& settings,
                    const MeshDependentResource& mdr,
                    GripperParams& out_dCost_dParam,
                    Debugger* const debugger);

double ComputeCost_SP(const GripperParams& params,
                      const GripperParams& init_params,
                      const GripperSettings& settings,
                      const MeshDependentResource& remeshed_mdr,
                      GripperParams& out_dCost_dParam, /* unused*/
                      Debugger* const debugger);

double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr);

bool Intersects(const GripperParams& params,
                const GripperSettings& settings,
                const MeshDependentResource& mdr);

typedef double (*CostFunction)(const GripperParams&,
                               const GripperParams&,
                               const GripperSettings&,
                               const MeshDependentResource&,
                               GripperParams&,
                               Debugger* const);

struct CostFunctionItem {
  const char* name;
  CostFunction cost_function;
  CostFunctionEnum cost_enum;
  bool has_grad;
};

const CostFunctionItem kCostFunctions[] = {
    CostFunctionItem{"Gradient-Based",
                     &ComputeCost,
                     CostFunctionEnum::kGradientBased,
                     true},
    CostFunctionItem{"SP", &ComputeCost_SP, CostFunctionEnum::kSP, false}};

}  // namespace core
}  // namespace psg