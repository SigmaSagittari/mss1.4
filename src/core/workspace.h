#pragma once

#include "algo/probability_engine/observed_board.h"
#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/structure.h"
#include "algo/probability_engine/shape_solver/shape_solver_common.h"
#include "algo/probability_engine/shape_solver/dfs_solver.h"
#include "algo/probability_engine/shape_solver/graph_solver/graph_solver.h"
#include "algo/probability_engine/shape_solver/graph_solver/graph_solver_dp.h"
#include "algo/probability_engine/shape_solver/graph_solver/graph_solver_order.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/probability_engine/probability/probability_external.h"
#include "algo/probability_engine/probability/probability.h"
#include "algo/probability_engine/probability/observe.h"
#include "algo/probability_engine/bruteforce/bruteforce_common.h"
#include "algo/probability_engine/bruteforce/bruteforce_normal.h"
#include "algo/probability_engine/bruteforce/multimask/bruteforce_multimask.h"
#include "algo/probability_engine/bruteforce/multimask/bruteforce_multimask_rootparallel.h"
#include "algo/probability_engine/bruteforce/bruteforce.h"
#include "algo/winrate_solver/native_solver/logic_structure.h"
#include "algo/winrate_solver/native_solver/logic_component_solver.h"
#include "algo/winrate_solver/native_solver/native_solver.h"
#include "algo/winrate_solver/java_transplant/pseudo_helper.h"
#include "algo/winrate_solver/java_transplant/long_term_risk_helper.h"
#include "algo/winrate_solver/java_transplant/java_evaluate.h"
#include "core/utility/combinatorics.h"
#include "core/utility/radix_sort.h"

namespace mss {

struct Workspace {
    Binom::Workspace binom;
    Basic::Workspace basic;
    ShapeSolver::Workspace shapeSolver;
    Structure::Workspace structure;
    Probability::Workspace probability{shapeSolver, binom};
    BruteForce::Workspace bruteForce{shapeSolver};
    radix_sort::Workspace radixSort;
    LongTermRiskReference::Workspace longTermRisk{basic, structure, probability};
    JavaEvaluate::Workspace javaEvaluate{longTermRisk};
    LogicComponentSolver::Workspace logicComponentSolver{probability, bruteForce};
};

} // namespace mss
