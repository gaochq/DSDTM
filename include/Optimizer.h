//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_OPTIMIZER_H
#define DSDTM_OPTIMIZER_H

#include "Camera.h"
#include "Frame.h"
#include "ceres/ceres.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "G2oTypes.h"

#include <Eigen/StdVector>

namespace DSDTM
{

class Frame;
class PoseGraph_Problem;

class Optimizer
{
public:
    Optimizer();
    ~Optimizer();

    static void PoseSolver(Frame &tCurFrame, int tIterations = 5);
    static double *se3ToDouble(Eigen::Matrix<double, 6, 1> tso3);


};

} //namespace DSDTM


#endif //DSDTM_OPTIMIZER_H
