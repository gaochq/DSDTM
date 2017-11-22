//
// Created by buyi on 17-11-9.
//

#include "Optimizer.h"


namespace DSDTM
{
Optimizer::Optimizer()
{

}

Optimizer::~Optimizer()
{

}

void Optimizer::PoseSolver(Frame &tCurFrame, int tIterations)
{
    const double tchiThreshold = 5.991;
    //! Create solver
    g2o::SparseOptimizer mOptimizer;

    //! Set Solver Options <6 dims pose and 3 dims landmark>
    g2o::BlockSolver_6_3::LinearSolverType *LinearSolver;
    LinearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *Block_Solver = new g2o::BlockSolver_6_3(LinearSolver);

    //! Set LM
    g2o::OptimizationAlgorithmLevenberg *Solver = new g2o::OptimizationAlgorithmLevenberg(Block_Solver);
    mOptimizer.setAlgorithm(Solver);
    mOptimizer.setVerbose(true);

    //! Add vertex
    g2o::VertexSE3Expmap *mVertex = new g2o::VertexSE3Expmap;
    mVertex->setEstimate(g2o::SE3Quat(tCurFrame.mT_c2w.so3().matrix(), tCurFrame.mT_c2w.translation()));
    mVertex->setId(0);
    mVertex->setFixed(false);

    mOptimizer.addVertex(mVertex);

    //! Add edge
    std::map<size_t, MapPoint*> tObservations = tCurFrame.Get_Observations();
    int N = tObservations.size();
    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> mvEdges;
    mvEdges.reserve(N);

    size_t tEdgeNum = 0;
    //TODO Add mutex_lock for Mappoint

    for (auto iter = tObservations.begin(); iter!=tObservations.end(); ++iter)
    {
        MapPoint* tMPoint = iter->second;

        if(tMPoint)
        {
            Eigen::Matrix<double,2,1> tObs;
            tObs << tCurFrame.mvFeatures[iter->first].mpx.x,
                    tCurFrame.mvFeatures[iter->first].mpx.y;

            g2o::EdgeSE3ProjectXYZOnlyPose *tEdge = new g2o::EdgeSE3ProjectXYZOnlyPose;
            tEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer.vertex(0)));
            tEdge->setMeasurement(tObs);
            tEdge->setInformation(Eigen::Matrix2d::Identity());
            //! Add Kernel function
            g2o::RobustKernelHuber *mRkHuber = new g2o::RobustKernelHuber;
            tEdge->setRobustKernel(mRkHuber);
            mRkHuber->setDelta(5.991);          //from ORB-SLAM

            tEdge->fx = tCurFrame.mCamera->mfx;
            tEdge->fy = tCurFrame.mCamera->mfy;
            tEdge->cx = tCurFrame.mCamera->mcx;
            tEdge->cy = tCurFrame.mCamera->mcy;

            tEdge->Xw = tMPoint->Get_Pose();

            mOptimizer.addEdge(tEdge);
            mvEdges.push_back(tEdge);
            tEdgeNum++;
        }
    }

    //! Start solver
    mOptimizer.initializeOptimization(0);
    mOptimizer.optimize(tIterations);

    for (int i = 0; i < mvEdges.size(); ++i)
    {
        g2o::EdgeSE3ProjectXYZOnlyPose* tEdge = mvEdges[i];
        tEdge->computeError();

        if(tEdge->chi2() > tchiThreshold)
            tCurFrame.mvbOutliers[i] = false;
    }

    //! Update frame pose
    g2o::VertexSE3Expmap *tSE3_Recov = static_cast<g2o::VertexSE3Expmap*>(mOptimizer.vertex(0));
    g2o::SE3Quat tPose_Recov = tSE3_Recov->estimate();

    DLOG(INFO) << tPose_Recov;
    tCurFrame.Set_Pose(Sophus::SE3(tPose_Recov.rotation(), tPose_Recov.translation()));
}

double *Optimizer::se3ToDouble(Eigen::Matrix<double, 6, 1> tse3)
{
    double *so3_tmp = new double[6];

    so3_tmp[0] = tse3(0, 0);
    so3_tmp[1] = tse3(1, 0);
    so3_tmp[2] = tse3(2, 0);
    so3_tmp[3] = tse3(3, 0);
    so3_tmp[4] = tse3(4, 0);
    so3_tmp[5] = tse3(5, 0);

    return so3_tmp;
}

}