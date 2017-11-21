//
// Created by buyi on 17-11-21.
//

#ifndef _G20_TYPE_H_
#define _G20_TYPE_H_

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/se3_ops.h>
#include <g2o/types/sba/types_sba.h>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/sim3.h>

#include <Eigen/Geometry>
#include "Camera.h"

// - Added EdgeInverseSim3ProjectXYZ EdgeSE3ProjectXYZ EdgeSE3ProjectXYZOnlyPose EdgeStereoSE3ProjectXYZOnlyPose EdgeInverseSim3ProjectXYZ
// - Modified VertexSim3Expmap to represent relative transformation between two cameras. Includes calibration of both cameras.
namespace g2o {

    using namespace std;
    using namespace Eigen;

/**
* \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
* the parameterization for the increments constructed is a 7d vector
* (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
*/
    class VertexSim3Expmap : public BaseVertex<7, Sim3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSim3Expmap();
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual void setToOriginImpl() {
            _estimate = Sim3();
        }

        virtual void oplusImpl(const double* update_)
        {
            Eigen::Map<Vector7d> update(const_cast<double*>(update_));

            if (_fix_scale)
                update[6] = 0;

            Sim3 s(update);
            setEstimate(s*estimate());
        }

        Vector2d _principle_point1, _principle_point2;
        Vector2d _focal_length1, _focal_length2;

        Vector2d cam_map1(const Vector2d & v) const
        {
            Vector2d res;
            res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
            res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
            return res;
        }

        Vector2d cam_map2(const Vector2d & v) const
        {
            Vector2d res;
            res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
            res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
            return res;
        }

        bool _fix_scale;

    protected:
    };

/**
* \brief 7D edge between two Vertex7
*/
    class EdgeSim3 : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3();
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
        void computeError()
        {
            const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
            const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

            Sim3 C(_measurement);
            Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
            _error = error_.log();
        }

        virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
        virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
        {
            VertexSim3Expmap* v1 = static_cast<VertexSim3Expmap*>(_vertices[0]);
            VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
            if (from.count(v1) > 0)
                v2->setEstimate(measurement()*v1->estimate());
            else
                v1->setEstimate(measurement().inverse()*v2->estimate());
        }
    };


/**/
    class EdgeSim3ProjectXYZ : public  BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3ProjectXYZ();
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        void computeError()
        {
            const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

            Vector2d obs(_measurement);
            _error = obs-v1->cam_map1(project(v1->estimate().map(v2->estimate())));
        }

        // virtual void linearizeOplus();

    };

/**/
    class EdgeInverseSim3ProjectXYZ : public  BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeInverseSim3ProjectXYZ();
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        void computeError()
        {
            const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

            Vector2d obs(_measurement);
            _error = obs-v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())));
        }

        // virtual void linearizeOplus();

    };


    class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Vector2d obs(_measurement);
            _error = obs-cam_project(v1->estimate().map(v2->estimate()));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            return (v1->estimate().map(v2->estimate()))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        double fx, fy, cx, cy;
    };

    class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeStereoSE3ProjectXYZ();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Vector3d obs(_measurement);
            _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            return (v1->estimate().map(v2->estimate()))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const;

        double fx, fy, cx, cy, bf;
    };

    class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPose(){}

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()
        {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            Vector2d obs(_measurement);
            _error = obs-cam_project(v1->estimate().map(Xw));

            if(abs(_error(0))>5 || abs(_error(1))>5)
            {
                DLOG(ERROR)<< "Wrong error";
                _error<< 0 ,0;
            }
            _error = obs-cam_project(v1->estimate().map(Xw));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            return (v1->estimate().map(Xw))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        Vector3d Xw;
        double fx, fy, cx, cy;
    };


    class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeStereoSE3ProjectXYZOnlyPose(){}

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            Vector3d obs(_measurement);
            _error = obs - cam_project(v1->estimate().map(Xw));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            return (v1->estimate().map(Xw))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector3d cam_project(const Vector3d & trans_xyz) const;

        Vector3d Xw;
        double fx, fy, cx, cy, bf;
    };

}// namespce g2o

#endif
