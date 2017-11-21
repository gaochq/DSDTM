//
// Created by buyi on 17-11-21.
//

#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

#include "G2oTypes.h"

namespace g2o {

    using namespace std;
    using namespace Eigen;

    Vector2D project2d(const Vector3D& v)  {
        Vector2D res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector3D unproject2d(const Vector2D& v)  {
        Vector3D res;
        res(0) = v(0);
        res(1) = v(1);
        res(2) = 1;
        return res;
    }

    inline Vector3D invert_depth(const Vector3D & x){
        return unproject2d(x.head<2>())/x[2];
    }


    VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, Sim3>()
    {
        _marginalized=false;
        _fix_scale = false;
    }

    bool VertexSim3Expmap::read(std::istream& is)
    {
        Vector7d cam2world;
        for (int i=0; i<6; i++){
            is >> cam2world[i];
        }
        is >> cam2world[6];
//    if (! is) {
//      // if the scale is not specified we set it to 1;
//      std::cerr << "!s";
//      cam2world[6]=0.;
//    }

        for (int i=0; i<2; i++)
        {
            is >> _focal_length1[i];
        }
        for (int i=0; i<2; i++)
        {
            is >> _principle_point1[i];
        }

        setEstimate(Sim3(cam2world).inverse());
        return true;
    }

    bool VertexSim3Expmap::write(std::ostream& os) const
    {
        Sim3 cam2world(estimate().inverse());
        Vector7d lv=cam2world.log();
        for (int i=0; i<7; i++){
            os << lv[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _focal_length1[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _principle_point1[i] << " ";
        }
        return os.good();
    }

    EdgeSim3::EdgeSim3() :
            BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>()
    {
    }

    bool EdgeSim3::read(std::istream& is)
    {
        Vector7d v7;
        for (int i=0; i<7; i++){
            is >> v7[i];
        }

        Sim3 cam2world(v7);
        setMeasurement(cam2world.inverse());

        for (int i=0; i<7; i++)
            for (int j=i; j<7; j++)
            {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSim3::write(std::ostream& os) const
    {
        Sim3 cam2world(measurement().inverse());
        Vector7d v7 = cam2world.log();
        for (int i=0; i<7; i++)
        {
            os  << v7[i] << " ";
        }
        for (int i=0; i<7; i++)
            for (int j=i; j<7; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

/**Sim3ProjectXYZ*/

    EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() :
            BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
    {}

    bool EdgeSim3ProjectXYZ::read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
            is >> _measurement[i];
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

/**InverseSim3ProjectXYZ*/

    EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
            BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
    {}

    bool EdgeInverseSim3ProjectXYZ::read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
            is >> _measurement[i];
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZ::linearizeOplus() {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3d xyz = vi->estimate();
        Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        Matrix<double,2,3> tmp;
        tmp(0,0) = fx;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*fx;

        tmp(1,0) = 0;
        tmp(1,1) = fy;
        tmp(1,2) = -y/z*fy;

        _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

        _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
        _jacobianOplusXj(0,2) = y/z *fx;
        _jacobianOplusXj(0,3) = -1./z *fx;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *fx;

        _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
        _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
        _jacobianOplusXj(1,2) = -x/z *fy;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *fy;
        _jacobianOplusXj(1,5) = y/z_2 *fy;
    }

    Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz, const float &bf) const{
        const float invz = 1.0f/trans_xyz[2];
        Vector3d res;
        res[0] = trans_xyz[0]*invz*fx + cx;
        res[1] = trans_xyz[1]*invz*fy + cy;
        res[2] = res[0] - bf*invz;
        return res;
    }

    EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>()
    {}

    bool EdgeStereoSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<=3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeStereoSE3ProjectXYZ::write(std::ostream& os) const {

        for (int i=0; i<=3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3d xyz = vi->estimate();
        Vector3d xyz_trans = T.map(xyz);

        const Matrix3d R =  T.rotation().toRotationMatrix();

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
        _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
        _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

        _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
        _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
        _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

        _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
        _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
        _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

        _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
        _jacobianOplusXj(0,2) = y/z *fx;
        _jacobianOplusXj(0,3) = -1./z *fx;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *fx;

        _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
        _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
        _jacobianOplusXj(1,2) = -x/z *fy;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *fy;
        _jacobianOplusXj(1,5) = y/z_2 *fy;

        _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
        _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
        _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
        _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
        _jacobianOplusXj(2,4) = 0;
        _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
    }


//Only Pose

    bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;

        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;
    }

    Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }


    Vector3d EdgeStereoSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
        const float invz = 1.0f/trans_xyz[2];
        Vector3d res;
        res[0] = trans_xyz[0]*invz*fx + cx;
        res[1] = trans_xyz[1]*invz*fy + cy;
        res[2] = res[0] - bf*invz;
        return res;
    }


    bool EdgeStereoSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<=3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeStereoSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

        for (int i=0; i<=3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;

        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;

        _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
        _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
        _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
        _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
    }

}// namespace g2o