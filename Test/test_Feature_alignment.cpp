// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "Camera.h"
#include "Feature_alignment.h"

using namespace Eigen;

void generateRefPatchNoWarpInterpolate(const cv::Mat& img, const Vector2d& px,
                                       cv::Mat& ref_patch_with_border)
{
    // compute interpolation weights
    const int u_r = floor(px[0]);
    const int v_r = floor(px[1]);
    const float subpix_u = px[0]-u_r;
    const float subpix_v = px[1]-v_r;
    const float wTL = (1.0-subpix_u)*(1.0-subpix_v);
    const float wTR = subpix_u * (1.0-subpix_v);
    const float wBL = (1.0-subpix_u)*subpix_v;
    const float wBR = subpix_u * subpix_v;

    // loop through search_patch, interpolate
    ref_patch_with_border = cv::Mat(10, 10, CV_8UC1);
    uint8_t* patch_ptr = ref_patch_with_border.data;
    const int stride = img.step.p[0];
    for(int y=0; y<10; ++y)
    {
        uint8_t* img_ptr = (uint8_t*) img.data  + (v_r+y-4-1)*stride + u_r-4-1;
        for(int x=0; x<10; ++x, ++patch_ptr, ++img_ptr)
            *patch_ptr = wTL*img_ptr[0] + wTR*img_ptr[1] + wBL*img_ptr[stride] + wBR*img_ptr[stride+1];
    }
}

int main(int argc, char **argv)
{

    std::string img_name("/home/buyi/Datasets/SVO/sin2_tex2_h1_v8_d/img/frame_000002_0.png");
    printf("Loading image '%s'\n", img_name.c_str());
    cv::Mat img(cv::imread(img_name, 0));
    assert(img.type() == CV_8UC1);

    // TODO: test on corner/gradient features!
    Vector2d px_true(130.2,120.3);

    // create reference patch with border
    cv::Mat ref_patch_with_border;
    generateRefPatchNoWarpInterpolate(img, px_true, ref_patch_with_border);

    // create reference patch, aligned
    uint8_t ref_patch[64] ;
    uint8_t* ref_patch_ptr = ref_patch;
    for(int y=1; y<9; ++y)
    {
        uint8_t* ref_patch_border_ptr = ref_patch_with_border.data + y*10 + 1;
        for(int x=0; x<8; ++x, ++ref_patch_border_ptr, ++ref_patch_ptr)
            *ref_patch_ptr = *ref_patch_border_ptr;
    }

    Vector2d px_est, px_error(-1.1, -0.8);

    DSDTM::TicToc tc;
    for(int i=0; i<1; ++i)
    {
        px_est = px_true-px_error;
        DSDTM::Feature_Alignment::Align2DGaussNewton(img, ref_patch_with_border.data, ref_patch, 3, px_est);
    }
    Vector2d e = px_est-px_true;
    printf("1000Xalign 2D took %fms, error = %fpx \t (ref i7-W520: 2.306000ms, 0.015102px)\n", tc.toc(), e.norm());



    return 0;
}

