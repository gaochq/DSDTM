//
// Created by buyi on 17-10-16.
//

#include "Feature_detection.h"


namespace DSDTM
{
    Feature_detector::Feature_detector(const int Image_height, const int Image_width):
            mImg_height(Image_height), mImg_width(Image_width)

    {
        mCell_size   = Config::Get<int>("Camera.CellSize");
        mPyr_levels  = Config::Get<int>("Camera.PyraLevels");

        mGrid_rows = ceil(static_cast<double>(1.0*Image_height/mCell_size));
        mGrid_cols = ceil(static_cast<double>(1.0*Image_width/mCell_size));
        mvGrid_occupy.resize(mGrid_rows*mGrid_cols, false);
    }

    Feature_detector::~Feature_detector()
    {

    }

    int Feature_detector::Get_CellIndex(int x, int y, int level)
    {
        int Index;
        int scale = (1<<level);
        Index = (scale*y)/mCell_size*mGrid_cols + (scale*x)/mCell_size;

        return Index;
    }

    void Feature_detector::Set_CellIndexOccupy(cv::Point2f &px)
    {
        int Index = static_cast<int>((px.y/mCell_size)*mGrid_cols) + static_cast<int>(px.x/mCell_size);
        mvGrid_occupy[Index] = true;
    }

    void Feature_detector::Set_ExistingFeatures(const mFeatures &features)
    {
        mvGrid_occupy.resize(mGrid_rows*mGrid_cols, false);
        std::for_each(features.begin(), features.end(), [&](Feature feature)
        {
            Set_CellIndexOccupy(feature.mpx);
        });
    }

    void Feature_detector::Set_ExistingFeatures(const std::vector<cv::Point2f> &features)
    {
        int Index;
        mvGrid_occupy.resize(mGrid_rows*mGrid_cols, false);

        for (int i = 0; i < features.size(); ++i)
        {
            Index = static_cast<int>((features[i].y/mCell_size)*mGrid_cols) + static_cast<int>(features[i].x/mCell_size);
            mvGrid_occupy[Index] = true;
        }
    }

    void Feature_detector::ResetGrid()
    {
        std::fill(mvGrid_occupy.begin(), mvGrid_occupy.end(), false);
    }

    void Feature_detector::detect(Frame* frame, const double detection_threshold)
    {
        Corners corners(mGrid_cols*mGrid_rows, Corner(0, 0, detection_threshold, 0, 0.0f));

        for(int L=0; L<mPyr_levels; ++L)
        {
            const int scale = (1<<L);
            std::vector<fast::fast_xy> fast_corners;
#if __SSE2__
            fast::fast_corner_detect_10_sse2((fast::fast_byte*) frame->mvImg_Pyr[L].data, frame->mvImg_Pyr[L].cols,
                                             frame->mvImg_Pyr[L].rows, frame->mvImg_Pyr[L].cols, 20, fast_corners);
#elif HAVE_FAST_NEON
            fast::fast_corner_detect_9_neon((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
                                             img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
            fast::fast_corner_detect_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
                                        img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif
            std::vector<int> scores, nm_corners;
            fast::fast_corner_score_10((fast::fast_byte*) frame->mvImg_Pyr[L].data, frame->mvImg_Pyr[L].cols, fast_corners, 20, scores);
            fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

            for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
            {
                fast::fast_xy& xy = fast_corners.at(*it);
                const int k = static_cast<int>((xy.y*scale)/mCell_size)*mGrid_cols
                              + static_cast<int>((xy.x*scale)/mCell_size);
                if(mvGrid_occupy[k])
                    continue;
                const float score = shiTomasiScore(frame->mvImg_Pyr[L], xy.x, xy.y);
                if(score > corners.at(k).score)
                    corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
            }
        }

        // Create feature for every corner that has high enough corner score
        std::for_each(corners.begin(), corners.end(), [&](Corner& c)
        {
            if(c.score > detection_threshold)
                frame->mvFeatures.push_back(Feature(frame, cv::Point2f(c.x, c.y), c.level));
        });

        ResetGrid();
    }

    //! http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html
    float Feature_detector::shiTomasiScore(const cv::Mat &img, int u, int v)
    {
        assert(img.type() == CV_8UC1);

        float dXX = 0.0;
        float dYY = 0.0;
        float dXY = 0.0;
        const int halfbox_size = 4;
        const int box_size = 2*halfbox_size;
        const int box_area = box_size*box_size;
        const int x_min = u-halfbox_size;
        const int x_max = u+halfbox_size;
        const int y_min = v-halfbox_size;
        const int y_max = v+halfbox_size;

        if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
            return 0.0; // patch is too close to the boundary

        //! Get the gradient sum of the patch
        const int stride = img.step.p[0];
        for( int y=y_min; y<y_max; ++y )
        {
            const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
            const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
            const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
            const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
            for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
            {
                float dx = *ptr_right - *ptr_left;
                float dy = *ptr_bottom - *ptr_top;
                dXX += dx*dx;
                dYY += dy*dy;
                dXY += dx*dy;
            }
        }

        // Find and return smaller eigenvalue:
        dXX = dXX / (2.0 * box_area);
        dYY = dYY / (2.0 * box_area);
        dXY = dXY / (2.0 * box_area);
        return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
    }


}// namespace DSDTM