#pragma once

#include <opencv2/core.hpp>
#include "kalman_filter.h"

struct My_RotatedRect {
    float center_x;
    float center_y;
    float width;
    float height;
    float angle;
    float landmarksX1;
    float landmarksY1;
};

class Track {
public:
    // Constructor
    Track();

    // Destructor
    ~Track() = default;

    void Init(const My_RotatedRect& bbox);
    void Predict();
    void Update(const My_RotatedRect& bbox);
    My_RotatedRect GetStateAsBbox() const;
    float GetNIS() const;

    int coast_cycles_ = 0, hit_streak_ = 0;

private:
    Eigen::VectorXd ConvertBboxToObservation(const My_RotatedRect& bbox) const;
    My_RotatedRect ConvertStateToBbox(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};
