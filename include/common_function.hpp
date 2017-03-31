/*
 * common_function.hpp
 *
 *  Created on: Apr 1, 2017
 *      Author: magnus
 */

#ifndef INCLUDE_COMMON_FUNCTION_HPP_
#define INCLUDE_COMMON_FUNCTION_HPP_

#include <iostream>

#include <unordered_map>
#include <vector>
#include <array>

#include <set>

#include <string>

using std::string;

std::unordered_map < string, cv::Scalar> solarized_palette = {
    { string("base03"), cv::Scalar(54, 43, 0)},        // dark bg
    { string("base02"), cv::Scalar(117, 110, 88)},
    { string("base01"), cv::Scalar(66, 54, 7)},
    { string("base3"), cv::Scalar(227, 246, 253)},     // light bg
    { string("yellow"), cv::Scalar(0, 137, 181)},
    { string("orange"), cv::Scalar(22, 75, 203)},
    { string("magenta"), cv::Scalar(130, 54, 211)},
    { string("violet"), cv::Scalar(196, 113, 108)},
    { string("blue"), cv::Scalar(210, 139, 38)},
    { string("cyan"), cv::Scalar(152, 161, 42)},
    { string("green"), cv::Scalar(0, 153, 133)},};

template <typename Dtype>
void cvt2ptsAbc(const Dtype x1, const Dtype y1, const Dtype x2, const Dtype y2,
                Dtype& a, Dtype& b, Dtype& c) {

  Dtype move = x2 - x1;
  Dtype rise = y2 - y1;
  if (move == 0) {
    a = 1;
    b = 0;
    c = -x1;
  } else {
    Dtype slope = rise / move;
    Dtype temp = y1 - slope * x1;
    a = slope;
    b = -1.;
    c = temp;
  }
}

template <typename Dtype>
void ABC_2points(const Dtype A, const Dtype B, const Dtype C, Dtype& x1, Dtype& y1,
    Dtype& x2, Dtype& y2, Dtype X_LO=0, Dtype X_UP=960, Dtype Y_LO=0, Dtype Y_UP=1280) {
  using namespace std;
  // A x + B y + C = 0
  if (B == 0.) {        // x = xxx
    x1 = -C / A;
    x2 = x1;
    y1 = Y_LO;
    y2 = Y_UP;
  } else if (A == 0.) {     // y = yyy
    x1 = X_LO;
    x2 = X_UP;
    y1 = -C / B;
    y2 = y1;
  } else {
    Dtype xs[4];
    Dtype ys[4];
    xs[0] = X_LO;
    ys[0] = -(C + A * xs[0]) / B;
    xs[1] = X_UP;
    ys[1] = -(C + A * xs[1]) / B;

    ys[2] = Y_LO;
    xs[2] = -(C + B * ys[2]) / A;
    ys[3] = Y_UP;
    xs[3] = -(C + B * ys[3]) / A;

    set<pair<Dtype, Dtype>> pts_set;
    for (int i = 0; i < 4; ++i) {
      if (xs[i] >= X_LO && xs[i] <= X_UP && ys[i] >= Y_LO && ys[i] <= Y_UP) {
        pts_set.insert(make_pair(xs[i], ys[i]));
      }
    }
    // set<pair<Dtype, Dtype>>::iterator it = pts_set.begin();
    auto it = pts_set.begin();
    x1 = (*it).first;
    y1 = (*it).second;
    ++it;
    x2 = (*it).first;
    y2 = (*it).second;
  }
}

GRANSAC::VPFloat Slope(int x0, int y0, int x1, int y1) {
  return (GRANSAC::VPFloat) (y1 - y0) / (x1 - x0);
}

void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth) {
  GRANSAC::VPFloat slope = Slope(a.x, a.y, b.x, b.y);

  cv::Point p(0, 0), q(img.cols, img.rows);

  p.y = -(a.x - p.x) * slope + a.y;
  q.y = -(b.x - q.x) * slope + b.y;

  cv::line(img, p, q, color, LineWidth, 8, 0);
}


#endif /* INCLUDE_COMMON_FUNCTION_HPP_ */
