#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <random>

#include "GRANSAC.hpp"
#include "VanPtModel.hpp"
#include "common_function.hpp"

#include <opencv2/opencv.hpp>

int main(int argc, char * argv[]) {

  // |-> Generate 2D points waiting for Line-fitting task, amount of which is nPoints
  // data were kept in std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
  const int canvas_width = 960;
  const int canvas_height = 1280;
  cv::Mat canvas(canvas_height, canvas_width, CV_8UC3);

  { // setting canvas's bg
    canvas.setTo( solarized_palette["base3"] );
  } // setting canvas's bg

  std::vector<float> pts_array;
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> cand_lines;
  { // loading samples
    const char * lines_list_path = "../dependency/sample2_data/lines_data.csv";
    CvMLData mlData1;
    mlData1.read_csv(lines_list_path);
    const CvMat* tmp1 = mlData1.get_values();
    cv::Mat lines_mat_32f(tmp1, true);
    tmp1->CvMat::~CvMat();

    for(int i=0; i<lines_mat_32f.rows; ++i) {
      int x1 = lines_mat_32f.at<float>(i, 0);
      int y1 = lines_mat_32f.at<float>(i, 1);
      int x2 = lines_mat_32f.at<float>(i, 2);
      int y2 = lines_mat_32f.at<float>(i, 3);
      GRANSAC::VPFloat a, b, c;

      // plot on canvas
      cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2), solarized_palette["yellow"], 2);
      cvt2ptsAbc((GRANSAC::VPFloat)x1, (GRANSAC::VPFloat)y1, (GRANSAC::VPFloat)x2, (GRANSAC::VPFloat)y2, a, b, c);


      std::shared_ptr<GRANSAC::AbstractParameter> cand_line = std::make_shared < Line2D > (a, b, c);
      // insert into data items vector
      cand_lines.push_back(cand_line);
    }
  } // loading samples

  // key function
  GRANSAC::RANSAC<VanPtModel, 2> estimator;

  estimator.Initialize(10, cand_lines.size()); // Threshold, iterations

  int start = cv::getTickCount();
  estimator.Estimate(cand_lines);
  int end = cv::getTickCount();


  std::cout << "RANSAC took: "
            << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms."
            << std::endl;

  // params about best model
  bool found_model {false};
  { // extract best models
    auto best_line = estimator.GetBestModel();
    auto best_model_line1 = std::dynamic_pointer_cast < Line2D > (best_line->GetModelParams()[0]);
    auto best_model_line2 = std::dynamic_pointer_cast < Line2D > (best_line->GetModelParams()[1]);
    if( best_model_line1 && best_model_line2 ) {
      found_model = true;
      std::cout << "found!" << std::endl;

      std::cout << "best_line_1: " << (*best_model_line1) << std::endl;
      std::cout << "best_line_2: " << (*best_model_line2) << std::endl;
      // DrawFullLine(canvas, Pt1, Pt2, solarized_palette["yellow"], 6);
    }
  } // extract best models

  { // plot inliers
    auto best_inliers = estimator.GetBestInliers();      // key function
    cv::Scalar& color = solarized_palette["magenta"];
    for( auto& inlier: best_inliers ) {
      auto il = std::dynamic_pointer_cast < Line2D > (inlier);
      std::cout << *il << std::endl;
      GRANSAC::VPFloat a {il->m_line2d[0]};
      GRANSAC::VPFloat b {il->m_line2d[1]};
      GRANSAC::VPFloat c {il->m_line2d[2]};
      GRANSAC::VPFloat x1, y1, x2, y2;
      ABC_2points(a, b, c, x1, y1, x2, y2);
      cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2),
                    solarized_palette["magenta"], 1);
    }
  } // plot inliers


  cv::imwrite("sample2_vanishing_point.png", canvas);

  return 0;
}
