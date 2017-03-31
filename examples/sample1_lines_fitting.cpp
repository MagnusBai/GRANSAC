#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <random>

#include "GRANSAC.hpp"
#include "LineModel.hpp"
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

  // <-| load 2D points
  using std::vector;
  vector< vector<int> > points_blobs;
  { // loading into points_blobs
    const char * pix_list_path = "../dependency/sample1_data/pixdata.csv";
    const char * segment_list_path = "../dependency/sample1_data/segment.csv";

    CvMLData mlData1;
    mlData1.read_csv(pix_list_path);
    const CvMat* tmp1 = mlData1.get_values();
    cv::Mat pix_mat_32f(tmp1, true);
    tmp1->CvMat::~CvMat();

    CvMLData mlData2;
    mlData2.read_csv(segment_list_path);
    const CvMat* tmp2 = mlData2.get_values();
    cv::Mat seg_mat_32f(tmp2, true);
    tmp2->CvMat::~CvMat();

    points_blobs = vector<vector<int>>( seg_mat_32f.rows );

    std::array<cv::Scalar, 4> colors_array = {solarized_palette["yellow"],
                                              solarized_palette["orange"],
                                              solarized_palette["cyan"],
                                              solarized_palette["green"]};

    for(int i_blob=0; i_blob<seg_mat_32f.rows; ++i_blob) {  // traversal each blob
      int h_head = seg_mat_32f.at<float>(i_blob, 0)/2;
      int h_rear = seg_mat_32f.at<float>(i_blob, 1)/2;

      vector<int>& points_blob = points_blobs[i_blob];
      points_blob.resize(2*(h_rear-h_head));

      int h_blob = 0;
      for(int h=h_head; h<h_rear; ++h, ++h_blob) {      // traversal each pix in blob
        int x = pix_mat_32f.at<float>(h, 0);
        int y = pix_mat_32f.at<float>(h, 1);
        points_blob[h_blob*2+0] = x;
        points_blob[h_blob*2+1] = y;
        // cv::Scalar& color = colors_array[i_blob%4];
        // canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(color[0], color[1], color[2]);
      }
    }
  }  // loading into points_blobs

  { // plotting points on canvas
    vector<int>& points_blob = points_blobs[6];
    cv::Scalar& color = solarized_palette["cyan"];
    for(int i=0; i<points_blob.size()/2; ++i) {
      int x = points_blob[i*2+0];
      int y = points_blob[i*2+1];
      canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(color[0], color[1], color[2]);
    }
  } // plotting points on canvas

  vector<std::shared_ptr<GRANSAC::AbstractParameter>> cand_points;
  { // construct data structure of data items (::AbstractParameter)
    vector<int>& points_blob = points_blobs[6];
    for(int i=0; i<points_blob.size()/2; ++i) {
      int x = points_blob[i*2+0];
      int y = points_blob[i*2+1];

      // one data item
      std::shared_ptr<GRANSAC::AbstractParameter> cand_pt = std::make_shared < Point2D > (x, y);
      cand_points.push_back(cand_pt);
    }
  } // construct data structure of data items (::AbstractParameter)


  // key functions
  GRANSAC::RANSAC<Line2DModel, 2> estimator;

  estimator.Initialize(3, 100); // Threshold, iterations

  int start = cv::getTickCount();
  estimator.Estimate(cand_points);
  int end = cv::getTickCount();


  std::cout << "RANSAC took: "
            << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms."
            << std::endl;


  // params about best model
  bool found_model = false;
  { // extract best models
    auto best_line = estimator.GetBestModel();
    auto best_model_pt1 = std::dynamic_pointer_cast < Point2D > (best_line->GetModelParams()[0]);
    auto best_model_pt2 = std::dynamic_pointer_cast < Point2D > (best_line->GetModelParams()[1]);
    if( best_model_pt1 && best_model_pt2 ) {
      found_model = true;
      cv::Point Pt1(best_model_pt1->m_Point2D[0], best_model_pt1->m_Point2D[1]);
      cv::Point Pt2(best_model_pt2->m_Point2D[0], best_model_pt2->m_Point2D[1]);
      DrawFullLine(canvas, Pt1, Pt2, solarized_palette["yellow"], 6);
    }
  } //

  { // plot inliers
    auto best_inliers = estimator.GetBestInliers();      // key function
    cv::Scalar& color = solarized_palette["magenta"];
    for (auto& Inlier : best_inliers) {
      auto il = std::dynamic_pointer_cast < Point2D > (Inlier);
      int x = il->m_Point2D[0];
      int y = il->m_Point2D[1];
      canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(color[0], color[1], color[2]);
    }
  } // plot inliers

  cv::imwrite("sample1_LineFitting.png", canvas);

  return 0;
}
