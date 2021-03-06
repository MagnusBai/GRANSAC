#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <random>

#include "GRANSAC.hpp"
#include "LineModel.hpp"
#include "common_function.hpp"

int main(int argc, char * argv[]) {

  // |-> Getting arguments
  if (argc != 1 && argc != 3) {
    std::cout << "[ USAGE ]: " << argv[0] << " [<Image Size> = 1000] [<nPoints> = 500]"
        << std::endl;
    return -1;
  }

  int Side = 1000;
  int nPoints = 500;
  if (argc == 3) {
    Side = std::atoi(argv[1]);
    nPoints = std::atoi(argv[2]);
  }
  // <-| Getting arguments


  // |-> Generate 2D points waiting for Line-fitting task, amount of which is nPoints
  cv::Mat Canvas(Side, Side, CV_8UC3);
  Canvas.setTo( solarized_palette["base3"] );

  // Randomly generate points in a 2D plane roughly aligned in a line for testing
  std::random_device SeedDevice;
  std::mt19937 RNG = std::mt19937(SeedDevice());

  std::uniform_int_distribution<int> UniDist(0, Side - 1); // [Incl, Incl]
  int Perturb = 25;
  std::normal_distribution<GRANSAC::VPFloat> PerturbDist(0, Perturb);

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
  for (int i = 0; i < nPoints; ++i) {
    int Diag = UniDist(RNG);
    cv::Point Pt(floor(Diag + PerturbDist(RNG)), floor(Diag + PerturbDist(RNG)));
    cv::circle(Canvas, Pt, floor(Side / 100), solarized_palette["cyan"], -1);

    std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared < Point2D > (Pt.x, Pt.y);
    CandPoints.push_back(CandPt);
  }
  // <-| Generate 2D points


  // |-> Key process
  GRANSAC::RANSAC<Line2DModel, 2> Estimator;
  Estimator.Initialize(20, 100); // Threshold, iterations

  int start = cv::getTickCount();
  Estimator.Estimate(CandPoints);
  int end = cv::getTickCount();


  std::cout << "RANSAC took: "
      << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms."
      << std::endl;
  // <-| Key process

  // |-> Show inliers
  auto BestInliers = Estimator.GetBestInliers();
  if (BestInliers.size() > 0) {
    for (auto& Inlier : BestInliers) {
      auto RPt = std::dynamic_pointer_cast < Point2D > (Inlier);
      cv::Point Pt(floor(RPt->m_Point2D[0]), floor(RPt->m_Point2D[1]));
      cv::circle(Canvas, Pt, floor(Side / 100), solarized_palette["violet"], -1);
    }
  }
  // <-| Show inliers

  // |-> extract best model from Inliers
  auto BestLine = Estimator.GetBestModel();
  if (BestLine) {
    auto BestLinePt1 = std::dynamic_pointer_cast < Point2D > (BestLine->GetModelParams()[0]);
    auto BestLinePt2 = std::dynamic_pointer_cast < Point2D > (BestLine->GetModelParams()[1]);
    if (BestLinePt1 && BestLinePt2) {
      cv::Point Pt1(BestLinePt1->m_Point2D[0], BestLinePt1->m_Point2D[1]);
      cv::Point Pt2(BestLinePt2->m_Point2D[0], BestLinePt2->m_Point2D[1]);
      DrawFullLine(Canvas, Pt1, Pt2, solarized_palette["magenta"], 2);
    }
  }
  // <-| extract best model from Inliers


  cv::imwrite("sample0_LineFitting.png", Canvas);

  return 0;
}
