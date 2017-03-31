// an extensive model for GRANSAC
// the goal of this model is finding best vanishing point among lines
// create by Magnus Bai
// https://github.com/MagnusBai

#pragma once

#include "AbstractModel.hpp"

#include <limits>

typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;
typedef std::array<GRANSAC::VPFloat, 3> Vector3VP;

//class Point2D: public GRANSAC::AbstractParameter {
//public:
//  Point2D(GRANSAC::VPFloat x, GRANSAC::VPFloat y) {
//    m_Point2D[0] = x;
//    m_Point2D[1] = y;
//  }
//  ;
//
//  Vector2VP m_Point2D;
//};
class Line2D: public GRANSAC::AbstractParameter {
public:
  Line2D(GRANSAC::VPFloat a, GRANSAC::VPFloat b, GRANSAC::VPFloat c) {
    m_line2d[0] = a;
    m_line2d[1] = b;
    m_line2d[2] = c;
  }
  ;

  Vector3VP m_line2d;
};

std::ostream& operator<< (std::ostream& stream, const Line2D& line) {
  stream << line.m_line2d[0] << "*x+" << line.m_line2d[1]
         << "*y+" << line.m_line2d[2] << "=0";
  return stream;
}

class VanPtModel: public GRANSAC::AbstractModel<2> {
protected:
  // Parametric form
  GRANSAC::VPFloat m_vanpt_x, m_vanpt_y;

  virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override {
    auto line2d = std::dynamic_pointer_cast < Line2D > (Param);
    if (line2d == nullptr)
      throw std::runtime_error(
          "VanPtModel::ComputeDistanceMeasure() - Passed parameter are not of type Point2D.");

    // return the distance between a point and a line
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // |a*x0+b*y0+c|/sqrt(a^2+b^2)
    GRANSAC::VPFloat a = line2d->m_line2d[0];
    GRANSAC::VPFloat b = line2d->m_line2d[1];
    GRANSAC::VPFloat c = line2d->m_line2d[2];
    return abs(a*m_vanpt_x+b*m_vanpt_y+c)/sqrt(a*a+b*b);
  }
  ;

public:
  VanPtModel(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams) {
    Initialize(InputParams);
  }
  ;

  virtual void Initialize(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams) override {
    using GRANSAC::VPFloat;
    if (InputParams.size() != 2)
      throw std::runtime_error(
          "VanPtModel - Number of input parameters does not match minimum number required for this model.");

    // Check for AbstractParamter types
    auto line1 = std::dynamic_pointer_cast < Line2D > (InputParams[0]);
    auto line2 = std::dynamic_pointer_cast < Line2D > (InputParams[1]);
    VPFloat a1 = line1->m_line2d[0]; VPFloat b1 = line1->m_line2d[1]; VPFloat c1 = line1->m_line2d[2];
    VPFloat a2 = line2->m_line2d[0]; VPFloat b2 = line2->m_line2d[1]; VPFloat c2 = line2->m_line2d[2];
    if (line1 == nullptr || line2 == nullptr)
      throw std::runtime_error("VanPtModel - InputParams type mismatch. It is not a Line2D.");

    std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

    // Compute the vanishing points parameters (intersection point of 2 lines)
    if( a1/b1==a2/b2 ) {    // has no intersection point
      m_vanpt_x = std::numeric_limits<GRANSAC::VPFloat>::infinity();
      m_vanpt_y = std::numeric_limits<GRANSAC::VPFloat>::infinity();
      // so Distance of any line to this point is inf
    }
    else {
      // (a1*b2*x - a2*b1*x) + 0*b1*b2*y + c1*b2 - c2*b1 = 0
      // 0*a1*a2*x + (b1*a2*y - b2*a1*y) + c1*a2 - c2*a1 = 0
      m_vanpt_x = -(c1*b2 - c2*b1)/(a1*b2 - a2*b1);
      m_vanpt_y = -(c1*a2 - c2*a1)/(b1*a2 - b2*a1);
    }
  }
  ;

  virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>>Evaluate(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> EvaluateParams, GRANSAC::VPFloat Threshold)
  {
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
    int nTotalParams = EvaluateParams.size();
    int nInliers = 0;

    for(auto& Param : EvaluateParams)
    {
      if(ComputeDistanceMeasure(Param) < Threshold)
      {
        Inliers.push_back(Param);
        nInliers++;
      }
    }

    GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

    return std::make_pair(InlierFraction, Inliers);
  };
};

