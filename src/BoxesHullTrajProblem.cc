#include <iostream>
#include <fstream>
#include <limits>

#include <Eigen/Geometry>

#include <hpp/foot/BoxesHullTrajProblem.hh>
#include <hpp/foot/functions/BoxAbovePlan.hh>

using namespace Eigen;
using namespace std;
namespace hpp{
namespace foot
{
BoxesHullTrajProblem::BoxesHullTrajProblem(const std::string& configPath)
    : config_(configPath),
      initPos_(config_["initPos"].asVector3d()),
      finalPos_(config_["finalPos"].asVector3d()),
      boxSize_(config_["BoxSize"].asVector3d()),
      nBoxes_(config_["nBoxes"].asSize_t()),
      initBox_(-1, boxSize_, initPos_, true),
      initBoxAbovePlanFct_(initBox_),
      fixedFinalBox_(finalPos_)
{
  if (config_.has("securityDistance"))
  {
    securityDistance_ = config_["securityDistance"].asDouble();
  }
  else
    securityDistance_ = 0;

  if (config_.has("maxStepHeight"))
  {
    maxStepHeight_ = config_["maxStepHeight"].asDouble();
  }
  else
    maxStepHeight_ = 0;

  if (config_.has("obstacles"))
  {
    obstacles_ = config_["obstacles"].asVecBox();
    for (size_t i = 0; i < obstacles_.size(); i++)
    {
      obstacles_[i].setFixed(true);
    }
  }

  if (config_.has("fixedPlanes"))
    fixedPlanes_ = config_["fixedPlanes"].asVecFixedPlan();

  nObstacles_ = obstacles_.size();
  nFixedPlanes_ = fixedPlanes_.size();
  nPlans_ = nBoxes_ * nObstacles_;

  nMobilePlanCstr_ = nObstacles_ * nBoxes_;
  nFixedPlanCstr_ = nFixedPlanes_ * nBoxes_;

  // threshold_ = -boxSize_.minCoeff() / 2;

  for (int i = 0; i < static_cast<int>(nBoxes_); ++i)
  {
    Box b(i, boxSize_);
    boxes_.push_back(b);
    boxAbovePlanFcts_.push_back(BoxAbovePlan(b));
  }
  fixedFinalBox_ = FixedBoxPosition(finalPos_);

  for (auto p : fixedPlanes_)
  {
    for (auto b : boxes_)
    {
      boxAboveFixedPlanFcts_.push_back(BoxAboveFixedPlan(b, p));
    }
  }

  for (auto o : obstacles_)
  {
    obstacleAbovePlanFcts_.push_back(BoxAbovePlan(o));
  }

  for (int i = -1; i < static_cast<int>(nBoxes_) - 1; ++i)
  {
    for (int j = 0; j < static_cast<int>(nObstacles_); ++j)
    {
      plans_.push_back(PlanForHull(j, i, i + 1));
    }
  }
  for (auto c : boxAboveFixedPlanFcts_)
  {
    Eigen::IOFormat CleanFmt(2, 0, ", ", "", "[", "]");
    std::stringstream ss;
    ss << "Box " << std::to_string(c.box().index())
       << " above fixed plan: {normal:"
       << c.normal().transpose().format(CleanFmt) << ", d: " << c.d() << "}";
    std::cout << ss.str() << std::endl;
    cstrNames_.push_back(ss.str());
  }

  for (size_t i = 0; i < static_cast<size_t>(nPlans_); i++)
  {
    std::string str("Plan " + std::to_string(i) + " between HullBoxes " +
                    std::to_string(plans_[i].box0Above()) + " and " +
                    std::to_string(plans_[i].box1Above()) + " and obstacle " +
                    std::to_string(plans_[i].boxBelow()));
    std::cout << str << std::endl;
    cstrNames_.push_back(str);
  }

  std::stringstream sstm;
  sstm << "FeetTraj" << nBoxes_ << "Boxes";

  manifold_size = 3 * nBoxes_ + 6 * nObstacles_ * nBoxes_; 
  outRepObjDiff_.resize(1, manifold_size); // fixme: skim
  outRepObjDiff_.setZero();
  outRep_.resize(
       static_cast<Index>(24 * nMobilePlanCstr_ + 8 * nFixedPlanCstr_ + 3),
       manifold_size);
  outRep_.setZero();
}

BoxesHullTrajProblem::~BoxesHullTrajProblem() {}

Eigen::VectorXd BoxesHullTrajProblem::findInitPoint()
{
  Eigen::VectorXd xInit(manifold_size);
  xInit.setZero();
  double pi = M_PI;
  // First compute the initial guess for the foot trajectory
  for (size_t i = 0; i < nBoxes_; i++)
  {
    xInit.segment(3 * i, 3) =
        initPos_ + (double)(i + 1) * (finalPos_ - initPos_) / (double)(nBoxes_);
    xInit(3 * i + 2) +=
        maxStepHeight_ * std::sin((double)(i + 1) / (double)(nBoxes_)*pi);
  }

  for (size_t i = 0; i < plans_.size(); i++)
  {
    Eigen::Vector3d box0Above(
        getBoxPositionFromX(plans_[i].box0Above(), xInit));
    Eigen::Vector3d box1Above(
        getBoxPositionFromX(plans_[i].box1Above(), xInit));
    Eigen::Vector3d boxBelow(obstacles_[plans_[i].boxBelow()].center());
    Eigen::Vector3d n, center;
    n = (box1Above + box0Above) / 2.0 - boxBelow;
    center = boxBelow + n / 2.0;
    n.normalize();
    double d = center.dot(n);
    xInit.segment(3 * nBoxes_ + 4 * i + 1, 3) << n;
    xInit(3 * nBoxes_ + 4 * i) = d;
  }

  return xInit;
 
}

Eigen::Vector3d BoxesHullTrajProblem::getBoxPositionFromX(
    size_t i, const Eigen::VectorXd& x) const
{
  if (i == -1)
    return initPos_;
  else
    return x.segment(3 * i, 3);
}

void BoxesHullTrajProblem::getManifoldsize(const Index& nBoxes,const Index& nObstacles){
  assert(nBoxes > 0 && "Number of Boxes must be positive and non null");
  assert(nObstacles >= 0 && "Number of Obstacles must be positive");
  
  Index MBox = 3;
  Index MPlane = 6;

  manifold_size = MBox * nBoxes + MPlane * nObstacles * nBoxes;  
}

Eigen::VectorXd BoxesHullTrajProblem::getTangentLB() const
{
  VectorXd out(manifold_size);
  double infinity = std::numeric_limits<double>::infinity();
  for (int i = 0; i < out.size(); i++) out(i) = -infinity;
}

void BoxesHullTrajProblem::getTangentUB(RefVec out) const
{
  assert(out.size() == manifold_size && "wrong size");
  double infinity = std::numeric_limits<double>::infinity();

  for (int i = 0; i < out.size(); i++) out(i) = infinity;
}

void BoxesHullTrajProblem::evalObj(double& out, const VectorXd& x) const
{
  out = 0;
  //distance between first mobile box and initial position
  Eigen::Vector3d pos = initPos_;
  Eigen::Vector3d posNext = x.head(3);
  Eigen::Vector3d dist = posNext - pos;
  out += dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
  for (size_t i = 0; i < nBoxes_ - 1; ++i)
  {
    // distance between successive mobile boxes
    pos = x.segment(i*3, 3);
    posNext = x.segment(i*3+3, 3);
    dist = posNext - pos;
    out += dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
  }
   
  // TODO:: objective fucntion sum (b_k+1 - b_k)^2
 
}

void BoxesHullTrajProblem::evalLinCstr(RefVec, size_t) const {}

void BoxesHullTrajProblem::evalLinCstrDiff(RefMat, size_t) const {}

void BoxesHullTrajProblem::getLinCstrLB(RefVec, size_t) const {}

void BoxesHullTrajProblem::getLinCstrUB(RefVec, size_t) const {}

void BoxesHullTrajProblem::evalNonLinCstr(RefVec out, size_t i, RefVec x) const
{
  assert(i < numberOfCstr() && "This constraint index is out of bounds");
  assert(out.size() == nonLinCstrDim(i) && "wrong size");
  out.setZero();

  Eigen::Vector4d nullQuat(0, 0, 0, 1);

  if (i < nFixedPlanCstr_)
  {
    size_t iC = static_cast<size_t>(boxAboveFixedPlanFcts_[i].box().index());
    Eigen::Vector3d trans = x.segment(3*iC, 3); //phi_x_z()(0)(iC)[0];
    boxAboveFixedPlanFcts_[i].compute(out, trans, nullQuat);
  }
  else if (i == nFixedPlanCstr_)
  {
    Eigen::Vector3d trans = x.segment(nBoxes_*3-3, 3);
    fixedFinalBox_.compute(out, trans);
  }
  else if (i < numberOfCstr() - nMobilePlanCstr_)
  {
    const size_t iPlan(static_cast<size_t>(i) - nFixedPlanCstr_ - 1);
    const int iBox0Above(plans_[iPlan].box0Above());
    const int iBox1Above(plans_[iPlan].box1Above());
    const size_t iBox0AboveSize_t(static_cast<size_t>(iBox0Above));
    const size_t iBox1AboveSize_t(static_cast<size_t>(iBox1Above));
    const size_t iBoxBelow(static_cast<size_t>(plans_[iPlan].boxBelow()));
    Eigen::Vector3d transBelow = obstacles_[iBoxBelow].center();
    Eigen::Vector3d normal = x.segment(3*nBoxes_ + 4 * (i - nBoxes_ -1) + 1, 3); //phi_x_z()(1)(iPlan)[1];
    double d =  x(3*nBoxes_ + 4 * (i - nBoxes_ -1));


    Eigen::Vector3d trans0Above, trans1Above;

    const BoxAbovePlan* box0AbovePlanFct(nullptr);
    const BoxAbovePlan* box1AbovePlanFct(nullptr);

    if (iBox0Above != -1 && iBox1AboveSize_t != nBoxes_)
    {
      trans0Above = x.segment(3*iBox0AboveSize_t, 3); // phi_x_z()(0)(iBox0AboveSize_t)[0];
      trans1Above = x.segment(3*iBox1AboveSize_t, 3);
      box0AbovePlanFct = &boxAbovePlanFcts_[iBox0AboveSize_t];
      box1AbovePlanFct = &boxAbovePlanFcts_[iBox1AboveSize_t];
    }
    else if (iBox0Above == -1 && iBox1AboveSize_t != nBoxes_)
    {
      trans0Above = initPos_;
      trans1Above = x.segment(3*iBox1AboveSize_t, 3);
      box0AbovePlanFct = &initBoxAbovePlanFct_;
      box1AbovePlanFct = &boxAbovePlanFcts_[iBox1AboveSize_t];
    }
    else
    {
      std::cerr << "Box0 is initial pos and Box1 is final, there is no "
                   "intermediary boxes..." << std::endl;
    }

    box0AbovePlanFct->compute(out.head(8), trans0Above, nullQuat, d, normal);
    box1AbovePlanFct->compute(out.segment(8, 8), trans1Above, nullQuat, d,normal);
    obstacleAbovePlanFcts_[iBoxBelow].compute(out.tail(8), transBelow, nullQuat, -d, -normal);
  }
  else if (i < numberOfCstr())
  {
    Eigen::Vector3d normal = x.segment(3*nBoxes_ + 4 * (i - nBoxes_ * 2 -1) + 1, 3); //phi_x_z()(1)(iPlan)[1];
    out(0) = pow(normal.norm(), 2);
  }
}
void BoxesHullTrajProblem::evalNonLinCstrDiff(RefMat out, size_t i, RefVec x) const
{
  assert(i < numberOfCstr() && "This constraint index is out of bounds");
  assert(out.rows() == nonLinCstrDim(i) && "wrong row size");
  assert(out.cols() == x.size() && "Wrong cols size");

  outRep_.setZero();
  Eigen::Vector4d nullQuat(0, 0, 0, 1);

  if (i < nFixedPlanCstr_)
  {
    int iC = boxAboveFixedPlanFcts_[i].box().index();
    Eigen::Vector3d trans = x.segment(iC*3, 3);
    boxAboveFixedPlanFcts_[i].diffTrans(outRep_.block(8 * static_cast<long>(i), 3 * iC, 8, 3), trans, nullQuat);
    out = outRep_.middleRows(8 * static_cast<long>(i), 8);
  }
  else if (i == nFixedPlanCstr_)
  {
    fixedFinalBox_.diff(outRep_.block(8 * static_cast<long>(i), 3 * (nBoxes_ - 1), 3, 3));
    out = outRep_.middleRows(8 * static_cast<long>(i), 3);
  }
  else if (i < numberOfCstr() - nMobilePlanCstr_)
  {
    const size_t iPlan(static_cast<size_t>(i) - nFixedPlanCstr_ - 1);
    if(iPlan >= plans_.size())
      throw std::out_of_range("plan index");
    const int iBox0Above(plans_[iPlan].box0Above());
    const int iBox1Above(plans_[iPlan].box1Above());
    const size_t iBox0AboveSize_t(static_cast<size_t>(iBox0Above));
    const size_t iBox1AboveSize_t(static_cast<size_t>(iBox1Above));
    const size_t iBoxBelow(static_cast<size_t>(plans_[iPlan].boxBelow()));

    Eigen::Vector3d transBelow = obstacles_[iBoxBelow].center();
    Eigen::Vector3d normal = x.segment(3*nBoxes_ + 4 * (i - nBoxes_ -1) + 1, 3); //phi_x_z()(1)(iPlan)[1];
    double d =  x(3*nBoxes_ + 4 * (i - nBoxes_ -1));

    long rowBeginLong = static_cast<long>(16 * iPlan);
    long box0AboveBeginLong = static_cast<long>(3 * iBox0Above);
    long box1AboveBeginLong = static_cast<long>(3 * iBox1Above);
    long planBeginLong = static_cast<long>(3 * nBoxes_ + 4 * iPlan);

    Eigen::Matrix<double, 8, 1> tmpWTF;

    Eigen::Vector3d trans0Above, trans1Above;
    const BoxAbovePlan* box0AbovePlanFct = &boxAbovePlanFcts_[iBox0AboveSize_t];
    const BoxAbovePlan* box1AbovePlanFct = &boxAbovePlanFcts_[iBox1AboveSize_t];
    if (iBox0Above != -1 && iBox1AboveSize_t != nBoxes_)
    {
      trans0Above = x.segment(3*iBox0AboveSize_t, 3); // phi_x_z()(0)(iBox0AboveSize_t)[0];
      trans1Above = x.segment(3*iBox1AboveSize_t, 3); //phi_x_z()(0)(iBox1AboveSize_t)[0];
      box0AbovePlanFct->diffTrans(
          outRep_.block(rowBeginLong, box0AboveBeginLong, 8, 3), trans0Above,
          nullQuat, d, normal);
      out.block(0, box0AboveBeginLong, 8, 3) = outRep_.block(rowBeginLong, box0AboveBeginLong, 8, 3);    
      box1AbovePlanFct->diffTrans(
          outRep_.block(rowBeginLong + 8, box1AboveBeginLong, 8, 3),
          trans1Above, nullQuat, d, normal);
      out.block(8, box1AboveBeginLong, 8, 3) = outRep_.block(rowBeginLong + 8, box1AboveBeginLong, 8, 3);    
    }
    else if (iBox0Above == -1 && iBox1AboveSize_t != nBoxes_)
    {
      trans0Above = initPos_;
      trans1Above = x.segment( 3*iBox1AboveSize_t, 3); //phi_x_z()(0)(iBox1AboveSize_t)[0];
      box0AbovePlanFct = &initBoxAbovePlanFct_;
      box1AbovePlanFct->diffTrans(
          outRep_.block(rowBeginLong + 8, box1AboveBeginLong, 8, 3),
          trans1Above, nullQuat, d, normal);
      out.block(8, box1AboveBeginLong, 8, 3) = outRep_.block(rowBeginLong + 8, box1AboveBeginLong, 8, 3);    
    }
    // else if (iBox0Above != -1 && iBox1AboveSize_t == nBoxes_)
    //{
    // trans0Above = phi_x_z()(0)(iBox0AboveSize_t)[0];
    // trans1Above = finalPos_;
    // box1AbovePlanFct = &finalBoxAbovePlanFct_;
    // box0AbovePlanFct->diffTrans(
    // outRep_.block(rowBeginLong, box0AboveBeginLong, 8, 3), trans0Above,
    // nullQuat, d, normal);
    //}
    else
    {
      std::cerr << "Box0 is initial pos and Box1 is final, there is no "
                   "intermediary boxes..." << std::endl;
    }

    // Box0
    box0AbovePlanFct->diffD(tmpWTF, trans0Above, nullQuat, d, normal);
    outRep_.block(rowBeginLong, planBeginLong, 8, 1) = tmpWTF;
    out.block(0, planBeginLong, 8, 1) = outRep_.block(rowBeginLong, planBeginLong, 8, 1) = tmpWTF;  
    box0AbovePlanFct->diffNormal(
        outRep_.block(rowBeginLong, planBeginLong + 1, 8, 3), trans0Above,
        nullQuat, d, normal);
    out.block(0, planBeginLong + 1, 8, 3)  = outRep_.block(rowBeginLong, planBeginLong + 1, 8, 3);
    // Box1
    box1AbovePlanFct->diffD(tmpWTF, trans1Above, nullQuat, d, normal);
    outRep_.block(rowBeginLong + 8, planBeginLong, 8, 1) = tmpWTF;
    out.block(8, planBeginLong, 8, 1) = outRep_.block(rowBeginLong + 8, planBeginLong, 8, 1);
    box1AbovePlanFct->diffNormal(
        outRep_.block(rowBeginLong + 8, planBeginLong + 1, 8, 3), trans1Above,
        nullQuat, d, normal);
    out.block(8, planBeginLong + 1, 8, 3) = outRep_.block(rowBeginLong + 8, planBeginLong + 1, 8, 3);
    // Obstacle
    obstacleAbovePlanFcts_[iBoxBelow].diffD(tmpWTF, transBelow, nullQuat, -d,
                                            -normal);
    outRep_.block(rowBeginLong + 16, planBeginLong, 8, 1) = -tmpWTF;
    out.block(16, planBeginLong, 8, 1) = outRep_.block(rowBeginLong + 16, planBeginLong, 8, 1);
    Eigen::Matrix<double, 8, 3> tmpDiffNormal;
    obstacleAbovePlanFcts_[iBoxBelow].diffNormal(tmpDiffNormal, transBelow,
                                                 nullQuat, -d, -normal);
    outRep_.block(rowBeginLong + 16, planBeginLong + 1, 8, 3) = -tmpDiffNormal;
    out.block(16, planBeginLong + 1, 8, 3) = outRep_.block(rowBeginLong + 16, planBeginLong + 1, 8, 3);
  }
  else if (i < numberOfCstr()){
    Eigen::Vector3d normal = x.segment(3*nBoxes_ + 4 * (i - nBoxes_ * 2 -1) + 1, 3);
    for (int j=0; j<3; j++)
      out(0, j + 3*nBoxes_ + 4 * (i - nBoxes_ * 2 -1) + 1) = 2.0 * normal(j);
  }
}
void BoxesHullTrajProblem::getNonLinCstrLB(RefVec out, size_t i) const
{
  assert(i < numberOfCstr() && "This constraint index is out of bounds");
  assert(out.size() == nonLinCstrDim(i) && "wrong size");
  if (i < nFixedPlanCstr_)
  {
    boxAboveFixedPlanFcts_[i].LB(out);
  }
  else if (i == nFixedPlanCstr_)
  {
    fixedFinalBox_.LB(out);
  }
  else if (i < numberOfCstr() - nMobilePlanCstr_)
  {
    const size_t iPlan(static_cast<size_t>(i) - nFixedPlanCstr_);
    const int iBox0Above(plans_[iPlan].box0Above());
    const int iBox1Above(plans_[iPlan].box1Above());
    const size_t iBox0AboveSize_t(static_cast<size_t>(iBox0Above));
    const size_t iBox1AboveSize_t(static_cast<size_t>(iBox1Above));
    const size_t iBoxBelow(static_cast<size_t>(plans_[iPlan].boxBelow()));

    boxAbovePlanFcts_[iBox0AboveSize_t].LB(out.head(8), securityDistance_);
    boxAbovePlanFcts_[iBox1AboveSize_t].LB(out.segment(8, 8), securityDistance_);
    boxAbovePlanFcts_[iBoxBelow].LB(out.tail(8), securityDistance_);
  }
  else if (i < numberOfCstr())
  {
    out(0) = 1.0;
  }
}
void BoxesHullTrajProblem::getNonLinCstrUB(RefVec out, size_t i) const
{
  assert(i < numberOfCstr() && "This constraint index is out of bounds");
  assert(out.size() == nonLinCstrDim(i) && "wrong size");
  if (i < nFixedPlanCstr_)
  {
    boxAboveFixedPlanFcts_[i].UB(out);
  }
  else if (i == nFixedPlanCstr_)
  {
    fixedFinalBox_.UB(out);
  }
  else if (i < numberOfCstr() - nMobilePlanCstr_)
  {
    const size_t iPlan(static_cast<size_t>(i) - nFixedPlanCstr_);
    const int iBox0Above(plans_[iPlan].box0Above());
    const int iBox1Above(plans_[iPlan].box1Above());
    const size_t iBox0AboveSize_t(static_cast<size_t>(iBox0Above));
    const size_t iBox1AboveSize_t(static_cast<size_t>(iBox1Above));
    const size_t iBoxBelow(static_cast<size_t>(plans_[iPlan].boxBelow()));

    boxAbovePlanFcts_[iBox0AboveSize_t].UB(out.head(8));
    boxAbovePlanFcts_[iBox1AboveSize_t].UB(out.segment(8, 8));
    boxAbovePlanFcts_[iBoxBelow].UB(out.tail(8));
  } 
  else if (i < numberOfCstr())
  {
    out(0) = 1.0;
  }
  else
    throw std::out_of_range("constraint index");
}

size_t BoxesHullTrajProblem::numberOfCstr() const
{
  size_t nCstr = nFixedPlanCstr_ + 1 + nMobilePlanCstr_ * 2; //12 + 1 + 12 + 12
 
  return nCstr;
}

Index BoxesHullTrajProblem::linCstrDim(size_t) const { return 0; }

Index BoxesHullTrajProblem::nonLinCstrDim(size_t i) const
{
  if (i < nFixedPlanCstr_)
    return 8;
  else if (i == nFixedPlanCstr_)
    return 3;
  else if (i < numberOfCstr() - nMobilePlanCstr_)
    return 24;
  else if (i < numberOfCstr())
    return 1;
  else
    return 0;
}

std::string BoxesHullTrajProblem::getCstrName(const size_t) const
{
  std::string str("");
  return str;
}

} /* feettrajectory */
}