#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

using namespace ifopt;
using namespace std;
class ExCost: public CostTerm {
public:
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {}

  double GetCost() const override
  {
    Eigen::VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    double out = 0;
    // distance between first mobile box and initial position
    Eigen::Vector3d pos = initPos_;
    Eigen::Vector3d posNext = x.head(3);
    Eigen::Vector3d dist = posNext - pos;
    out += dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
    for (size_t i = 0; i < nBoxes_ - 1; ++i)
    {
      // distance between successive mobile boxes
      pos = x.segment(3*i, 3);
      posNext = x.segment(3*i+3, 3);
      dist = posNext - pos;
      out += dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
    }

    return out;
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Eigen::VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
      Eigen::MatrixXd J(1, x.size());
      J.setZero();
      int boxRepDim = 3;
      Eigen::Vector3d pos = initPos_;
      Eigen::Vector3d posNext = x.head(3);
      Eigen::Vector3d dist = posNext - pos;
      
      J.block(0, (0) * boxRepDim, 1, boxRepDim) += 2 * dist.transpose();
      for (int i = 0; i < static_cast<int>(nBoxes_) - 1; ++i)
      {
        pos = x.segment(3*i, 3);
        posNext = x.segment(3*i+3, 3);
        dist = posNext - pos;
        J.block(0, i * boxRepDim, 1, boxRepDim) += -2 * dist.transpose();
        J.block(0, (i + 1) * boxRepDim, 1, boxRepDim) += 2 * dist.transpose();
      }

      for (int i=0; i < nBoxes_*3; i++)
        jac.coeffRef(0, i) = J(0, i);
    }
  }
  void SetBoxNum(const int& nbox){
    nBoxes_ = nbox;
  }
  void SetInitPose(const Eigen::Vector3d& init){
    initPos_ = init;
  }

private:
  int nBoxes_;
  Eigen::Vector3d initPos_;
};
