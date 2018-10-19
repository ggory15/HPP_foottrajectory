#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <hpp/foot/BoxesHullTrajProblem.hh>

using namespace hpp::foot;
using namespace ifopt;

class ExConstraint : public ConstraintSet {
public:
  ExConstraint(const int& size) : ExConstraint("constraint1", size) {

  }

  // This constraint set just contains 1 constraint, however generally
  // each set can contain multiple related constraints.
  ExConstraint(const std::string& name, const int& size) : ConstraintSet(size, name) {}
  ExConstraint(const int& index, const int& size, BoxesHullTrajProblem *a) : ConstraintSet(size, std::to_string(index)) {
    index_ = index;
    myProb_ = a;
  }
  
  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    myProb_->evalNonLinCstr(g, index_, x);

    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    VectorXd lb(GetRows()), ub(GetRows());
    myProb_->getNonLinCstrLB(lb, index_);
    myProb_->getNonLinCstrUB(ub, index_);
    
    for (int i=0; i< GetRows(); i++)
      b.at(i) = Bounds(lb(i), ub(i));
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "var_set1". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.
    if (var_set == "var_set1") {
      VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
      MatrixXd out(GetRows(), x.size());
      out.setZero();
      myProb_->evalNonLinCstrDiff(out, index_, x);

      for (int i=0; i<out.rows(); i++)
        for (int j=0; j<out.cols(); j++)
          jac_block.coeffRef(i, j) = out(i, j);
    }
  }
  void GetProblem(BoxesHullTrajProblem *a){
    myProb_ = a;
  }
private:
  BoxesHullTrajProblem* myProb_;
  int index_;
};