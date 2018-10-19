#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

using namespace ifopt;

class ExVariables : public VariableSet {
public:
  // Every variable set has a name, here "var_set1". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  ExVariables() : ExVariables("var_set1", 108) {};
  ExVariables(const std::string& name, const int& size) : VariableSet(size, name)
  {

  }
  ExVariables(const std::string& name, const Eigen::VectorXd& init_vec) : VariableSet(init_vec.size(), name)
  {
    x_ = init_vec;
  }

  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    x_ = x;
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  Eigen::VectorXd GetValues() const override
  {
    return x_;
  };

  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    for (int i=0; i<GetRows(); i++)
        bounds.at(i) = NoBound;
 
    return bounds;
  }

private:
  Eigen::VectorXd x_;
  VecBound bounds_;
};