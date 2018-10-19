#include <iostream>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <hpp/foot/utils/Printer.hh>
#include <hpp/foot/solvers/NLPconstraint.hh>
#include <hpp/foot/solvers/NLPvariable.hh>
#include <hpp/foot/solvers/NLPcost.hh>
#include <hpp/foot/BoxesHullTrajProblem.hh>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

using namespace std;
using namespace hpp::foot;
using namespace ifopt;

int main(int argc, char* argv[])
{
    std::string ymlPath;
  if (argc > 1)
  {
    ymlPath = std::string(CONFIGS_DATA_DIR) + "/" + argv[1] + ".yml";
  }
  else
  {
    std::cout << "Loading default file \"singleObstacle.yml\"" << std::endl;
    ymlPath = std::string(CONFIGS_DATA_DIR) + "/singleObstacle.yml";
  }
  ProblemConfig config(ymlPath);

  int nBoxes = config["nBoxes"]; // 12
  int nObstacles = 0;
  if (config.has("obstacles")) // 1
    nObstacles = static_cast<int>(config["obstacles"].asVecBox().size());
  
  int MBox = 3;
  int MPlane = 6;
  int manifold_size = MBox * nBoxes + MPlane * nObstacles * nBoxes;  

  BoxesHullTrajProblem myProb(ymlPath);

  Eigen::VectorXd initVec(myProb.dimVar());
  if (myProb.config().has("x0"))
  {
    initVec << myProb.config()["x0"].asVectorXd();
    std::cout << "myProb.config()[x0].asVectorXd(): "
              << myProb.config()["x0"].asVectorXd().transpose() << std::endl;
  }
  else
    initVec = myProb.findInitPoint();

  if (myProb.config().has("initialGuessRandomFactor"))
  {
    initVec = initVec +
              myProb.config()["initialGuessRandomFactor"] *
                  Eigen::VectorXd::Random(myProb.dimVar());
  }

  // 1. define the problem
  cout << "Start Define NLP" << endl; 
  Problem nlp;
  cout << "init" << initVec.tail(72).transpose() << endl;
  std::shared_ptr<ExVariables> variable(new ExVariables("var_set1", initVec));
  std::shared_ptr<ExCost> cost(new ExCost());
  cost->SetBoxNum(nBoxes);
  cost->SetInitPose(config["initPos"].asVector3d());
  
  std::vector<std::shared_ptr<ExConstraint>> constraint_set;
  for (int i=0; i<36; i++)
    constraint_set.push_back(std::make_shared<ExConstraint>(i, myProb.nonLinCstrDim(i), &myProb));
  
  nlp.AddVariableSet  (variable);
  for (auto c : constraint_set)
    nlp.AddConstraintSet(c);
  nlp.AddCostSet      (cost);
  nlp.PrintCurrent();

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("max_iter", 100);
  //ipopt.SetOption("print_level", 5);

  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.head(36).transpose() << std::endl;


  return 0;
}