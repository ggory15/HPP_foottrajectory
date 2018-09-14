#include <iostream>
#include <hpp/foot/utils/ProblemConfig.hh>
#include <hpp/foot/TrajectoryProblem.hh>
#include <hpp/foot/solvers/LPQPsolver.hh>
#include <hpp/foot/utils/Printer.hh>

using namespace std;
using namespace hpp::foot;

LPQPSolver * LPQP_;

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
    TrajectoryProblem myProb(ymlPath);
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

    std::cout << "myProb: " << myProb << std::endl;

    // this part is for solving qp and lq.
    LPQP_ = new LPQPSolver(myProb, myProb.maxIter());
    myProb.normalizeNormals(initVec);

    LPQP_->init(initVec);
    LPQP_->solve();  

    LPQP_->logAllX("/home/skim/foot_data/");

    // // Display the results
    // printAllIterations(myProb.config()["logName"], myProb, LPQP_->res(),
    //                     "/home/skim/foot_data/");
    // std::cout << "log is saved" << std::endl;
    // std::string command = "$DEVEL_HPP_DIR/src/foot_trajectory/scripts/animSteps.py logs/";
    // if (argc > 1)
    // {
    // command += argv[1];
    // }
    // else
    // {
    // command += "singleObstacle";
    // }
    // command += ".log ";
    // if (myProb.config().has("plotPlanes"))
    // command += myProb.config()["plotPlanes"];
    // system(command.c_str());

    return 0;
}