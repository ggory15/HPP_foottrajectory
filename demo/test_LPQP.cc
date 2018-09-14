#include <iostream>
#include <hpp/foot/utils/ProblemConfig.hh>
#include <hpp/foot/TrajectoryProblem.hh>
#include <hpp/foot/solvers/LPQPsolver.hh>

using namespace std;
using namespace hpp::foot;

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
    LPQPSolver LPQP(myProb, myProb.maxIter());
    myProb.normalizeNormals(initVec);

    LPQP.init(initVec);
    LPQP.solve();  

    return 0;
}