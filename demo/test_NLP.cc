#include <iostream>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <hpp/foot/utils/Printer.hh>

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
  ProblemConfig config(ymlPath);

  int nBoxes = config["nBoxes"]; // 12
  int nObstacles = 0;
  if (config.has("obstacles")) // 1
    nObstacles = static_cast<int>(config["obstacles"].asVecBox().size());

    return 0;
}