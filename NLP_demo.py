import foot
from numpy import *
import pinocchio as se3
import os
import eigenpy



set_printoptions(linewidth=100, suppress=True, threshold=nan)

display = True
plot = True

ProblemConfig = foot.Config();
filename = str(os.path.dirname(os.path.abspath(__file__)))
ProblemConfig.setFromConfigFile(filename + '/config/singleObstacle.yml')

#Create NLP_framework
myProb = foot.BoxesHullTrajProble(ProblemConfig)
xInit = myProb.findInitPoint()
#print ("Initial Start Position", xInit)

#Define the Problem
import pyipopt
from foot.solvers import *

## for variable
nvar = len(xInit.T)
NLPvariables = variable(nvar)
NLPcosts = cost(ProblemConfig.nBoxes, np.matrix([ProblemConfig.initPos]), nvar);
NLPconstraint = constraint(0, myProb)
NLPconst_set = constraints()
#NLPconst_set.pushback_constraint(NLPconstraint)
for i in range(0, 37):
    NLPconst_set.pushback_constraint(constraint(i, myProb))

x0 = squeeze(asarray(xInit))
print("inital_ x0", x0)

def apply_new(x):
    return True

nlp = pyipopt.create(nvar, NLPvariables.x_L, NLPvariables.x_U, NLPconst_set.ncon, NLPconst_set.getG_L(), NLPconst_set.getG_U(), NLPconst_set.nnzj, 0, NLPcosts.eval_f, NLPcosts.eval_grad_f, NLPconst_set.eval_g, NLPconst_set.eval_jac_g)

nlp.str_option('linear_solver', 'mumps')
#nlp.str_option("jacobian_approximation", "finite-difference-values")
nlp.str_option("jacobian_approximation", "exact")
nlp.str_option("hessian_approximation", "limited-memory")
nlp.int_option('max_iter', 100)
nlp.num_option('tol', 0.001)
print("Going to call solve")
x, zl, zu, constraint_multipliers, obj, status = nlp.solve(x0)

nlp.close()

def print_variable(variable_name, value):
    for i in range(len(value)):
        print("{} {}".format(variable_name + "["+str(i)+"] =", value[i]))

print("Solution of the primal variables, x")
print(x[0:36])

