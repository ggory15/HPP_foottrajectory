CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")

SET(PROJECT_NAME hpp-foot)
SET(PROJECT_DESCRIPTION
  "Foor Trajectory Generation by Using Hyper Plane"
  )
SET(PROJECT_URL "")

SETUP_PROJECT()
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0")
ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML-CPP_INCLUDE_DIRS})

SET(QPOASES_PREFIX ${CMAKE_PREFIX_PATH} CACHE PATH "Set the prefix where qpOases is installed")
FIND_PATH(QPOASES_INCLUDE_DIRS NAMES "qpOASES.hpp" PATHS ${QPOASES_PREFIX} PATH_SUFFIXES include)
FIND_LIBRARY(QPOASES_LIBRARY "qpOASES" PATHS ${QPOASES_PREFIX} PATH_SUFFIXES lib)
INCLUDE_DIRECTORIES(${QPOASES_INCLUDE_DIRS})

ADD_REQUIRED_DEPENDENCY("ifopt")

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
SET(${PROJECT_NAME}_HEADERS
	include/hpp/foot/utils/Box.hh
  include/hpp/foot/utils/FixedPlan.hh
  include/hpp/foot/utils/PlanForHull.hh
  include/hpp/foot/utils/ProblemConfig.hh
  include/hpp/foot/utils/defs.hh
  include/hpp/foot/utils/Printer.hh
  #
  include/hpp/foot/functions/BoxAbovePlan.hh
  include/hpp/foot/functions/BoxAboveFixedPlan.hh
  include/hpp/foot/functions/CostDistance.hh
  include/hpp/foot/functions/FixedBoxPosition.hh
  include/hpp/foot/functions/PlanBetweenBoxAndObstacle.hh
  #
  include/hpp/foot/solvers/LPQPsolver.hh
  include/hpp/foot/solvers/QP.hh
  include/hpp/foot/solvers/QPPlanesFixed.hh
  include/hpp/foot/solvers/QPBoxesFixed.hh
  include/hpp/foot/solvers/QPBoxesFixedIndividual.hh

  #
  include/hpp/foot/TrajectoryProblem.hh
  include/hpp/foot/BoxesHullTrajProblem.hh
)


add_subdirectory (src)
add_subdirectory (demo)

PKG_CONFIG_APPEND_LIBS("hpp-foot")

SETUP_PROJECT_FINALIZE()
