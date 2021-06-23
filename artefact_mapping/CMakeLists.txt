cmake_minimum_required (VERSION 2.8)
project(abb_odn)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

#############
# LIBRARIES #
#############
set(SOURCES
  src/object-tracking.cc
  src/object-tracking-pipeline.cc
  src/object-tracker.cc
  src/tracker-utils.cc
)

cs_add_executable(${PROJECT_NAME} ${SOURCES})
#target_link_libraries(${PROJECT_NAME} dark::dark)

##########
# EXPORT #
##########
cs_install()
cs_export()
