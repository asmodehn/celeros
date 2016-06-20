cmake_minimum_required(VERSION 2.8.3)
project(celeros)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint rospy actionlib_msgs dynamic_reconfigure)
catkin_python_setup()

# make sure configure headers are built before any node using them
#add_dependencies(rostful ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_package()

catkin_add_env_hooks(42.celeros_hostname SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

# Install executable scripts
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS}
               ${${PROJECT_NAME}_PY_SRCS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Unit tests
if (CATKIN_ENABLE_TESTING)
    # Unit tests using nose, but needing a running ROS core.
    find_package(catkin REQUIRED COMPONENTS rostest)
    add_rostest(tests/celeros/testceleros.test)
endif()


##############################################################################
# Installs
##############################################################################

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#TODO : REDIS URL
#catkin_add_env_hooks(42.gopher_hostname SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

# special install since flower only support files as config ( and not modules ) for now...
install(FILES celeros/flowerconfig.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/celeros/
)

# special install since celeros only support files as config ( and not modules ) for now it seems...
install(FILES celeros/config.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/celeros/
)