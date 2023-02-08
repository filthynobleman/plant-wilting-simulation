set(SIMULATION_LINK_LIBRARIES imgui glfw MD5 PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES imgui glfw MD5 PositionBasedDynamics Simulation Utils CopyPBDShaders CopyPBDModels CopyPBDPlants CopyPBDScenes CopyImguiFonts)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  #set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES})
else()
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)
  include_directories( ${OPENGL_INCLUDE_DIRS})

  set(SIMULATION_LINK_LIBRARIES
    ${SIMULATION_LINK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

# Tree water diffusion library
include_directories("${CMAKE_SOURCE_DIR}/extern/pwd/include")
set(libpwd "${CMAKE_SOURCE_DIR}/extern/pwd/lib/pwd.lib")

############################################################
# GenericParameters
############################################################
include_directories(${GenericParameters_INCLUDE_DIR})
if(TARGET Ext_GenericParameters)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)
endif()

add_executable(PlantDemo
	  PlantDemo.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(PlantDemo PROPERTIES FOLDER "Demos")
set_target_properties(PlantDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(PlantDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(PlantDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(PlantDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(PlantDemo ${SIMULATION_LINK_LIBRARIES} ${libpwd})

############################################################
# Discregrid
############################################################
include_directories(${Discregrid_INCLUDE_DIR})
if (TARGET Ext_Discregrid)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
endif()
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${Discregrid_LIBRARIES})

# add_executable(DirectPositionBasedSolverForStiffRodsDemo
# 	  DirectPositionBasedSolverForStiffRodsDemo.cpp
	  
# 	  StiffRodsSceneLoader.h
# 	  StiffRodsSceneLoader.cpp

# 	  ../Common/LogWindow.cpp
# 	  ../Common/LogWindow.h
# 	  ../Common/Simulator_GUI_imgui.cpp
# 	  ../Common/Simulator_GUI_imgui.h
# 	  ../Common/imguiParameters.cpp
# 	  ../Common/imguiParameters.h
# 	  ../Common/DemoBase.cpp
# 	  ../Common/DemoBase.h
	  
# 	  ${VIS_FILES}          
# 	  ${PROJECT_PATH}/Common/Common.h

# 	  CMakeLists.txt
# )

# set_target_properties(DirectPositionBasedSolverForStiffRodsDemo PROPERTIES FOLDER "Demos")
# set_target_properties(DirectPositionBasedSolverForStiffRodsDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
# set_target_properties(DirectPositionBasedSolverForStiffRodsDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
# set_target_properties(DirectPositionBasedSolverForStiffRodsDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
# add_dependencies(DirectPositionBasedSolverForStiffRodsDemo ${SIMULATION_DEPENDENCIES})
# target_link_libraries(DirectPositionBasedSolverForStiffRodsDemo ${SIMULATION_LINK_LIBRARIES})

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

file(COPY "${CMAKE_SOURCE_DIR}/extern/pwd/bin/pwd.dll"
          DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")


VIS_SOURCE_GROUPS()
