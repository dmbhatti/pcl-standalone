cmake_minimum_required(VERSION 2.8)
project(pcl-standalone)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(Eigen3 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${QT_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${QT_LIBRARY_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

file(GLOB headers       "*.h*")
file(GLOB sources       "*.cpp")
file(GLOB headers_algos "algos/*.h")
file(GLOB sources_algos "algos/*.cpp")
file(GLOB headers_ade   "algos/ADEVision/*.h")
file(GLOB sources_ade   "algos/ADEVision/*.cpp")
file(GLOB ui_files      "*.ui")

set(project_headers ${headers} ${headers_algos} ${headers_ade})
set(project_sources ${sources} ${sources_algos} ${sources_ade})
set(project_forms   ${ui_files})
set(VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

qt4_wrap_cpp(project_headers_moc    ${project_headers})
qt4_wrap_ui(project_forms_headers   ${project_forms})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

add_executable(pcl-standalone ${project_sources} ${project_forms_headers} ${project_headers_moc})
target_link_libraries(pcl-standalone ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS})

