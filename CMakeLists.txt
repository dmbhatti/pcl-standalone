cmake_minimum_required (VERSION 2.8)

project      (pclApp)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

file(GLOB_RECURSE headers_FILES "*.h")
file(GLOB_RECURSE sources_FILES "*.cpp")
file(GLOB_RECURSE ui_FILES "*.ui")

set  (project_SOURCES ${sources_FILES})
set  (project_HEADERS ${headers_FILES})
set  (project_FORMS   ${ui_FILES})
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") Cannot use C11 with PCL

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pclApp ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pclApp ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

