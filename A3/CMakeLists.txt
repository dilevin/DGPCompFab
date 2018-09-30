#Notes
#Required dependencies
#OSX/LINUX PKGCONGIG
cmake_minimum_required (VERSION 3.2)

project(CompFabA3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/eigen CACHE PATH "Root include directory for Eigen3" FORCE) 
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/GAUSS/cmake CACHE PATH "Gauss CMake files" FORCE)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake CACHE PATH "Root include directory for Eigen3" FORCE)
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/GAUSS/cmake CACHE PATH "Gauss CMake files" FORCE)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CompFabA3_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CompFabA3_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CompFabA3_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CompFabA3_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CompFabA3_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CompFabA3_BINARY_DIR}/lib )

find_package(Eigen3 REQUIRED)

# libigl
set(LIBIGL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/include CACHE FILEPATH "Root include directory for libigl")
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)
option(GAUSS_USE_UI "Use Gauss' UI" OFF)

find_package(LIBIGL REQUIRED QUIET)

#Third Party Libraries
set(LIBIGL_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/include CACHE FILEPATH "Root include directory for libigl")
set(LIBFIVE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/libfive/libfive/include CACHE FILEPATH "Root include directory for libfive")

#get header and source files
file(GLOB HEADER_CODE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_CODE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${LIBIGL_INCLUDE_PATH})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/glad/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/imgui)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../util/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/../ThirdParty/Gauss ${CMAKE_CURRENT_BINARY_DIR}/Gauss)
add_subdirectory(${PROJECT_SOURCE_DIR}/../util ${CMAKE_CURRENT_BINARY_DIR}/util)
include_directories(${Core_SOURCE_DIR}/include)
include_directories(${Base_SOURCE_DIR}/include)
include_directories(${FEM_SOURCE_DIR}/include)
include_directories(${Gauss_EXT_INCLUDE_DIRS})

if(APPLE)
   add_definitions(-DLIBFIVE_DYLIB=${CompFabA3_BINARY_DIR}/lib/libfive.dylib)
#    add_definitions(-DUTILS_DYLIB=${CompFabA3_BINARY_DIR}/lib/libutils.dylib)
else()
    add_definitions(-DLIBFIVE_DYLIB=${CompFabA3_BINARY_DIR}/lib/libfive.so)
#    add_definitions(-DUTILS_DYLIB=${CompFabA3_BINARY_DIR}/lib/libutils.so)
endif(APPLE)

add_executable(CompFabA3 ${SRC_CODE} ${HEADER_CODE})
target_link_libraries(CompFabA3 igl::core igl_tetgen igl::opengl_glfw igl::opengl_glfw_imgui utils Core Base FEM ParticleSystem ${GAUSS_LIBS}) 
target_compile_options(CompFabA3 PUBLIC -DLLVMDIR="${LLVM_INSTALL_PREFIX}" -I${LLVM_INSTALL_PREFIX}/include)

set_target_properties(CompFabA3 PROPERTIES ENABLE_EXPORTS 1)

if(MSVC)
  set_target_properties(CompFabA3 PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 1)
  set_property(TARGET CompFabA3 APPEND_STRING PROPERTY LINK_FLAGS
              "/EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00D_K@Z
               /EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00DM@Z
               /EXPORT:cling_runtime_internal_throwIfInvalidPointer")
endif()



