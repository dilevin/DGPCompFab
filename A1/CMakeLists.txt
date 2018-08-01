#Notes
#Required dependencies
#OSX/LINUX PKGCONGIG
cmake_minimum_required (VERSION 3.2)

project(CompFabA1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/eigen CACHE PATH "Root include directory for Eigen3" FORCE) 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake CACHE PATH "Root include directory for Eigen3" FORCE)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CompFabA1_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CompFabA1_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CompFabA1_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CompFabA1_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CompFabA1_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CompFabA1_BINARY_DIR}/lib )


#cling
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/src/build/install/lib/cmake/cling CACHE PATH "Root include directory for Eigen3" FORCE)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/src/build/install/lib/cmake/clang CACHE PATH "Root include directory for Eigen3" FORCE)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/../ThirdParty/src/build/install/lib/cmake/llvm CACHE PATH "Root include directory for Eigen3" FORCE)

find_package(Eigen3 REQUIRED)
find_package(LLVM REQUIRED)
find_package(Clang REQUIRED)

# Keep symbols for JIT resolution
set(LLVM_NO_DEAD_STRIP 1)

#Flags which prevent crashes when compiling against libfive
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -g -fPIC -pedantic -Werror=switch -march=native")
if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++")
endif()
    set(CMAKE_CXX_FLAGS_DEBUG "-O0")
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Output/Debug)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Output/Release)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set(CMAKE_CXX_FLAGS "/EHsc /WX /D_USE_MATH_DEFINES /D_SCL_SECURE_NO_WARNINGS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4267 /wd4244 /wd4305")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
endif()

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
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)
find_package(Cling REQUIRED)

#Third Party Libraries
set(LIBIGL_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/include CACHE FILEPATH "Root include directory for libigl")
set(LIBFIVE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/libfive/libfive/include CACHE FILEPATH "Root include directory for libfive")

#get header and source files
file(GLOB HEADER_CODE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_CODE ${PROJECT_SOURCE_DIR}/src/*.cpp)

#libfive utility files
#file(GLOB UTIL_HEADER_CODE ${PROJECT_SOURCE_DIR}/../ThirdParty/libfive/libfive/test/util/*.hpp)
#file(GLOB UTIL_SRC_CODE ${PROJECT_SOURCE_DIR}/../ThirdParty/libfive/libfive/test/util/*.cpp)

#Third Party Add Subdirectories
include_directories(${LIBFIVE_INCLUDE_PATH})
include_directories(${LIBFIVE_INCLUDE_PATH}/../test)
include_directories(${LIBIGL_INCLUDE_PATH})
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/glad/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/libigl/external/imgui)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../util/include)

#I need access to my include directories for the interpeter 

#add_subdirectory(${PROJECT_SOURCE_DIR}/../ThirdParty/libfive ${CMAKE_CURRENT_BINARY_DIR}/libfive)
add_subdirectory(${PROJECT_SOURCE_DIR}/../util ${CMAKE_CURRENT_BINARY_DIR}/util)

#add definitions for setting the interpreter libraries and include directories
add_definitions(-DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR})
add_definitions(-DA1_INCLUDE_DIR=${PROJECT_SOURCE_DIR}/include)
add_definitions(-DLIBFIVE_INCLUDE_DIR=${LIBFIVE_INCLUDE_PATH})
add_definitions(-DLIBFIVE_UTILS_INCLUDE_DIR=${LIBFIVE_INCLUDE_PATH}/../test)
add_definitions(-DUTILS_INCLUDE_DIR=${PROJECT_SOURCE_DIR}/../util/include)
    
#interp.loadLibrary("/Users/dilevin/Documents/Teaching/CompFabUofT/2018b/Instructor/A1/build/libfive/libfive/src/Release/libfive.dylib");
#interp.loadLibrary("/Users/dilevin/Documents/Teaching/CompFabUofT/2018b/Instructor/A1/build/util/Release/libutils.dylib");
if(APPLE)
    add_definitions(-DLIBFIVE_DYLIB=${CompFabA1_BINARY_DIR}/lib/libfive.dylib)
    add_definitions(-DUTILS_DYLIB=${CompFabA1_BINARY_DIR}/lib/libutils.dylib)
else()
    add_definitions(-DLIBFIVE_DYLIB=${CompFabA1_BINARY_DIR}/lib/libfive.so)
    add_definitions(-DUTILS_DYLIB=${CompFabA1_BINARY_DIR}/lib/libutils.so)
endif(APPLE)

add_executable(CompFabA1 ${SRC_CODE} ${HEADER_CODE} ${UTIL_HEADER_CODE} ${UTIL_SRC_CODE})
target_link_libraries(CompFabA1 five igl::core igl::opengl_glfw igl::opengl_glfw_imgui utils clingInterpreter) 
target_compile_options(CompFabA1 PUBLIC -DLLVMDIR="${LLVM_INSTALL_PREFIX}" -I${LLVM_INSTALL_PREFIX}/include)

set_target_properties(CompFabA1 PROPERTIES ENABLE_EXPORTS 1)

if(MSVC)
  set_target_properties(CompFabA1 PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 1)
  set_property(TARGET CompFabA1 APPEND_STRING PROPERTY LINK_FLAGS
              "/EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00D_K@Z
               /EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00DM@Z
               /EXPORT:cling_runtime_internal_throwIfInvalidPointer")
endif()



