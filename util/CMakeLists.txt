#Notes
#Required dependencies
#OSX/LINUX PKGCONGIG
cmake_minimum_required (VERSION 3.2)

project(utils)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ThirdParty/eigen3.3/cmake CACHE PATH "Root include directory for Eigen3" FORCE)

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

set(LIBFIVE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/libfive/libfive/include CACHE FILEPATH "Root include directory for libfive")

#get header and source files
file(GLOB HEADER_CODE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_CODE ${PROJECT_SOURCE_DIR}/src/*.cpp)

find_package(Eigen3 REQUIRED)

#Third Party Add Subdirectories
include_directories(${PROJECT_SOURCE_PATH}/include)
include_directories(${LIBFIVE_INCLUDE_PATH})
include_directories(${LIBFIVE_INCLUDE_PATH}/../test)
include_directories(${EIGEN3_INCLUDE_DIR})

#I need access to my include directories for the interpeter 
#add_subdirectory(${PROJECT_SOURCE_DIR}/../ThirdParty/libfive ${CMAKE_CURRENT_BINARY_DIR}/libfive)
add_subdirectory(${PROJECT_SOURCE_DIR}/../ThirdParty/libfive ${CMAKE_CURRENT_BINARY_DIR}/libfive)

add_library(utils SHARED ${SRC_CODE} ${HEADER_CODE})
target_link_libraries(utils five)
target_compile_options(utils PUBLIC -DLLVMDIR="${LLVM_INSTALL_PREFIX}" -I${LLVM_INSTALL_PREFIX}/include)

set_target_properties(utils PROPERTIES ENABLE_EXPORTS 1)

if(MSVC)
  set_target_properties(utils PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 1)
  set_property(TARGET utils APPEND_STRING PROPERTY LINK_FLAGS
              "/EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00D_K@Z
               /EXPORT:?setValueNoAlloc@internal@runtime@cling@@YAXPEAX00DM@Z
               /EXPORT:cling_runtime_internal_throwIfInvalidPointer")
endif()



