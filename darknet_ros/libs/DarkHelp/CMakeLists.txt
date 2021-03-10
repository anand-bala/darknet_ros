# DarkHelp - C++ helper class for Darknet's C API. Copyright 2019-2021 Stephane
# Charette <stephanecharette@gmail.com> MIT license applies.  See "license.txt"
# for details.

cmake_minimum_required(VERSION 3.10)
include(Version.cmake)

project(
  DarkHelp
  VERSION ${DH_VERSION}
  DESCRIPTION "C++ helper class for Darknet's C API."
  LANGUAGES C CXX
  HOMEPAGE_URL https://www.ccoderun.ca/darkhelp/api/index.html
)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui, ccmake
  set_property(
    CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel"
                                    "RelWithDebInfo"
  )
endif()
# Set standards
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Compiler definitions
function(default_compiler_definitions target)
  option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

  set(MSVC_WARNINGS
      /W4 # Baseline reasonable warnings
      /wd4100 # disable "unreferenced formal parameter"
      /permissive- # standards conformance mode for MSVC compiler.
  )

  set(CLANG_WARNINGS
      -Wall # Enable default warnings
      -Wextra # reasonable and standard
      -Wpedantic # warn if non-standard C++ is used
      -Wno-unused-parameter
  )

  if(WARNINGS_AS_ERRORS)
    set(CLANG_WARNINGS ${CLANG_WARNINGS} -Werror)
    set(MSVC_WARNINGS ${MSVC_WARNINGS} /WX)
  endif()

  set(GCC_WARNINGS ${CLANG_WARNINGS})

  if(MSVC)
    set(PROJECT_WARNINGS ${MSVC_WARNINGS})
  elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_WARNINGS ${CLANG_WARNINGS})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS})
  else()
    message(
      AUTHOR_WARNING
        "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler."
    )
  endif()

  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
  target_compile_features(${target} PUBLIC cxx_std_11)
endfunction()

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
# Assume that darknet has already been built by parent project

add_library(DarkHelp src/DarkHelp.cpp)
target_include_directories(DarkHelp PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DarkHelp PRIVATE dark)
default_compiler_definitions(DarkHelp)

install(FILES include/DarkHelp.hpp DESTINATION include)
install(TARGETS DarkHelp DESTINATION lib)
