
#--------------------------------------------------------------------------------------------------
# CMake configuration.
#--------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

cmake_policy(SET CMP0079 NEW) # Needed to import external libraries

#--------------------------------------------------------------------------------------------------
# Project configuration.
#--------------------------------------------------------------------------------------------------

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not permitted; run CMake inside an empty build directory.")
endif()

set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Profile" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release
    CACHE STRING "Choose the type of build, options are: ${CMAKE_CONFIGURATION_TYPES}" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif()

message(STATUS "Build Type: " ${CMAKE_BUILD_TYPE})

project(Unfolding)

#--------------------------------------------------------------------------------------------------
# Language and compiler features.
#--------------------------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#--------------------------------------------------------------------------------------------------
# Build options.
#--------------------------------------------------------------------------------------------------

option(WITH_FRONTEND "Build Project With Frontend?" OFF)
message(STATUS "With Frontend: " ${WITH_FRONTEND})

#--------------------------------------------------------------------------------------------------
# Find packages.
#--------------------------------------------------------------------------------------------------

if(WITH_FRONTEND)
  # OpenGL
  set(OpenGL_GL_PREFERENCE LEGACY)
  find_package(OpenGL REQUIRED)
endif()

# OpenMP
find_package(OpenMP REQUIRED)

if(WITH_FRONTEND)
  # QT
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_AUTOUIC ON)

  find_package(Qt5 COMPONENTS
                Core
                Widgets
                OpenGL
              REQUIRED)
endif()

#--------------------------------------------------------------------------------------------------
# Include paths.
#--------------------------------------------------------------------------------------------------

# Includes
include_directories(
  SYSTEM external/Eigen
  SYSTEM external/libigl/include
  SYSTEM external/Spectra/include
  src
)

# Subdirectories
add_subdirectory(src)

# TODO: Verschiedene Compiler unterstützen

#--------------------------------------------------------------------------------------------------
# Compilation/Linking Flags.
#--------------------------------------------------------------------------------------------------

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  # Windows
  target_compile_options(Unfolding PUBLIC
    # Common flags
    /Wall
    /Wp64
    # Release
    $<$<CONFIG:RELEASE>:/O3>
    $<$<CONFIG:RELEASE>:/WX>
    # Debug
    $<$<CONFIG:DEBUG>:/Od>
  )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Linux
  target_compile_options(Unfolding PUBLIC
    # Common flags
    -Wall
    -Wpedantic
    -Wno-reorder
    # Release
    $<$<CONFIG:RELEASE>:-O3>
    $<$<CONFIG:RELEASE>:-Werror>
    # Debug
    $<$<CONFIG:DEBUG>:-Og>
    # Profile
    $<$<CONFIG:Profile>:-O3>
    $<$<CONFIG:Profile>:-pg>
    $<$<CONFIG:Profile>:-Werror>
  )
  target_link_options(Unfolding PUBLIC
    $<$<CONFIG:Profile>:-pg>
  )
else()
  # MacOS
endif()


# Link library
target_link_libraries(Unfolding
  # Common
  OpenMP::OpenMP_CXX
  # Frontent
  $<$<BOOL:${WITH_FRONTEND}>:OpenGL::GL>
  $<$<BOOL:${WITH_FRONTEND}>:Qt5::Widgets>
  $<$<BOOL:${WITH_FRONTEND}>:Qt5::Core>
  $<$<BOOL:${WITH_FRONTEND}>:Qt5::OpenGL>
)


#--------------------------------------------------------------------------------------------------
# Shaders.
#--------------------------------------------------------------------------------------------------

# Create Copy Command
add_custom_target(
  copy_shader_files
  ${CMAKE_COMMAND}
  -D SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  -D DESTINATION_DIR=${CMAKE_CURRENT_BINARY_DIR} 
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyShaders.cmake
  comment "Copying Files for target: Unfolding" VERBATIM 
)

add_dependencies(Unfolding copy_shader_files)
