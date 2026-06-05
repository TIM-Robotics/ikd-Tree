cmake_minimum_required(VERSION 3.16)
project(ikd_Tree CXX)

# ----------------------------------------------------------------------------
# ikd-Tree is ROS-independent. Its only real dependencies are PCL (for the
# pcl::PointXYZ / PointXYZI / PointXYZINormal struct definitions) and Eigen
# (Eigen::aligned_allocator). The previous pcl_ros / pcl_conversions
# requirements were unused by the source and have been removed so the library
# builds (and is testable) on a lightweight PCL+Eigen image without ROS.
# ----------------------------------------------------------------------------

option(IKD_TREE_BUILD_TESTS "Build ikd-Tree unit/stress tests" OFF)
option(IKD_TREE_USE_SHIM_POINTTYPES
       "Use bundled minimal point-type shims instead of PCL (no PCL/Eigen needed)" OFF)
set(IKD_TREE_SANITIZER "none" CACHE STRING "Sanitizer: none | asan-ubsan | tsan")
set_property(CACHE IKD_TREE_SANITIZER PROPERTY STRINGS none asan-ubsan tsan)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

add_library(ikd-Tree STATIC ikd_Tree.cpp)
target_compile_features(ikd-Tree PUBLIC cxx_std_17)
target_include_directories(ikd-Tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

find_package(Threads REQUIRED)
target_link_libraries(ikd-Tree PUBLIC Threads::Threads)

if(IKD_TREE_USE_SHIM_POINTTYPES)
  message(STATUS "ikd-Tree: using bundled point-type shims (no PCL/Eigen)")
  target_include_directories(ikd-Tree PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/shims>)
else()
  find_package(PCL REQUIRED COMPONENTS common)
  target_include_directories(ikd-Tree PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(ikd-Tree PUBLIC ${PCL_LIBRARIES})
  # Eigen headers normally come transitively with PCL; link the imported target if present.
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(ikd-Tree PUBLIC Eigen3::Eigen)
  endif()
endif()

# ---- sanitizer helper (applied to the library AND the test binary) ----
function(ikd_apply_sanitizer tgt)
  if(IKD_TREE_SANITIZER STREQUAL "asan-ubsan")
    target_compile_options(${tgt} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
    # Static runtime embeds ASan/UBSan first in the binary, avoiding the
    # "runtime does not come first in initial library list" error without LD_PRELOAD.
    target_link_options(${tgt} PRIVATE -fsanitize=address,undefined -static-libasan -static-libubsan)
  elseif(IKD_TREE_SANITIZER STREQUAL "tsan")
    target_compile_options(${tgt} PRIVATE -fsanitize=thread -fno-omit-frame-pointer -g)
    target_link_options(${tgt} PRIVATE -fsanitize=thread -static-libtsan)
  elseif(NOT IKD_TREE_SANITIZER STREQUAL "none")
    message(FATAL_ERROR "Unknown IKD_TREE_SANITIZER='${IKD_TREE_SANITIZER}' (use none|asan-ubsan|tsan)")
  endif()
endfunction()
ikd_apply_sanitizer(ikd-Tree)

if(IKD_TREE_BUILD_TESTS)
  enable_testing()
  add_executable(test_ikd_tree test/test_ikd_tree.cpp)
  target_link_libraries(test_ikd_tree PRIVATE ikd-Tree)
  ikd_apply_sanitizer(test_ikd_tree)

  if(IKD_TREE_SANITIZER STREQUAL "tsan")
    # Modern kernels' high-entropy ASLR makes TSan abort with "unexpected memory
    # mapping"; running under setarch -R disables per-process randomization.
    add_test(NAME ikd_tree_unit COMMAND setarch -R $<TARGET_FILE:test_ikd_tree>)
  else()
    add_test(NAME ikd_tree_unit COMMAND test_ikd_tree)
  endif()
  set_tests_properties(ikd_tree_unit PROPERTIES TIMEOUT 300)
endif()
