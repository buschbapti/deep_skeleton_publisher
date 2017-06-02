# - Try to find openpose
# Once done, this will define
#
#  openpose_FOUND - system has openpose
#  openpose_INCLUDE_DIRS - the openpose include directories
#  openpose_LIBRARIES - link these to use openpose

include(LibFindMacros)

libfind_package(openpose cuda)
libfind_package(openpose caffe)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(openpose_PKGCONF openpose)

# Include dir
find_path(openpose_INCLUDE_DIR
  NAMES openpose
  PATHS ${openpose_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(openpose_LIBRARY
  NAMES openpose
  PATHS ${openpose_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
set(openpose_PROCESS_INCLUDES openpose_INCLUDE_DIR cuda_INCLUDE_DIR caffe_INCLUDE_DIR)
set(openpose_PROCESS_LIBS openpose_LIBRARY cuda_LIBRARY caffe_INCLUDE_DIR)
libfind_process(openpose)