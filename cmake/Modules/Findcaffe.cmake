# - Try to find caffe
# Once done, this will define
#
#  caffe_FOUND - system has cuda
#  caffe_INCLUDE_DIRS - the cuda include directories
#  caffe_LIBRARIES - link these to use cuda

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(caffe_PKGCONF caffe)

# Include dir
find_path(cuda_INCLUDE_DIR
  NAMES caffe
  PATHS ${caffe_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(caffe_LIBRARY
  NAMES caffe
  PATHS ${caffe_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
set(caffe_PROCESS_INCLUDES caffe_INCLUDE_DIR)
set(caffe_PROCESS_LIBS caffe_LIBRARY)
libfind_process(caffe)