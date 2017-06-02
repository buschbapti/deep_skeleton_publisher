# - Try to find cuda
# Once done, this will define
#
#  cuda_FOUND - system has cuda
#  cuda_INCLUDE_DIRS - the cuda include directories
#  cuda_LIBRARIES - link these to use cuda

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(cuda_PKGCONF cuda)

# Include dir
find_path(cuda_INCLUDE_DIR
  NAMES cuda/include
  PATHS ${cuda_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(cuda_LIBRARY
  NAMES cuda
  PATHS ${cuda_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
set(cuda_PROCESS_INCLUDES cuda_INCLUDE_DIR)
set(cuda_PROCESS_LIBS cuda_LIBRARY)
libfind_process(cuda)