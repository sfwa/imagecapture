# - Try to find libusb-1.0
# Once done, this will define
#
#  LibUSB_FOUND - system has libusb
#  LibUSB_INCLUDE_DIRS - the libusb include directories
#  LibUSB_LIBRARY - link these to use libusb

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(LibUSB_PKGCONF libusb-1.0)

# Include dir
find_path(LibUSB_INCLUDE_DIR
  NAMES libusb.h
  PATHS ${LibUSB_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES libusb-1.0
)

# Finally the library itself
find_library(LibUSB_LIBRARY
  NAMES usb-1.0
  PATHS ${LibUSB_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(LibUSB_PROCESS_INCLUDES LibUSB_INCLUDE_DIR)
set(LibUSB_PROCESS_LIBS LibUSB_LIBRARY)
libfind_process(LibUSB)


