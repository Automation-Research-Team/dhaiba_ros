# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindDhaibaConnectN
# --------
#
# Find DhaibaConnectN
#
# ::
#
#   DhaibaConnectN_FOUND       - True if DhaibaConnectN_INCLUDE_DIR & DhaibaConnectN_LIBRARY are found
#   DhaibaConnectN_LIBRARY     - path to DhaibaConnectN Library
#   DhaibaConnectN_INCLUDE_DIR - path to DhaibaConnectN Header files
#

find_path(DhaibaConnectN_INCLUDE_DIR
  NAMES DhaibaConnectN/Common.h
  PATHS
    /usr/local/include/Dhaiba
    /opt/DhaibaConnect_r200914/DhaibaConnectN/include
  PATH_SUFFIXES Dhaiba
)

find_library(DhaibaConnectN_LIBRARY
  NAMES DhaibaConnectN
  PATHS
    /usr/local/lib
    /opt/DhaibaConnect_r200914/DhaibaConnectN/lib
)

if (DhaibaConnectN_INCLUDE_DIR AND DhaibaConnectN_LIBRARY)
    set(DhaibaConnectN_FOUND 1)
endif()

mark_as_advanced(
  DhaibaConnectN_INCLUDE_DIR
  DhaibaConnectN_LIBRARY
)

