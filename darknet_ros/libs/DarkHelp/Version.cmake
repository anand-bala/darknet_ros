# DarkHelp - C++ helper class for Darknet's C API. Copyright 2019-2021 Stephane
# Charette <stephanecharette@gmail.com> MIT license applies.  See "license.txt"
# for details.

set(DH_VER_MAJOR 1)
set(DH_VER_MINOR 1)
set(DH_VER_PATCH 9)
set(DH_VER_COMMIT 2)

set(DH_VERSION ${DH_VER_MAJOR}.${DH_VER_MINOR}.${DH_VER_PATCH})
set(DH_FULL_VERSION
    ${DH_VER_MAJOR}.${DH_VER_MINOR}.${DH_VER_PATCH}-${DH_VER_COMMIT}
)
message("Building ver: ${DH_FULL_VERSION}")

add_definitions(-DDH_VERSION="${DH_FULL_VERSION}")
