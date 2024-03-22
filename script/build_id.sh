#! /bin/bash

dir="../inc"

# Get the latest commit SHA from Git
ID=$(git rev-parse HEAD | head -c 8)

# Output a C++ header file with the build ID as a constant
cat << EOF > $dir/BuildID.hpp
#pragma once

#include <cstdint>

namespace mercury::blackstar
{
    static const uint32_t k_buildID { 0x$ID };
}

EOF
