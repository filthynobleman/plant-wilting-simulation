# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src/Ext_Discregrid"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src/Ext_Discregrid-build"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/Discregrid"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/tmp"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src/Ext_Discregrid-stamp"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src"
  "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src/Ext_Discregrid-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Filippo/Documents/workspace/plant-wiltin-simulation/build/extern/Discregrid/src/Ext_Discregrid-stamp/${subDir}")
endforeach()
