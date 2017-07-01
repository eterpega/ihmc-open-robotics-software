#!/bin/bash

# Functions
function gradlePublish {
  cd $1
  echo "Publishing $1"
  gradle --no-daemon --info test
  cd ..
}

#gradlePublish IHMCJavaToolkit
#gradlePublish IHMCRoboticsToolkit
#gradlePublish IHMCCommonsStaging
#gradlePublish IHMCGeometry
gradlePublish IHMCCommunication
