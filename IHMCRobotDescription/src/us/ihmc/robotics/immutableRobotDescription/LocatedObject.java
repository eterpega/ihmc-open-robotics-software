package us.ihmc.robotics.immutableRobotDescription;

import javax.vecmath.Vector3d;

/**
 * Represents any object that has a 3D location in space represented as an offset from its parent.
 */
public interface LocatedObject
{
   Vector3d getOffsetFromJoint();
}
