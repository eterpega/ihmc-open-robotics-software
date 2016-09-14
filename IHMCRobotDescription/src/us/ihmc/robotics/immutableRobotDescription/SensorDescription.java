package us.ihmc.robotics.immutableRobotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface SensorDescription extends NamedObject, LocatedObject
{
   RigidBodyTransform getTransformToJoint();
}
