package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable
public interface ForceSensorDescription extends SensorDescription
{
   boolean getUseGroundContactPoints();
}
