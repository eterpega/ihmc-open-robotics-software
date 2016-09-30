package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class JointWrenchSensorDescription extends SensorDescription
{
   public static ImmutableJointWrenchSensorDescription.Builder builder()
   {
      return ImmutableJointWrenchSensorDescription.builder();
   }
}
