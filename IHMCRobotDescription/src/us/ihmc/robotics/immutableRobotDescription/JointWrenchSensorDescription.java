package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class JointWrenchSensorDescription implements SensorDescription
{
   public static JointWrenchSensorDescriptionBuilder builder()
   {
      return new JointWrenchSensorDescriptionBuilder();
   }
}
