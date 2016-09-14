package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class ForceSensorDescription implements SensorDescription
{
   public abstract boolean getUseGroundContactPoints();

   public static ForceSensorDescriptionBuilder builder()
   {
      return new ForceSensorDescriptionBuilder();
   }
}
