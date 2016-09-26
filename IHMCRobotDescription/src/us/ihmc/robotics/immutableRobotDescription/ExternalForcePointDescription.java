package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class ExternalForcePointDescription implements KinematicPointDescription
{
   public static ExternalForcePointDescriptionBuilder builder()
   {
      return new ExternalForcePointDescriptionBuilder();
   }
}
