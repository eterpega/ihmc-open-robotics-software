package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class GroundContactPointDescription implements KinematicPointDescription
{
   public static GroundContactPointDescriptionBuilder builder()
   {
      return new GroundContactPointDescriptionBuilder();
   }
}
