package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class GroundContactPointDescription implements KinematicPointDescription
{
   public static ImmutableGroundContactPointDescription.Builder builder()
   {
      return ImmutableGroundContactPointDescription.builder();
   }
}
