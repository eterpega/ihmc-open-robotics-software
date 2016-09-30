package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class GroundContactPointDescription implements KinematicPointDescription
{
   @Override public ModifiableGroundContactPointDescription toModifiable()
   {
      return ModifiableGroundContactPointDescription.create().from(this);
   }

   public static ImmutableGroundContactPointDescription.Builder builder()
   {
      return ImmutableGroundContactPointDescription.builder();
   }
}
