package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class ExternalForcePointDescription implements KinematicPointDescription
{
   @Override public ModifiableObject toModifiable()
   {
      return ModifiableExternalForcePointDescription.create().from(this);
   }

   public static ImmutableExternalForcePointDescription.Builder builder()
   {
      return ImmutableExternalForcePointDescription.builder();
   }
}
