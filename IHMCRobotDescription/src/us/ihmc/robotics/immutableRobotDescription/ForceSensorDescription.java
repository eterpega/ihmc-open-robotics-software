package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class ForceSensorDescription extends SensorDescription
{
   @Override public ModifiableForceSensorDescription toModifiable()
   {
      return ModifiableForceSensorDescription.create().from(this);
   }

   public abstract boolean getUseGroundContactPoints();

   public static ImmutableForceSensorDescription.Builder builder()
   {
      return ImmutableForceSensorDescription.builder();
   }
}
