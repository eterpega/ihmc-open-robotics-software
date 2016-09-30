package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class IMUSensorDescription extends SensorDescription
{
   public abstract GaussianParameter getAccelerationNoise();

   public abstract GaussianParameter getAccelerationBias();

   public abstract GaussianParameter getAngularVelocityNoise();

   public abstract GaussianParameter getAngularVelocityBias();

   @Override public ModifiableIMUSensorDescription toModifiable()
   {
      return ModifiableIMUSensorDescription.create().from(this);
   }

   public static ImmutableIMUSensorDescription.Builder builder()
   {
      return ImmutableIMUSensorDescription.builder();
   }
}
