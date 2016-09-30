package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class IMUSensorDescription extends SensorDescription
{
   public abstract GaussianParameter getAccelerationNoise();

   public abstract GaussianParameter getAccelerationBias();

   public abstract GaussianParameter getAngularVelocityNoise();

   public abstract GaussianParameter getAngularVelocityBias();

   public static ImmutableIMUSensorDescription.Builder builder()
   {
      return ImmutableIMUSensorDescription.builder();
   }
}
