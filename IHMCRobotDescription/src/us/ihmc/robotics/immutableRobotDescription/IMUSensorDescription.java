package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class IMUSensorDescription implements SensorDescription
{
   public abstract double getAccelerationNoiseMean();

   public abstract double getAccelerationNoiseStandardDeviation();

   public abstract double getAccelerationBiasMean();

   public abstract double getAccelerationBiasStandardDeviation();

   public abstract double getAngularVelocityNoiseMean();

   public abstract double getAngularVelocityNoiseStandardDeviation();

   public abstract double getAngularVelocityBiasMean();

   public abstract double getAngularVelocityBiasStandardDeviation();

   public static IMUSensorDescriptionBuilder builder()
   {
      return new IMUSensorDescriptionBuilder();
   }
}
