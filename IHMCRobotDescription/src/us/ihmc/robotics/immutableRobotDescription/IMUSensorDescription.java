package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable
public interface IMUSensorDescription extends SensorDescription
{
   double getAccelerationNoiseMean();

   double getAccelerationNoiseStandardDeviation();

   double getAccelerationBiasMean();

   double getAccelerationBiasStandardDeviation();

   double getAngularVelocityNoiseMean();

   double getAngularVelocityNoiseStandardDeviation();

   double getAngularVelocityBiasMean();

   double getAngularVelocityBiasStandardDeviation();
}
