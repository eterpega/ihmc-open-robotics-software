package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Parameter;

/**
 * A wrapper class for mean and standard deviation
 */
@Immutable public abstract class GaussianParameter
{
   @Parameter public abstract double getMean();

   @Parameter public abstract double getStandardDeviation();

   public static GaussianParameterBuilder builder()
   {
      return new GaussianParameterBuilder();
   }

   public static GaussianParameter fromMeanStd(double mean, double standardDeviation)
   {
      return builder().mean(mean).standardDeviation(standardDeviation).build();
   }
}
