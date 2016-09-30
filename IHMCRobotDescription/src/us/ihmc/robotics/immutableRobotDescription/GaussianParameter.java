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

   public static ImmutableGaussianParameter.Builder builder()
   {
      return ImmutableGaussianParameter.builder();
   }

   public static GaussianParameter fromMeanStd(double mean, double standardDeviation)
   {
      return builder().mean(mean).standardDeviation(standardDeviation).build();
   }
}
