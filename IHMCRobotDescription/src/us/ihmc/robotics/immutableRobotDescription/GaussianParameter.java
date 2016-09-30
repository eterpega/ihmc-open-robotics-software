package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;
import org.immutables.value.Value.Parameter;

/**
 * A wrapper class for mean and standard deviation
 */
@Immutable @Modifiable public abstract class GaussianParameter implements ModifiableObject
{
   @Parameter public abstract double getMean();

   @Parameter public abstract double getStandardDeviation();

   @Override public ModifiableGaussianParameter toModifiable()
   {
      return ModifiableGaussianParameter.create().from(this);
   }

   public static ImmutableGaussianParameter.Builder builder()
   {
      return ImmutableGaussianParameter.builder();
   }

   public static GaussianParameter fromMeanStd(double mean, double standardDeviation)
   {
      return builder().mean(mean).standardDeviation(standardDeviation).build();
   }
}
