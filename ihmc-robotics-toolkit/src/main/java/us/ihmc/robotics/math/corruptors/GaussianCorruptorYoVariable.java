package us.ihmc.robotics.math.corruptors;

import java.util.Random;

import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GaussianCorruptorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final Random random;
   private final DoubleProvider standardDeviation;
   private final DoubleProvider input;

   private static DoubleProvider createStandardDeviationYoDouble(String namePrefix, double initialValue, YoVariableRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "StandardDeviation", registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public GaussianCorruptorYoVariable(String name, YoVariableRegistry registry, Random random, double standardDeviation)
   {
      this(name, registry, random, standardDeviation, null);
   }

   public GaussianCorruptorYoVariable(String name, YoVariableRegistry registry, Random random, double standardDeviation, DoubleProvider inputVariable)
   {
      this(name, "", registry, random, createStandardDeviationYoDouble(name, standardDeviation, registry), inputVariable);
   }

   public GaussianCorruptorYoVariable(String name, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation)
   {
      this(name, "", registry, random, standardDeviation, null);
   }

   public GaussianCorruptorYoVariable(String name, String description, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation)
   {
      this(name, description, registry, random, standardDeviation, null);
   }

   public GaussianCorruptorYoVariable(String name, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation, DoubleProvider inputVariable)
   {
      this(name, "", registry, random, standardDeviation, inputVariable);
   }

   public GaussianCorruptorYoVariable(String name, String description, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation,
                                      DoubleProvider inputVariable)
   {
      super(name, description, registry);
      this.random = random;
      this.input = inputVariable;

      if (standardDeviation == null)
         standardDeviation = createStandardDeviationYoDouble(name, 0.0, registry);
      this.standardDeviation = standardDeviation;
   }

   @Override
   public void update()
   {
      if (input == null)
      {
         throw new NullPointerException("GaussianCorruptorYoVariable must be constructed with a non null "
               + "input variable to call update(), otherwise use update(double)");
      }

      update(input.getValue());
   }

   public void update(double input)
   {
      double noise = standardDeviation.getValue() * random.nextGaussian();
      set(input + noise);
   }
}
