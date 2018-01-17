package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class IntegerSliderConfiguration implements SliderConfiguration
{
   private final int initialValue;

   public IntegerSliderConfiguration(int initialValue)
   {
      this.initialValue = initialValue;
   }

   @Override
   public double defaultLowerLimit()
   {
      // TODO: Think of a more reasonable way to calculate default limits.
      return initialValue - 1;
   }

   @Override
   public double defaultUpperLimit()
   {
      // TODO: Think of a more reasonable way to calculate default limits.
      return initialValue + 1;
   }

   @Override
   public boolean snapToTicks()
   {
      return true;
   }

   @Override
   public int minorTickCount(double lower, double upper)
   {
      return 4;
   }

   @Override
   public int majorTickCount(double lower, double upper)
   {
      return 1;
   }

   @Override
   public boolean lowerLimitMutable()
   {
      return true;
   }

   @Override
   public boolean upperLimitMutable()
   {
      return true;
   }

   @Override
   public void applyValueToParameter(double value, Parameter parameter)
   {
      if (!(parameter instanceof IntegerParameter))
      {
         throw new IllegalArgumentException("expected IntegerParameter, got: " + parameter);
      }

      ((IntegerParameter) parameter).set((int) Math.round(value));
   }

   @Override
   public double getValueFromParameter(Parameter parameter)
   {
      if (!(parameter instanceof IntegerParameter))
      {
         throw new IllegalArgumentException("expected IntegerParameter, got: " + parameter);
      }

      return ((IntegerParameter) parameter).get();
   }
}
