package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class DoubleSliderConfiguration implements SliderConfiguration
{
   private final double initialValue;

   public DoubleSliderConfiguration(double initialValue)
   {
      this.initialValue = initialValue;
   }

   @Override
   public double defaultLowerLimit()
   {
      // TODO: Think of a more reasonable way to calculate default limits.
      return Math.floor(initialValue - 1);
   }

   @Override
   public double defaultUpperLimit()
   {
      // TODO: Think of a more reasonable way to calculate default limits.
      return Math.ceil(initialValue + 1);
   }

   @Override
   public boolean snapToTicks()
   {
      return false;
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
      if (!(parameter instanceof DoubleParameter))
      {
         throw new IllegalArgumentException("expected DoubleParameter, got: " + parameter);
      }

      ((DoubleParameter) parameter).set(value);
   }

   @Override
   public double getValueFromParameter(Parameter parameter)
   {
      if (!(parameter instanceof DoubleParameter))
      {
         throw new IllegalArgumentException("expected DoubleParameter, got: " + parameter);
      }

      return ((DoubleParameter) parameter).get();
   }
}
