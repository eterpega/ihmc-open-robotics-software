package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class DoubleArraySliderConfiguration implements SliderConfiguration
{
   private final int index;
   private final double initialValue;

   public DoubleArraySliderConfiguration(int index, double initialValue)
   {
      this.index = index;
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
      if (!(parameter instanceof DoubleArrayParameter))
      {
         throw new IllegalArgumentException("expected DoubleArrayParameter, got: " + parameter);
      }

      ((DoubleArrayParameter) parameter).set(index, value);
   }

   @Override
   public double getValueFromParameter(Parameter parameter)
   {
      if (!(parameter instanceof DoubleArrayParameter))
      {
         throw new IllegalArgumentException("expected DoubleArrayParameter, got: " + parameter);
      }

      return ((DoubleArrayParameter) parameter).get(index);
   }
}
