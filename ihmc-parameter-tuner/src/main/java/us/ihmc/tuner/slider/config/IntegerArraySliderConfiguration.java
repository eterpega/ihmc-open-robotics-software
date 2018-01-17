package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.IntegerArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class IntegerArraySliderConfiguration implements SliderConfiguration
{
   private final int index;
   private final double initialValue;

   public IntegerArraySliderConfiguration(int index, double initialValue)
   {
      this.index = index;
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
      if (!(parameter instanceof IntegerArrayParameter))
      {
         throw new IllegalArgumentException("expected IntegerArrayParameter, got: " + parameter);
      }

      ((IntegerArrayParameter) parameter).set(index, (int) Math.round(value));
   }

   @Override
   public double getValueFromParameter(Parameter parameter)
   {
      if (!(parameter instanceof IntegerArrayParameter))
      {
         throw new IllegalArgumentException("expected IntegerArrayParameter, got: " + parameter);
      }

      return ((IntegerArrayParameter) parameter).get(index);
   }
}
