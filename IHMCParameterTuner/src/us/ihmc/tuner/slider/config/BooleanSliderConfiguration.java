package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class BooleanSliderConfiguration implements SliderConfiguration
{
   @Override
   public double defaultLowerLimit()
   {
      return 0.0;
   }

   @Override
   public double defaultUpperLimit()
   {
      return 1.0;
   }

   @Override
   public boolean snapToTicks()
   {
      return true;
   }

   @Override
   public int minorTickCount(double lower, double upper)
   {
      return 0;
   }

   @Override
   public int majorTickCount(double lower, double upper)
   {
      return 1;
   }

   @Override
   public boolean lowerLimitMutable()
   {
      return false;
   }

   @Override
   public boolean upperLimitMutable()
   {
      return false;
   }

   @Override
   public void applyValueToParameter(double value, Parameter parameter)
   {
      if (!(parameter instanceof BooleanParameter))
      {
         throw new IllegalArgumentException("expected BooleanParameter, got: " + parameter);
      }

      ((BooleanParameter) parameter).set(value >= 0.5);
   }

   @Override
   public double getValueFromParameter(Parameter parameter)
   {
      if (!(parameter instanceof BooleanParameter))
      {
         throw new IllegalArgumentException("expected BooleanParameter, got: " + parameter);
      }

      return ((BooleanParameter) parameter).get() ? 1.0 : 0.0;
   }
}
