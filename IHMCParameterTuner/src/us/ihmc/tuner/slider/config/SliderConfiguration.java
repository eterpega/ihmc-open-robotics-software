package us.ihmc.tuner.slider.config;

import us.ihmc.robotics.dataStructures.parameter.Parameter;

public interface SliderConfiguration
{
   double defaultLowerLimit();
   boolean lowerLimitMutable();
   double defaultUpperLimit();
   boolean upperLimitMutable();

   boolean snapToTicks();
   int minorTickCount(double lower, double upper);
   int majorTickCount(double lower, double upper);

   void applyValueToParameter(double value, Parameter parameter);
   double getValueFromParameter(Parameter parameter);
}
