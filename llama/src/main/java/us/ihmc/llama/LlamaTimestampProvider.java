package us.ihmc.llama;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.commons.Conversions;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;

public class LlamaTimestampProvider implements SensorTimestampHolder
{
   private final FloatingRootJointRobot sdfRobot;

   public LlamaTimestampProvider(FloatingRootJointRobot sdfRobot)
   {
      this.sdfRobot = sdfRobot;
   }
   
   @Override
   public long getTimestamp()
   {
      return Conversions.secondsToNanoseconds(sdfRobot.getYoTime().getDoubleValue());
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return getTimestamp();
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return -1;
   }
}
