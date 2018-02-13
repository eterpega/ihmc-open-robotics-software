package us.ihmc.communication.packets;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class SimulatedLidarScanPacket extends Packet<SimulatedLidarScanPacket>
{
   public float[] ranges;
   public int sensorId;
   public LidarScanParameters params;

   public SimulatedLidarScanPacket()
   {
   }

   public SimulatedLidarScanPacket(int sensorId, LidarScanParameters params, float[] ranges)
   {
      this.ranges = ranges;
      this.sensorId = sensorId;
      this.params = params;
   }

   @Override
   public boolean epsilonEquals(SimulatedLidarScanPacket other, double epsilon)
   {
      boolean ret = true;
      for (int i = 0; i < ranges.length; i++)
      {
         ret &= MathTools.epsilonEquals(ranges[i], other.ranges[i], epsilon);
      }

      ret &= params.equals(other.params);

      return ret;
   }

   public long getScanStartTime()
   {
      return params.getTimestamp();
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return params;
   }

   public float[] getRanges()
   {
      return ranges;
   }

   public int getSensorId()
   {
      return sensorId;
   }

   public int size()
   {
      return ranges.length;
   }
}
