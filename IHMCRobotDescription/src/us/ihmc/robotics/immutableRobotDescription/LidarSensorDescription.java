package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.lidar.LidarScanParameters;

@Immutable public abstract class LidarSensorDescription implements SensorDescription
{
   public abstract LidarScanParameters getLidarScanParameters();

   public static LidarSensorDescriptionBuilder builder()
   {
      return new LidarSensorDescriptionBuilder();
   }
}
