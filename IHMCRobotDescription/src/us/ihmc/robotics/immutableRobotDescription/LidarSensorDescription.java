package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.lidar.LidarScanParameters;

@Immutable public abstract class LidarSensorDescription extends SensorDescription
{
   public abstract LidarScanParameters getLidarScanParameters();

   public static ImmutableLidarSensorDescription.Builder builder()
   {
      return ImmutableLidarSensorDescription.builder();
   }
}
