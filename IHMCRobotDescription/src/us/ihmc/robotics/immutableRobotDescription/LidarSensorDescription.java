package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.lidar.LidarScanParameters;

@Immutable
public interface LidarSensorDescription extends SensorDescription
{
   LidarScanParameters getLidarScanParameters();
}
