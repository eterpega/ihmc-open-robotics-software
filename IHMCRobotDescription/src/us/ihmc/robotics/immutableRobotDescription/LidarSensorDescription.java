package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;
import us.ihmc.robotics.lidar.LidarScanParameters;

@Immutable @Modifiable public abstract class LidarSensorDescription extends SensorDescription
{
   public abstract LidarScanParameters getLidarScanParameters();

   @Override public ModifiableLidarSensorDescription toModifiable()
   {
      return ModifiableLidarSensorDescription.create().from(this);
   }

   public static ImmutableLidarSensorDescription.Builder builder()
   {
      return ImmutableLidarSensorDescription.builder();
   }
}
