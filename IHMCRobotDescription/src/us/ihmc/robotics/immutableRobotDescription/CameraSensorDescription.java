package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class CameraSensorDescription extends SensorDescription
{
   public abstract double getFieldOfView();

   public abstract double getClipNear();

   public abstract double getClipFar();

   public abstract int getImageWidth();

   public abstract int getImageHeight();

   public static ImmutableCameraSensorDescription.Builder builder()
   {
      return ImmutableCameraSensorDescription.builder();
   }
}
