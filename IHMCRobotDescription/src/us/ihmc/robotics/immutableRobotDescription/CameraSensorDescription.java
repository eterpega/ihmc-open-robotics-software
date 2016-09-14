package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable
public interface CameraSensorDescription extends SensorDescription
{
   double getFieldOfView();

   double getClipNear();

   double getClipFar();

   int getImageWidth();

   int getImageHeight();
}
