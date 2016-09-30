package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class CameraSensorDescription extends SensorDescription
{
   public abstract double getFieldOfView();

   public abstract double getClipNear();

   public abstract double getClipFar();

   public abstract int getImageWidth();

   public abstract int getImageHeight();

   @Override public ModifiableCameraSensorDescription toModifiable()
   {
      return ModifiableCameraSensorDescription.create().from(this);
   }

   public static ImmutableCameraSensorDescription.Builder builder()
   {
      return ImmutableCameraSensorDescription.builder();
   }
}
