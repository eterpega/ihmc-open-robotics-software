package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.Plane;

@Immutable public abstract class FloatingPlanarJointDescription extends JointDescription
{
   public abstract Plane getPlane();

   public static ImmutableFloatingPlanarJointDescription.Builder builder()
   {
      return ImmutableFloatingPlanarJointDescription.builder();
   }

   static abstract class Builder implements JointDescription.Builder
   {
   }
}
