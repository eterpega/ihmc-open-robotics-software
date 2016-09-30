package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;
import us.ihmc.robotics.Plane;

@Immutable @Modifiable public abstract class FloatingPlanarJointDescription extends JointDescription
{
   public abstract Plane getPlane();

   @Override public final ModifiableFloatingPlanarJointDescription toModifiable()
   {
      return ModifiableFloatingPlanarJointDescription.create().from(this);
   }

   public static ImmutableFloatingPlanarJointDescription.Builder builder()
   {
      return ImmutableFloatingPlanarJointDescription.builder();
   }

   static abstract class Builder implements JointDescription.Builder
   {
   }
}
