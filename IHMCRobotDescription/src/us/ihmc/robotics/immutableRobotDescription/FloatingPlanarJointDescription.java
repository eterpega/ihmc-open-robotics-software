package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.Plane;

@Immutable public abstract class FloatingPlanarJointDescription implements JointDescription
{
   public abstract Plane getPlane();

   public static FloatingPlanarJointDescriptionBuilder builder()
   {
      return new FloatingPlanarJointDescriptionBuilder();
   }
}
