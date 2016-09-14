package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import us.ihmc.robotics.Plane;

@Immutable
public interface FloatingPlanarJointDescription extends JointDescription
{
   Plane getPlane();
}
