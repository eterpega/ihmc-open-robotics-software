package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

import javax.vecmath.Vector3d;

@Immutable @Modifiable public abstract class FloatingJointDescription extends JointDescription
{
   static abstract class Builder implements JointDescription.Builder
   {
   }

   @Default @Override public Vector3d getOffsetFromJoint()
   {
      return new Vector3d();
   }
}

