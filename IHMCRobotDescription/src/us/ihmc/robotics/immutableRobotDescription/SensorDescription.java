package us.ihmc.robotics.immutableRobotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Vector3d;

public abstract class SensorDescription implements NamedObject, LocatedObject, ModifiableObject
{
   public abstract RigidBodyTransform getTransformToJoint();

   @Override public final Vector3d getOffsetFromJoint()
   {
      Vector3d result = new Vector3d();
      getTransformToJoint().getTranslation(result);
      return result;
   }
}
