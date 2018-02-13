package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class CylinderDescriptionReadOnly implements ConvexShapeDescriptionReadOnly
{
   private final double radius, height;
   private final RigidBodyTransform rigidBodyTransformToCenter;

   public CylinderDescriptionReadOnly(double radius, double height, RigidBodyTransform rigidBodyTransform)
   {
      this.radius = radius;
      this.height = height;
      this.rigidBodyTransformToCenter = new RigidBodyTransform(rigidBodyTransform);
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void getRigidBodyTransformToCenter(RigidBodyTransform transformToPack)
   {
      transformToPack.set(rigidBodyTransformToCenter);
   }

}
