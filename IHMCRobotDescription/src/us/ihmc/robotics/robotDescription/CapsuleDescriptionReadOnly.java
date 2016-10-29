package us.ihmc.robotics.robotDescription;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CapsuleDescriptionReadOnly implements ConvexShapeDescription
{
   private final double radius;
   private final LineSegment3d capToCapLineSegment = new LineSegment3d();
   private final RigidBodyTransform transformToCenter;

   public CapsuleDescriptionReadOnly(double radius, double height, RigidBodyTransform transformToCenter)
   {
      this(radius, height, Axis.Z, transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, LineSegment3d capToCapLineSegment, RigidBodyTransform transformToCenter)
   {
      this.radius = radius;
      this.capToCapLineSegment.set(capToCapLineSegment);
      this.transformToCenter = new RigidBodyTransform(transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, double height, Axis longAxis, RigidBodyTransform transformToCenter)
   {
      if (height < 2.0 * radius)
         throw new RuntimeException("Capsule height must be at least 2.0 * radius!");
      this.radius = radius;

      switch (longAxis)
      {
      case X:
      {
         this.capToCapLineSegment.set(-height / 2.0 + radius, 0.0, 0.0, 0.0, 0.0, height / 2.0 - radius);
         break;
      }
      case Y:
      {
         this.capToCapLineSegment.set(0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius, 0.0);
         break;
      }
      case Z:
      {
         this.capToCapLineSegment.set(0.0, 0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius);
         break;
      }
      }

      this.transformToCenter = new RigidBodyTransform(transformToCenter);
   }

   public double getRadius()
   {
      return radius;
   }

   public void getCapToCapLineSegment(LineSegment3d lineSegmentToPack)
   {
      lineSegmentToPack.set(capToCapLineSegment);
   }

   public void getRigidBodyTransformToCenter(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToCenter);
   }

}
