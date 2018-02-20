package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class Capsule3D extends Shape3D<Capsule3D>
{
   /** Radius of the cylinder part. */
   private double radius;
   private LineSegment3D line3D = new LineSegment3D();

   public Capsule3D()
   {

   }

   public Capsule3D(Capsule3D other)
   {
      set(other);
   }

   public Capsule3D(double radius, LineSegment3D line3D)
   {
      setRadius(radius);
      setLineSegment(line3D);
   }

   public Capsule3D(RigidBodyTransform pose, double radius, LineSegment3D line3D)
   {
      setPose(pose);
      setRadius(radius);
      setLineSegment(line3D);
   }

   public Capsule3D(Pose3D pose, double radius, LineSegment3D line3D)
   {
      setPose(shapePose);
      setRadius(radius);
      setLineSegment(line3D);
   }

   public void setRadius(double radius)
   {
      if (2 * radius > line3D.length())
         throw new RuntimeException("Capsule radius is too long comparing to its length");
      this.radius = radius;
   }

   public void setLineSegment(LineSegment3D line3D)
   {
      this.line3D.set(line3D);
   }

   public void setLineSegment(Point3DReadOnly pointOne, Point3DReadOnly pointTwo)
   {
      this.line3D.set(pointOne, pointTwo);
   }

   public double getRadius()
   {
      return radius;
   }

   public LineSegment3D getLineSegment()
   {
      return line3D;
   }

   @Override
   public boolean epsilonEquals(Capsule3D other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void set(Capsule3D other)
   {
      setPose(other.shapePose);
      setRadius(other.radius);
      setLineSegment(other.line3D);
   }

   @Override
   public boolean geometricallyEquals(Capsule3D other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   protected boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

}
