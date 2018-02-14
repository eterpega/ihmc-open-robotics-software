package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;

public class CollisionMeshDescription implements CollisionMaskHolder
{
   private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
   private final ArrayList<ConvexShapeDescriptionReadOnly> convexShapeDescriptions = new ArrayList<>();
   private boolean isGround = false;
   private int collisionGroup = 0x00;
   private int collisionMask = 0x00;

   private int estimatedNumberOfContactPoints = 24;

   public void setEstimatedNumberOfContactPoints(int estimatedNumberOfContactPoints)
   {
      this.estimatedNumberOfContactPoints = estimatedNumberOfContactPoints;
   }

   public int getEstimatedNumberOfContactPoints()
   {
      return estimatedNumberOfContactPoints;
   }

   public void addConvexShape(ConvexShapeDescriptionReadOnly convexShapeDescription)
   {
      convexShapeDescriptions.add(convexShapeDescription);
   }

   public void addSphere(double radius)
   {
      SphereDescriptionReadOnly sphere = new SphereDescriptionReadOnly(radius, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(sphere);
   }

   public void addCapsule(double radius, LineSegment3D capToCapLineSegment)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, capToCapLineSegment, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCapsule(double radius, double height, Axis longAxis)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, longAxis, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCapsule(double radius, double height)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCubeReferencedAtBottomMiddle(double lengthX, double widthY, double heightZ)
   {
      transformGenerator.translate(0.0, 0.0, heightZ / 2.0);
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, transformGenerator.getRigidBodyTransformCopy());
      transformGenerator.translate(0.0, 0.0, -heightZ / 2.0);
      convexShapeDescriptions.add(cube);
   }

   public void addCubeReferencedAtCenter(double lengthX, double widthY, double heightZ)
   {
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cube);
   }

   public void addCylinderReferencedAtCenter(double radius, double height)
   {
      CylinderDescriptionReadOnly cylinder = new CylinderDescriptionReadOnly(radius, height, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cylinder);
   }

   public void addCylinderReferencedAtBottomMiddle(double radius, double height)
   {
      transformGenerator.translate(0.0, 0.0, height / 2.0);
      CylinderDescriptionReadOnly cylinder = new CylinderDescriptionReadOnly(radius, height, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cylinder);
      transformGenerator.translate(0.0, 0.0, -height / 2.0);
   }

   public void addConvexPolytope(ConvexPolytope polytope)
   {
      ConvexPolytopeDescriptionReadOnly polytopeReadOnly = new ConvexPolytopeDescriptionReadOnly(polytope, transformGenerator.getRigidBodyTransformCopy());
      convexShapeDescriptions.add(polytopeReadOnly);
   }

   public void getConvexShapeDescriptions(ArrayList<ConvexShapeDescriptionReadOnly> convexShapeDescriptionsToPack)
   {
      convexShapeDescriptionsToPack.addAll(convexShapeDescriptions);
   }

   public boolean getIsGround()
   {
      return isGround;
   }

   @Override
   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   @Override
   public int getCollisionMask()
   {
      return collisionMask;
   }

   public void setIsGround(boolean isGround)
   {
      this.isGround = isGround;
   }

   @Override
   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   @Override
   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   public void translate(double x, double y, double z)
   {
      transformGenerator.translate(x, y, z);
   }

   public void translate(Vector3D translationVector)
   {
      transformGenerator.translate(translationVector);
   }
   
   public void transform(RigidBodyTransform transform)
   {
      transformGenerator.setTransform(transform);
   }

   public void identity()
   {
      transformGenerator.identity();
   }

   public void rotateEuler(Vector3D eulerAngles)
   {
      transformGenerator.rotateEuler(eulerAngles);
   }

   public void rotate(RotationMatrixReadOnly rotation)
   {
      transformGenerator.rotate(rotation);
   }

   public void rotate(double rotationAngle, Axis axis)
   {
      transformGenerator.rotate(rotationAngle, axis);
   }

   public void scale(double factor)
   {
      throw new RuntimeException("TODO: Implement me");
   }
}
