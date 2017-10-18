package us.ihmc.manipulation.planning.collisionAvoidance;

import java.util.ArrayList;

import com.jme3.math.LineSegment;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotDescription.CapsuleDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ConvexPolytopeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.ConvexShapeDescription;
import us.ihmc.robotics.robotDescription.CubeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CylinderDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RobotCollisionMeshProvider
{
   private final int defaultCurvedSurfaceDivisions;

   public RobotCollisionMeshProvider(int numberOfCurvedSurfaceDivisions)
   {
      this.defaultCurvedSurfaceDivisions = numberOfCurvedSurfaceDivisions;
   }

   public void createCollisionMeshesFromRobotDescription(FullRobotModel robotModel, RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJoints = robotDescription.getRootJoints();
      if (rootJoints.size() > 1 || !(rootJoints.get(0) instanceof FloatingJointDescription))
         throw new RuntimeException("There should be only one floating joint");
   }

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, CollisionMeshDescription meshDescription)
   {
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      ArrayList<ConvexShapeDescription> collisionShapeDescriptions = new ArrayList<>();
      meshDescription.getConvexShapeDescriptions(collisionShapeDescriptions);
      ArrayList<Point3D> points = new ArrayList<>();
      for (ConvexShapeDescription shapeDescription : collisionShapeDescriptions)
      {
         points.clear();
         if (shapeDescription instanceof SphereDescriptionReadOnly)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            ((SphereDescriptionReadOnly) shapeDescription).getRigidBodyTransform(transform);
            ConvexPolytopeConstructor.getCollisionMeshPointsForSphere(transform, ((SphereDescriptionReadOnly) shapeDescription).getRadius(), defaultCurvedSurfaceDivisions, points);
         }
         else if (shapeDescription instanceof CapsuleDescriptionReadOnly)
         {
            LineSegment3D lineSegment = new LineSegment3D();
            ((CapsuleDescriptionReadOnly) shapeDescription).getCapToCapLineSegment(lineSegment);
            ConvexPolytopeConstructor.getCollisionMeshPointsForCapsule(lineSegment, ((CapsuleDescriptionReadOnly) shapeDescription).getRadius(), defaultCurvedSurfaceDivisions, points);
         }
         else if (shapeDescription instanceof CylinderDescriptionReadOnly)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            ((CylinderDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
            ConvexPolytopeConstructor.getCylindericalCollisionMesh(transform, ((CylinderDescriptionReadOnly) shapeDescription).getRadius(), ((CylinderDescriptionReadOnly) shapeDescription).getHeight(), defaultCurvedSurfaceDivisions, points);
         }
         else if (shapeDescription instanceof CubeDescriptionReadOnly)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            ((CubeDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
            ConvexPolytopeConstructor.getCuboidCollisionMesh(transform, ((CubeDescriptionReadOnly) shapeDescription).getLengthX(), ((CubeDescriptionReadOnly) shapeDescription).getWidthY(), ((CubeDescriptionReadOnly) shapeDescription).getHeightZ(), points);
         }
         else if (shapeDescription instanceof ConvexPolytopeDescriptionReadOnly)
         {
            ((ConvexPolytopeDescriptionReadOnly) shapeDescription).getConvexPolytope().getVertices(points);
         }
         else
            throw new RuntimeException("Unhandled collision mesh description shape: " + shapeDescription.getClass());
         
      }
      return ConvexPolytopeConstructor.createFramePolytope(bodyFixedFrame, points);
   }
}
