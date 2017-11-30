package us.ihmc.avatar.collisionAvoidance;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RobotCollisionMeshProvider
{
   /**
    * Debug flag. Setting this to true will enable the debug prints
    */
   private static final boolean debug = false;

   /**
    * Controls the number of segments a curved surface is broken up into. A larger value will
    * represent a smooth surface more accurately
    */
   private final int defaultCurvedSurfaceDivisions;

   public RobotCollisionMeshProvider(int numberOfCurvedSurfaceDivisions)
   {
      this.defaultCurvedSurfaceDivisions = numberOfCurvedSurfaceDivisions;
   }

   public Map<RigidBody, FrameConvexPolytope> createCollisionMeshesFromRobotDescription(RigidBody rootBody, JointDescription rootJointDescription)
   {
      Map<String, RigidBody> nameToRigidBodyMap = createNameBasedRigidBodyMap(rootBody);
      Map<String, LinkDescription> nameToLinkDescriptionMap = createNameToLinkDescriptionMap(rootJointDescription);
      Map<RigidBody, FrameConvexPolytope> collisionMeshMap = new HashMap<>();

      for (String name : nameToLinkDescriptionMap.keySet())
      {
         RigidBody rigidBody = nameToRigidBodyMap.get(name);
         LinkDescription linkDescription = nameToLinkDescriptionMap.get(name);
         collisionMeshMap.put(rigidBody, createCollisionMesh(rigidBody, linkDescription));
      }
      return collisionMeshMap;
   }

   public Map<RigidBody, FrameConvexPolytope> createCollisionMeshesFromRobotDescription(FullRobotModel fullRobotModel, RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJointDescriptions = robotDescription.getRootJoints();

      if (rootJointDescriptions.size() > 1 || !(rootJointDescriptions.get(0) instanceof FloatingJointDescription))
         throw new RuntimeException("There should be only one floating joint");

      FloatingJointDescription rootJointDescription = (FloatingJointDescription) rootJointDescriptions.get(0);
      return createCollisionMeshesFromRobotDescription(fullRobotModel.getElevator(), rootJointDescription);
   }

   private static Map<String, LinkDescription> createNameToLinkDescriptionMap(JointDescription rootJointDescription)
   {
      Map<String, LinkDescription> nameToLinkDescriptionMap = new HashMap<>();
      recursivelyAddLinkeDescriptions(rootJointDescription, nameToLinkDescriptionMap);
      return nameToLinkDescriptionMap;
   }

   private static void recursivelyAddLinkeDescriptions(JointDescription rootJointDescription, Map<String, LinkDescription> nameToLinkDescriptionMap)
   {
      nameToLinkDescriptionMap.put(rootJointDescription.getLink().getName(), rootJointDescription.getLink());
      for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
         recursivelyAddLinkeDescriptions(jointDescription, nameToLinkDescriptionMap);
   }

   private static Map<String, RigidBody> createNameBasedRigidBodyMap(RigidBody rootBody)
   {
      Map<String, RigidBody> nameToRigidBodyMap = new HashMap<>();
      RigidBody[] rigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (int i = 0; i < rigidBodies.length; i++)
         nameToRigidBodyMap.put(rigidBodies[i].getName(), rigidBodies[i]);
      return nameToRigidBodyMap;
   }

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, ArrayList<CollisionMeshDescription> meshDescriptions)
   {
      Vector3D centerOfMassOffset = new Vector3D();
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(), getCollisionMeshPoints(meshDescriptions, centerOfMassOffset));
   }

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, LinkDescription linkDescriptions)
   {
      Vector3D centerOfMassOffset = new Vector3D();
      linkDescriptions.getCenterOfMassOffset(centerOfMassOffset);
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(),
                                                           getCollisionMeshPoints(linkDescriptions.getCollisionMeshes(), centerOfMassOffset));
   }

   public List<Point3D> getCollisionMeshPoints(ArrayList<CollisionMeshDescription> meshDescriptions, Vector3D centerOfMassOffset)
   {
      ArrayList<ConvexShapeDescription> collisionShapeDescriptions = new ArrayList<>();
      for (int i = 0; i < meshDescriptions.size(); i++)
         meshDescriptions.get(i).getConvexShapeDescriptions(collisionShapeDescriptions);
      ArrayList<Point3D> pointsForRigidBody = new ArrayList<>();
      for (ConvexShapeDescription shapeDescription : collisionShapeDescriptions)
      {
         ArrayList<Point3D> pointsForShapeDescription = new ArrayList<>();
         getCollisionMeshPointsFromDescription(shapeDescription, pointsForShapeDescription);
         pointsForRigidBody.addAll(pointsForShapeDescription);
      }
      centerOfMassOffset.negate();
      ConvexPolytopeConstructor.shiftCentroid(centerOfMassOffset, pointsForRigidBody);
      return pointsForRigidBody;
   }

   public void getCollisionMeshPointsFromDescription(ConvexShapeDescription shapeDescription, List<Point3D> pointsToPack)
   {
      if (shapeDescription instanceof SphereDescriptionReadOnly)
      {
         if (debug)
            PrintTools.debug("Adding spherical mesh");
         RigidBodyTransform transform = new RigidBodyTransform();
         ((SphereDescriptionReadOnly) shapeDescription).getRigidBodyTransform(transform);
         ConvexPolytopeConstructor.getCollisionMeshPointsForSphere(transform, ((SphereDescriptionReadOnly) shapeDescription).getRadius(),
                                                                   defaultCurvedSurfaceDivisions, pointsToPack);
      }
      else if (shapeDescription instanceof CapsuleDescriptionReadOnly)
      {
         if (debug)
            PrintTools.debug("Adding capsule mesh");
         LineSegment3D lineSegment = new LineSegment3D();
         ((CapsuleDescriptionReadOnly) shapeDescription).getCapToCapLineSegment(lineSegment);
         ConvexPolytopeConstructor.getCollisionMeshPointsForCapsule(lineSegment, ((CapsuleDescriptionReadOnly) shapeDescription).getRadius(),
                                                                    defaultCurvedSurfaceDivisions, pointsToPack);
      }
      else if (shapeDescription instanceof CylinderDescriptionReadOnly)
      {
         if (debug)
            PrintTools.debug("Adding cylinderical mesh");
         RigidBodyTransform transform = new RigidBodyTransform();
         ((CylinderDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
         ConvexPolytopeConstructor.getCollisionMeshPointsForCylinder(transform, ((CylinderDescriptionReadOnly) shapeDescription).getRadius(),
                                                                     ((CylinderDescriptionReadOnly) shapeDescription).getHeight(),
                                                                     defaultCurvedSurfaceDivisions, pointsToPack);
      }
      else if (shapeDescription instanceof CubeDescriptionReadOnly)
      {
         if (debug)
            PrintTools.debug("Adding cube mesh");
         RigidBodyTransform transform = new RigidBodyTransform();
         ((CubeDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
         ConvexPolytopeConstructor.getCollisionMeshPointsForCuboid(transform, ((CubeDescriptionReadOnly) shapeDescription).getLengthX(),
                                                                   ((CubeDescriptionReadOnly) shapeDescription).getWidthY(),
                                                                   ((CubeDescriptionReadOnly) shapeDescription).getHeightZ(), pointsToPack);
      }
      else if (shapeDescription instanceof ConvexPolytopeDescriptionReadOnly)
      {
         if (debug)
            PrintTools.debug("Adding arbitrary mesh");
         ((ConvexPolytopeDescriptionReadOnly) shapeDescription).getConvexPolytope().getVertices(pointsToPack);
      }
      else
         throw new RuntimeException("Unhandled collision mesh description shape: " + shapeDescription.getClass());
   }

   public FrameConvexPolytope createCollisionMeshesFromDescription(ReferenceFrame referenceFrame, ConvexShapeDescription shapeDescription)
   {
      List<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsFromDescription(shapeDescription, points);
      return ConvexPolytopeConstructor.createFramePolytope(referenceFrame, points);
   }
}
