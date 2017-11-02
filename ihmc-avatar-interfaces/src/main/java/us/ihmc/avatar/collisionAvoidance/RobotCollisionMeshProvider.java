package us.ihmc.avatar.collisionAvoidance;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.THashMap;
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
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RobotCollisionMeshProvider
{
   private final boolean debug = true;
   private final int defaultCurvedSurfaceDivisions;

   public RobotCollisionMeshProvider(int numberOfCurvedSurfaceDivisions)
   {
      this.defaultCurvedSurfaceDivisions = numberOfCurvedSurfaceDivisions;
   }

   public THashMap<RigidBody, FrameConvexPolytope> createCollisionMeshesFromDescription(RigidBody rootBody, JointDescription rootJointDescription)
   {
      Map<String, RigidBody> nameToRigidBodyMap = createNameBasedRigidBodyMap(rootBody);
      Map<String, LinkDescription> nameToLinkDescriptionMap = createNameToLinkDescriptionMap(rootJointDescription);
      THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap = new THashMap<>();
      String[] rigidBodyNames = (String[]) nameToRigidBodyMap.keySet().toArray();
      for (int i = 0; i < rigidBodyNames.length; i++)
      {
         RigidBody rigidBody = nameToRigidBodyMap.get(rigidBodyNames[i]);
         LinkDescription linkDescription = nameToLinkDescriptionMap.get(rigidBodyNames[i]);
         collisionMeshMap.put(rigidBody, createCollisionMesh(rigidBody, linkDescription.getCollisionMeshes()));
      }
      return collisionMeshMap;
   }

   private Map<String, LinkDescription> createNameToLinkDescriptionMap(JointDescription rootJointDescription)
   {
      Map<String, LinkDescription> nameToLinkDescriptionMap = new THashMap<>();
      recursivelyAddLinkeDescriptions(rootJointDescription, nameToLinkDescriptionMap);
      return nameToLinkDescriptionMap;
   }

   private void recursivelyAddLinkeDescriptions(JointDescription rootJointDescription, Map<String, LinkDescription> nameToLinkDescriptionMap)
   {
      nameToLinkDescriptionMap.put(rootJointDescription.getLink().getName(), rootJointDescription.getLink());
      for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
         recursivelyAddLinkeDescriptions(jointDescription, nameToLinkDescriptionMap);
   }

   private Map<String, RigidBody> createNameBasedRigidBodyMap(RigidBody rootBody)
   {
      Map<String, RigidBody> nameToRigidBodyMap = new THashMap<>();
      RigidBody[] rigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (int i = 0; i < rigidBodies.length; i++)
         nameToRigidBodyMap.put(rigidBodies[i].getName(), rigidBodies[i]);
      return nameToRigidBodyMap;
   }

   public THashMap<RigidBody, FrameConvexPolytope> createCollisionMeshesFromRobotDescription(FullRobotModel fullRobotModel, RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJointDescriptions = robotDescription.getRootJoints();
      if (rootJointDescriptions.size() > 1 || !(rootJointDescriptions.get(0) instanceof FloatingJointDescription))
         throw new RuntimeException("There should be only one floating joint");
      FloatingJointDescription rootJointDescription = (FloatingJointDescription) rootJointDescriptions.get(0);
      THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap = new THashMap<>();
      recursivelyAddCollisionMeshes(collisionMeshMap, rootJointDescription, fullRobotModel);
      PrintTools.debug("Done with mesh generation");
      return collisionMeshMap;
   }

   private void recursivelyAddCollisionMeshes(THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap, JointDescription jointDescription,
                                              FullRobotModel fullRobotModel)
   {
      if (!(jointDescription.getName() == fullRobotModel.getRootJoint().getName()))
      {
         LinkDescription linkDescription = jointDescription.getLink();
         InverseDynamicsJoint joint = fullRobotModel.getOneDoFJointByName(jointDescription.getName());
         RigidBody rigidBody = joint.getSuccessor();
         if (debug)
            PrintTools.debug("Link : " + linkDescription.getName() + " Joint: " + joint.getName());
         collisionMeshMap.put(rigidBody, createCollisionMesh(rigidBody, linkDescription));
      }
      else
      {
         LinkDescription linkDescription = jointDescription.getLink();
         InverseDynamicsJoint joint = fullRobotModel.getRootJoint();
         RigidBody rigidBody = joint.getSuccessor();
         if (debug)
            PrintTools.debug("Link : " + linkDescription.getName() + " Joint: " + joint.getName());
         collisionMeshMap.put(rigidBody, createCollisionMesh(rigidBody, linkDescription));
      }
      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         recursivelyAddCollisionMeshes(collisionMeshMap, childJointDescription, fullRobotModel);
      }
   }

   private final Vector3D centerOfMassOffset = new Vector3D();

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, ArrayList<CollisionMeshDescription> meshDescriptions)
   {
      centerOfMassOffset.setToZero();
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(), getCollisionMeshPoints(meshDescriptions, centerOfMassOffset));
   }

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, LinkDescription linkDescriptions)
   {
      linkDescriptions.getCenterOfMassOffset(centerOfMassOffset);
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(),
                                                           getCollisionMeshPoints(linkDescriptions.getCollisionMeshes(), centerOfMassOffset));
   }

   public ArrayList<Point3D> getCollisionMeshPoints(ArrayList<CollisionMeshDescription> meshDescriptions, Vector3D centerOfMassOffset)
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
      ArrayList<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsFromDescription(shapeDescription, points);
      return ConvexPolytopeConstructor.createFramePolytope(referenceFrame, points);
   }
}
