package us.ihmc.avatar.collisionAvoidance;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import gnu.trove.map.hash.THashMap;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobots;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor.Axis;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class RobotCollisionMeshProviderTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testCollisionMeshCreationFromRobotDescription()
   {
      RobotDescription sevenDoFArm = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      JointNameMap sevenDoFArmJointNameMap = new KinematicsToolboxControllerTestRobots.SevenDoFArmJointMap();
      FullRobotModel robotModel = new FullRobotModelFromDescription(sevenDoFArm, sevenDoFArmJointNameMap, null);
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(4);
      THashMap<RigidBody, FrameConvexPolytope> collisionPolytopeMap = meshProvider.createCollisionMeshesFromRobotDescription(robotModel, sevenDoFArm);
      RobotFromDescription scsRobot = new RobotFromDescription(sevenDoFArm);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(collisionPolytopeMap.size(), true, scsRobot);
      for (RigidBody rigidBody : ScrewTools.computeRigidBodiesAfterThisJoint(robotModel.getRootJoint()))
      {
         if (collisionPolytopeMap.get(rigidBody) != null)
            viz.addPolytope(collisionPolytopeMap.get(rigidBody), Color.BLUE);
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      viz.updateNonBlocking();
      randomizeJointPositions(new Random(124815), robotModel, 0.75);
      new JointAnglesWriter(scsRobot, robotModel.getRootJoint(), robotModel.getOneDoFJoints()).updateRobotConfigurationBasedOnFullRobotModel();
      viz.update();
   }

   private void randomizeJointPositions(Random random, FullRobotModel randomizedFullRobotModel, double percentOfMotionRangeAllowed)
   {
      for (OneDoFJoint joint : randomizedFullRobotModel.getOneDoFJoints())
      {
         double jointLimitLower = joint.getJointLimitLower();
         if (Double.isInfinite(jointLimitLower))
            jointLimitLower = -Math.PI;
         double jointLimitUpper = joint.getJointLimitUpper();
         if (Double.isInfinite(jointLimitUpper))
            jointLimitUpper = -Math.PI;
         double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
         jointLimitLower += 0.5 * rangeReduction;
         jointLimitUpper -= 0.5 * rangeReduction;
         joint.setQ(RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper));
      }
   }

   @Test
   public void testMeshCollisionsWithObjects()
   {
      RobotDescription sevenDoFArm = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      JointNameMap sevenDoFArmJointNameMap = new KinematicsToolboxControllerTestRobots.SevenDoFArmJointMap();
      FullRobotModel robotModel = new FullRobotModelFromDescription(sevenDoFArm, sevenDoFArmJointNameMap, null);
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(4);
      THashMap<RigidBody, FrameConvexPolytope> collisionPolytopeMap = meshProvider.createCollisionMeshesFromRobotDescription(robotModel, sevenDoFArm);
      RobotFromDescription scsRobot = new RobotFromDescription(sevenDoFArm);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(collisionPolytopeMap.size() + 1, true, scsRobot);
      RigidBody[] rigidBodyList = ScrewTools.computeRigidBodiesAfterThisJoint(robotModel.getRootJoint());
      for (RigidBody rigidBody : rigidBodyList)
      {
         if (collisionPolytopeMap.get(rigidBody) != null)
            viz.addPolytope(collisionPolytopeMap.get(rigidBody));
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      viz.updateNonBlocking();
      Vector3D initialGuessDirection = new Vector3D(0.0, 0.0, 1.0);
      HybridGJKEPACollisionDetector collisionDetector = new HybridGJKEPACollisionDetector();
      FrameConvexPolytope obstacle = ConvexPolytopeConstructor.getFrameCapsuleCollisionMesh(new FramePoint3D(worldFrame, 0.0, 0.0, 0.5), Axis.X, 0.2, 0.05, 8);
      viz.addPolytope(obstacle);
      viz.updateNonBlocking();
      Vector3D collisionVector = new Vector3D();
      Vector3D collisionVector2 = new Vector3D();
      Point3D pointOnObstacle = new Point3D();
      Point3D pointOnRobot = new Point3D();
      for (RigidBody rigidBody : rigidBodyList)
      {
         FrameConvexPolytope rigidBodyMesh = collisionPolytopeMap.get(rigidBody);
         if (rigidBodyMesh != null && collisionDetector.checkCollisionBetweenTwoPolytopes(obstacle, rigidBodyMesh, initialGuessDirection))
         {
            PrintTools.debug("Colliding " + (rigidBody == null ? "null" : rigidBody.getName()) + " " + (rigidBodyMesh == null ? "null" : rigidBodyMesh.getNumberOfFaces()));
            viz.updateColor(rigidBodyMesh, Color.RED);
            collisionDetector.runEPAExpansion(obstacle, rigidBodyMesh, collisionVector);
            collisionDetector.getCollisionPoints(obstacle, rigidBodyMesh, pointOnObstacle, pointOnRobot);
            collisionVector2.sub(pointOnObstacle, pointOnRobot);
            PrintTools.debug(collisionVector2.toString() + " " + collisionVector.toString());
            viz.showCollisionVector(collisionVector);
         }
      }
      //randomizeJointPositions(new Random(124815), robotModel, 0.75);
      //new JointAnglesWriter(scsRobot, robotModel.getRootJoint(), robotModel.getOneDoFJoints()).updateRobotConfigurationBasedOnFullRobotModel();
      viz.update();
   }

   @Test
   public void testMeshCreationFromLinkDescription()
   {
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(10);
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBody rigidBody = new RigidBody(getClass().getName() + "RigidBody", transform, worldFrame);
      ArrayList<CollisionMeshDescription> collisionMeshDescriptionList = new ArrayList<>();
      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCubeReferencedAtCenter(1, 0.3, 0.6);
      collisionMeshDescriptionList.add(collisionMesh);
      FrameConvexPolytope frameConvexPolytope = meshProvider.createCollisionMesh(rigidBody, collisionMeshDescriptionList);
      assertTrue(frameConvexPolytope.getReferenceFrame() == rigidBody.getBodyFixedFrame());
      assertEquals(6, frameConvexPolytope.getNumberOfFaces());
      assertEquals(12, frameConvexPolytope.getNumberOfEdges());
      assertEquals(8, frameConvexPolytope.getNumberOfVertices());
   }
}
