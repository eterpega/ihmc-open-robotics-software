package us.ihmc.avatar.collisionAvoidance;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Map;
import java.util.Random;

import org.apache.commons.lang3.tuple.Pair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobots;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class RobotCollisionMeshProviderTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testCollisionMeshCreationFromRobotDescription()
   {
      KinematicsToolboxControllerTestRobots.SevenDoFArm sevenDoFArm = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> inverseDynamicsRobot = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(sevenDoFArm);
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(4);
      RigidBody rootBody = ScrewTools.getRootBody(inverseDynamicsRobot.getRight()[0].getSuccessor());
      Map<RigidBody, FrameConvexPolytope> collisionPolytopeMap = meshProvider.createCollisionMeshesFromRobotDescription(rootBody,
                                                                                                                        sevenDoFArm.getRootJoints().get(0));
      RobotFromDescription scsRobot = new RobotFromDescription(sevenDoFArm);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(collisionPolytopeMap.size(), true, scsRobot);
      for (RigidBody rigidBody : ScrewTools.computeRigidBodiesAfterThisJoint(rootBody.getChildrenJoints().get(0)))
      {
         if (collisionPolytopeMap.get(rigidBody) != null)
            viz.addPolytope(collisionPolytopeMap.get(rigidBody), Color.BLUE);
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      viz.updateNonBlocking();
      randomizeJointPositions(new Random(124815), inverseDynamicsRobot, 0.75);
      new JointAnglesWriter(scsRobot, inverseDynamicsRobot.getKey(), inverseDynamicsRobot.getValue()).updateRobotConfigurationBasedOnFullRobotModel();
      viz.update();
   }

   private void randomizeJointPositions(Random random, Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> randomizedFullRobotModel,
                                        double percentOfMotionRangeAllowed)
   {
      for (OneDoFJoint joint : randomizedFullRobotModel.getRight())
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
      PoseReferenceFrame collisionPointReferenceFrame = new PoseReferenceFrame("CollisionPointFrame", new FramePose());
      KinematicsToolboxControllerTestRobots.SevenDoFArm sevenDoFArm = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> inverseDynamicsRobot = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(sevenDoFArm);
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(4);
      RigidBody rootBody = ScrewTools.getRootBody(inverseDynamicsRobot.getRight()[0].getSuccessor());
      Map<RigidBody, FrameConvexPolytope> collisionPolytopeMap = meshProvider.createCollisionMeshesFromRobotDescription(rootBody,
                                                                                                                        sevenDoFArm.getRootJoints().get(0));
      RobotFromDescription scsRobot = new RobotFromDescription(sevenDoFArm);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(collisionPolytopeMap.size() + 2, true, scsRobot);
      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
      RigidBody[] rigidBodyList = ScrewTools.computeRigidBodiesAfterThisJoint(rootBody.getChildrenJoints().get(0));
      DenseMatrix64F jacobianMatrix = new DenseMatrix64F(6, 7);
      DenseMatrix64F jacobianTranspose = new DenseMatrix64F(7, 6);
      DenseMatrix64F xDot = new DenseMatrix64F(6, 1);
      DenseMatrix64F qDot = new DenseMatrix64F(7, 1);
      for (RigidBody rigidBody : rigidBodyList)
      {
         if (collisionPolytopeMap.get(rigidBody) != null)
            viz.addPolytope(collisionPolytopeMap.get(rigidBody));
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      Vector3D initialGuessDirection = new Vector3D(0.0, 0.0, 1.0);
      HybridGJKEPACollisionDetector collisionDetector = new HybridGJKEPACollisionDetector(Epsilons.ONE_TRILLIONTH);
      FrameConvexPolytope obstacle = ConvexPolytopeConstructor.getFrameCapsuleCollisionMesh(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), Axis.Z, 0.2, 0.05, 8);
      viz.addPolytope(obstacle);
      viz.updateNonBlocking();
      Vector3D collisionVector = new Vector3D(1.0, 0.0, 0.0);
      Point3D pointOnObstacle = new Point3D();
      Point3D pointOnRobot = new Point3D();
      boolean wasColliding = true;
      int count = 0;
      ExecutionTimer timer = new ExecutionTimer("Timer", null);
      timer.startMeasurement();
      while (wasColliding && (count++ < 1000))
      {
         wasColliding = false;
         for (RigidBody rigidBody : rigidBodyList)
         {
            FrameConvexPolytope rigidBodyMesh = collisionPolytopeMap.get(rigidBody);
            collisionDetector.setPolytopeA(obstacle);
            collisionDetector.setPolytopeB(rigidBodyMesh);
            collisionDetector.setSimplex(new ExtendedSimplexPolytope());
            if (rigidBodyMesh != null && collisionDetector.checkCollision())
            {
               wasColliding = true;
               //PrintTools.debug("Colliding " + (rigidBody == null ? "null" : rigidBody.getName()) + " " + (rigidBodyMesh == null ? "null" : rigidBodyMesh.getNumberOfFaces()));
               //viz.updateColor(rigidBodyMesh, Color.RED);
               collisionDetector.runEPAExpansion();
               collisionDetector.getCollisionPoints(pointOnObstacle, pointOnRobot);
               collisionVector.sub(pointOnObstacle, pointOnRobot);
               if (norm(collisionVector) < Epsilons.ONE_THOUSANDTH)
               {
                  collisionVector.normalize();
                  collisionVector.scale(Epsilons.ONE_THOUSANDTH);
               }
               //viz.showCollisionVector(pointOnRobot, pointOnObstacle);
               //viz.updateNonBlocking();
               InverseDynamicsJoint[] controllableJoints = ScrewTools.computeSubtreeJoints(rootBody);
               updateCollisionFrameFromPoint(collisionPointReferenceFrame, pointOnRobot, collisionVector);
               jacobianCalculator.clearJacobianMatrix();
               jacobianCalculator.setJacobianFrame(collisionPointReferenceFrame);
               jacobianCalculator.setKinematicChain(controllableJoints);
               jacobianCalculator.computeJacobianMatrix();
               jacobianCalculator.getJacobianMatrix(jacobianMatrix);
               //CommonOps.pinv(jacobianMatrix, jacobianInverse);
               CommonOps.transpose(jacobianMatrix, jacobianTranspose);
               for (int i = 0; i < 3; i++)
                  xDot.set(i + 3, 0, collisionVector.getElement(i));
               CommonOps.mult(jacobianTranspose, xDot, qDot);
               double scale = 5;
               for (int i = 0; i < controllableJoints.length; i++)
               {
                  OneDoFJoint oneDoFJoint = (OneDoFJoint) controllableJoints[i];
                  oneDoFJoint.setQ(oneDoFJoint.getQ() + qDot.get(i, 0) * scale);
               }
               new JointAnglesWriter(scsRobot, inverseDynamicsRobot.getKey(), inverseDynamicsRobot.getValue()).updateRobotConfigurationBasedOnFullRobotModel();
               //viz.updateNonBlocking();
            }
         }
         //if(!wasColliding)
         //   PrintTools.debug("No collisions!!");
      }
      timer.stopMeasurement();
      PrintTools.debug("Took: " + timer.getCurrentTime());
      viz.update();
   }

   private final Quaternion worldOrientation = new Quaternion();

   private void updateCollisionFrameFromPoint(PoseReferenceFrame poseFrame, Point3D collisionPoint, Vector3D collisionVector)
   {
      //TODO: Generate the frame axis using Gram-Schmidt orthogonalization on collision vector 

      // Set the pose 
      poseFrame.setPoseAndUpdate(collisionPoint, worldOrientation);
   }

   private double norm(Vector3DReadOnly vector)
   {
      return vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ();
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
