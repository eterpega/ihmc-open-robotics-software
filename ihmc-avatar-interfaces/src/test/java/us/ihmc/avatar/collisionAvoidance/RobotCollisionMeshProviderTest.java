package us.ihmc.avatar.collisionAvoidance;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;

import org.junit.Test;

import gnu.trove.map.hash.THashMap;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobots;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

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
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(collisionPolytopeMap.size(), true);
      for(RigidBody rigidBody : ScrewTools.computeRigidBodiesAfterThisJoint(robotModel.getRootJoint()))
      {
         if(collisionPolytopeMap.get(rigidBody) != null)
            viz.addPolytope(collisionPolytopeMap.get(rigidBody), Color.BLUE);
      }
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
      assertTrue(frameConvexPolytope.getReferenceFrame()  == rigidBody.getBodyFixedFrame());
      assertEquals(6, frameConvexPolytope.getNumberOfFaces());
      assertEquals(12, frameConvexPolytope.getNumberOfEdges());
      assertEquals(8, frameConvexPolytope.getNumberOfVertices());
   }
}
