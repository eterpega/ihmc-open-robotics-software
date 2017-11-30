package us.ihmc.atlas.collisionMesh;

import static org.junit.Assert.assertFalse;

import java.util.Map;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.collisionAvoidance.RobotCollisionMeshProvider;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class HardCollisionDetectionTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testAtlasFootIssue()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      FullHumanoidRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();
      RobotCollisionMeshProvider provider = new RobotCollisionMeshProvider(4);
      Map<RigidBody, FrameConvexPolytope> meshMap = provider.createCollisionMeshesFromRobotDescription(fullRobotModel, atlasRobotModel.getRobotDescription());

      RigidBody[] rigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(fullRobotModel.getRootJoint().getSuccessor());

      FrameConvexPolytope polytopeA = meshMap.get(ScrewTools.findRigidBodiesWithNames(rigidBodies, "r_lleg")[0]);
      RigidBodyTransform transform = new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.45, -0.4, 0.5));
      FrameConvexPolytope polytopeB = ConvexPolytopeConstructor.getFrameSphericalCollisionMeshByProjectingCube(worldFrame, transform, 0.1, 4);
      ExtendedSimplexPolytope simplex = new ExtendedSimplexPolytope();
      //PolytopeVisualizationHelper viz = new PolytopeVisualizationHelper(3, true);
      //viz.addPolytope(polytopeA);
      //viz.addPolytope(polytopeB);
      //viz.addPolytope(simplex.getPolytope());
      HybridGJKEPACollisionDetector collisionDetector = new HybridGJKEPACollisionDetector();
      PrintTools.debug("Setting up detector");
      collisionDetector.setSimplex(simplex);
      collisionDetector.setPolytopeA(polytopeA);
      collisionDetector.setPolytopeB(polytopeB);
      PrintTools.debug("Now checking collisions");
      assertFalse(collisionDetector.checkCollision());
      PrintTools.debug("Done checking collisions");
      //viz.update();
   }
}
