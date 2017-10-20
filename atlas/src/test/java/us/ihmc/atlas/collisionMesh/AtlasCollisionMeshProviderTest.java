package us.ihmc.atlas.collisionMesh;

import org.junit.Test;

import gnu.trove.map.hash.THashMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.avatar.collisionAvoidance.RobotCollisionMeshProvider;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.PrintTools;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class AtlasCollisionMeshProviderTest
{
   @Test
   public void testAtlasCollisionMesh()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      RobotDescription atlasRobotDescription = atlasRobotModel.getRobotDescription();
      FullHumanoidRobotModel atlasFullRobotModel = atlasRobotModel.createFullRobotModel();
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(8);
      THashMap<RigidBody, FrameConvexPolytope> atlasCollisionMesh = meshProvider.createCollisionMeshesFromRobotDescription(atlasFullRobotModel, atlasRobotDescription);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(atlasCollisionMesh.size(), true, atlasRobotModel.createHumanoidFloatingRootJointRobot(false));
      for(RigidBody rigidBody : ScrewTools.computeRigidBodiesAfterThisJoint(atlasFullRobotModel.getRootJoint()))
      {
         if(atlasCollisionMesh.get(rigidBody) != null)
            viz.addPolytope(atlasCollisionMesh.get(rigidBody));
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      viz.update();
   }
}
