package us.ihmc.atlas.collisionMesh;

import java.awt.Color;
import java.util.Map;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.avatar.collisionAvoidance.RobotCollisionMeshProvider;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AtlasCollisionMeshProviderVisualizer
{
   public static void main(String[] args)
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      RobotDescription atlasRobotDescription = atlasRobotModel.getRobotDescription();
      FullHumanoidRobotModel atlasFullRobotModel = atlasRobotModel.createFullRobotModel();
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(8);
      Map<RigidBody, FrameConvexPolytope> atlasCollisionMesh = meshProvider.createCollisionMeshesFromRobotDescription(atlasFullRobotModel,
                                                                                                                      atlasRobotDescription);

      YoVariableRegistry registry = new YoVariableRegistry("PolytopeVisualizer");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(atlasCollisionMesh.size(), registry, graphicsListRegistry);

      float hue = 0.0f;

      for (FrameConvexPolytope polytope : atlasCollisionMesh.values())
      {
         Color color = Color.getHSBColor(hue, 1.0f, 1.0f);
         hue += 1.0 / atlasCollisionMesh.size();
         viz.addPolytope(polytope, color);
      }

      viz.update();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();

      SimulationConstructionSet scs = new SimulationConstructionSet(atlasRobotModel.createHumanoidFloatingRootJointRobot(false), parameters);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.getRootRegistry().addChild(registry);

      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(.5);
      scs.addStaticLinkGraphics(coordinateSystem);
      scs.setGroundVisible(false);
      scs.setDT(1.0, 1);
      scs.startOnAThread();

   }
}
