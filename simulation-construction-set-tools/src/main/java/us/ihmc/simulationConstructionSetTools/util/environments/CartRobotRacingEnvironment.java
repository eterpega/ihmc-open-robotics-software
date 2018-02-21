package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableCylinderRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CartRobotRacingEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableCylinderRobot> envRobots = new ArrayList<ContactableCylinderRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private double plateSize = 2.0;
   private double plateHeightGap = 0.3;
   private double plateThickness = 0.1;

   public CartRobotRacingEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      for (int i = 0; i < 5; i++)
      {
         combinedTerrainObject.addBox(-0.5 * plateSize + plateSize * i, -0.5 * plateSize, 0.5 * plateSize + plateSize * i, 0.5 * plateSize,
                                      -plateThickness - plateHeightGap * i, -plateHeightGap * i);
      }

      //      RigidBodyTransform robotTransform = new RigidBodyTransform();
      //      ContactableCylinderRobot envRobot = new ContactableCylinderRobot("cylinderRobot", robotTransform, 0.2, 1.0, 100);
      //      envRobots.add(envRobot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
      //      return envRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      //      ContactController contactController = new ContactController();
      //      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);
      //
      //      contactController.addContactables(envRobots);
      //      envRobots.get(0).setController(contactController);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {

   }

}
