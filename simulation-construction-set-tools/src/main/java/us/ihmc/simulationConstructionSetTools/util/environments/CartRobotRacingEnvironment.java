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

   public CartRobotRacingEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      
      combinedTerrainObject.addBox(-2.0, -1.0, 2.0, 1.0, -3.6, -3.5);
      
      
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
