package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.virtualHoist.VirtualHoist;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieHangingSimulation
{
   public ValkyrieHangingSimulation()
   {
      DRCRobotModel robotModel = new ValkyrieRobotModelWithHoist(RobotTarget.SCS, false);
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);

      simulationStarter.addHighLevelControllerConfiguration(this::configureControllerFactory);

      simulationStarter.startSimulation(null, true);
   }

   private void configureControllerFactory(HighLevelHumanoidControllerFactory factory)
   {
      factory.useDefaultStandPrepControlState();
      factory.setInitialState(HighLevelControllerName.STAND_PREP_STATE);
   }

   private class ValkyrieRobotModelWithHoist extends ValkyrieRobotModel
   {
      public ValkyrieRobotModelWithHoist(RobotTarget target, boolean headless)
      {
         super(target, headless);
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         HumanoidFloatingRootJointRobot robot = super.createHumanoidFloatingRootJointRobot(createCollisionMeshes);

         Joint joint = robot.getJoint("torsoRoll");

         ArrayList<Vector3D> attachmentLocations = new ArrayList<Vector3D>();

         attachmentLocations.add(new Vector3D(0.0, 0.15, 0.412));
         attachmentLocations.add(new Vector3D(0.0, -0.15, 0.412));

         double updateDT = 0.0001;
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);

         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3D(0.0, 0.0, 2.5));
         virtualHoist.setHoistStiffness(20000.0);
         virtualHoist.setHoistDamping(5000.0);

         return robot;
      }
   }

   public static void main(String[] args)
   {
      new ValkyrieHangingSimulation();
   }
}
