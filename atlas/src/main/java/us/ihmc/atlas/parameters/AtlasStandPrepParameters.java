package us.ihmc.atlas.parameters;

import java.util.HashMap;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.ros.AtlasOrderedJointMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasStandPrepParameters implements WholeBodySetpointParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final AtlasJointMap jointMap;

   public AtlasStandPrepParameters(AtlasJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
   }

   private void useDefaultAngles()
   {
      boolean batteryMassSim = AtlasRobotModel.BATTERY_MASS_SIMULATOR_IN_ROBOT;

      setSetpoint(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);

      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), batteryMassSim ? 0.13 : 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), robotSide.negateIfRightSide(0.09));
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.933);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), batteryMassSim ? 1.75 : 1.65);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), batteryMassSim ? -0.77 : -0.75);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), -robotSide.negateIfRightSide(0.09));

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.3));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 2.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(0.5));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.01);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), 0.0);
      }
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(int jointIndex)
   {
      String jointName = AtlasOrderedJointMap.jointNames[jointIndex];
      return getSetpoint(jointName);
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if (setPoints.containsKey(jointName))
         return setPoints.get(jointName);
      else
         return 0.0;
   }
}
