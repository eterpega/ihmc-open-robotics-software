package us.ihmc.valkyrieRosControl;

import java.util.List;

import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;

public class ValkyrieWristJointLimitsCalculator
{
   // Data that was taken directly by exploring the range of motion of the left wrist on Valkyrie A.
   private final double[] pitchReference = {-0.9, -0.8, -0.7, -0.60,  0.00,  0.60,  0.7};
   private final double[] rollMin        = { 0.0, -0.1, -0.2, -0.35, -0.35, -0.35, -0.2};
   private final double[] rollMax        = { 0.0,  0.1,  0.2,  0.35,  0.35,  0.35,  0.2};
   
   private final double[] rollReference = {-0.35, -0.2, -0.1,  0.0,  0.1,  0.2,  0.3,  0.45};
   private final double[] pitchMin      = {-0.60, -0.7, -0.8, -0.9, -0.8, -0.7, -0.6, -0.50};
   private final double[] pitchMax      = { 0.60,  0.7,  0.7,  0.7,  0.7,  0.7,  0.6,  0.50};

   private final SideDependentList<OneDoFJoint> wristRollJoints = new SideDependentList<>();
   private final SideDependentList<OneDoFJoint> wristPitchJoints = new SideDependentList<>();

   public ValkyrieWristJointLimitsCalculator(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, ValkyrieJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String wristRollName = jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL);
         OneDoFJoint wristRollJoint = yoEffortJointHandleHolders.stream().filter(h -> h.getName().equals(wristRollName)).findFirst().get().getOneDoFJoint();
         wristRollJoints.put(robotSide, wristRollJoint);

         String wristPitchName = jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH);
         OneDoFJoint wristPitchJoint = yoEffortJointHandleHolders.stream().filter(h -> h.getName().equals(wristPitchName)).findFirst().get().getOneDoFJoint();
         wristPitchJoints.put(robotSide, wristPitchJoint);
      }
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint pitchJoint = wristPitchJoints.get(robotSide);
         OneDoFJoint rollJoint = wristRollJoints.get(robotSide);

         estimateLimits(robotSide, pitchJoint, rollJoint, pitchReference, rollMin, rollMax);
         estimateLimits(robotSide, rollJoint, pitchJoint, rollReference, pitchMin, pitchMax);
      }
   }

   private void estimateLimits(RobotSide robotSide, OneDoFJoint referenceJoint, OneDoFJoint jointToUpdateLimits, double[] reference, double[] min, double[] max)
   {
      double qRef = robotSide.negateIfRightSide(referenceJoint.getQ());

      int index = 0;
      for (; index < reference.length - 1; index++)
      {
         if (qRef < reference[index + 1])
            break;
      }

      double alpha = (qRef - reference[index]) / (reference[index + 1] - reference[index]);
      double lowerLimit = TupleTools.interpolate(min[index], min[index + 1], alpha);
      double upperLimit = TupleTools.interpolate(max[index], max[index + 1], alpha);

      jointToUpdateLimits.setJointLimitLower(robotSide.negateIfRightSide(lowerLimit));
      jointToUpdateLimits.setJointLimitUpper(robotSide.negateIfRightSide(upperLimit));
   }
}
