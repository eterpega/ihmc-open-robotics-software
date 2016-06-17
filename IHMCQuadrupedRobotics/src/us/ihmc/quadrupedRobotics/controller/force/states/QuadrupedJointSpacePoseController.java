package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class QuadrupedJointSpacePoseController implements QuadrupedController
{
   private final DoubleYoVariable robotTimestamp;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);

   //
   private final double hipRoll = 0;
   private final double hipPitch = 0;
   private final double kneePitch = 0;

   //Lay joint angles
   private final DoubleParameter hipPitchLayParameter = parameterFactory.createDouble("hipPitchLay", 1);
   private final DoubleParameter hipRollLayParameter = parameterFactory.createDouble("hipRollLay", 0.4);
   private final DoubleParameter kneePitchLayParameter = parameterFactory.createDouble("kneePitchLay", -2.2);

   //Sit joint angles
   private final DoubleParameter hipRollFrontSitParameter = parameterFactory.createDouble("hipRollFrontSit", 0.1);
   private final DoubleParameter hipRollHindSitParameter = parameterFactory.createDouble("hipRollHindSit", 0.0);
   private final DoubleParameter kneePitchFrontSitParameter = parameterFactory.createDouble("kneePitchFrontSit", -1.1);
   private final DoubleParameter kneePitchHindSitParameter = parameterFactory.createDouble("kneePitchHindSit", -2.7);
   private final DoubleParameter hipPitchFrontSitParameter = parameterFactory.createDouble("hipPitchFrontSit", -.55);
   private final DoubleParameter hipPitchHindSitParameter = parameterFactory.createDouble("hipPitchHindSit", -1.2);

   //Kneel joint angles
   private final DoubleParameter hipRollFrontKneelParameter = parameterFactory.createDouble("hipRollFrontKneel", 0.1);
   private final DoubleParameter hipRollHindKneelParameter = parameterFactory.createDouble("hipRollHindKneel", 0.0);
   private final DoubleParameter kneePitchFrontKneelParameter = parameterFactory.createDouble("kneePitchFrontKneel", -1.1);
   private final DoubleParameter kneePitchHindKneelParameter = parameterFactory.createDouble("kneePitchHindKneel", -2.2);
   private final DoubleParameter hipPitchFrontKneelParameter = parameterFactory.createDouble("hipPitchFrontKneel", -1.1);
   private final DoubleParameter hipPitchHindKneelParameter = parameterFactory.createDouble("hipPitchHindKneel", -.3);

   //Head joint angles
   //TODO: add joint head angles

   //Timing and PD gains
   private final DoubleParameter jointTrajectoryTotalTime = parameterFactory.createDouble("jointTrajectoryTotalTime", 1);
   private final DoubleParameter jointPGain = parameterFactory.createDouble("jointPGain", 1000);
   private final DoubleParameter jointDGain = parameterFactory.createDouble("jointDGain", 10);

   private final List<MinimumJerkTrajectory> trajectories;
   private final SDFFullQuadrupedRobotModel fullRobotModel;
   private final double controlDT;

   private double timeInFallTrajectory;
   private PDController pdController;
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private final List<Double> layJointAngles;
   private final List<Double> sitJointAngles;
   private final List<Double> kneelJointAngles;
   private final List<Double> headJointAngles;

   public enum jointSpaceRestPositions
   {
      SIT, LAY, KNEEL
   }

   private final EnumYoVariable<jointSpaceRestPositions> jointSpaceRestPosition = EnumYoVariable
         .create("jointSpaceRestPositions", jointSpaceRestPositions.class, registry);

   public QuadrupedJointSpacePoseController(QuadrupedRuntimeEnvironment runtimeEnvironment)
   {
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.trajectories = new ArrayList<>(fullRobotModel.getOneDoFJoints().length);
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         trajectories.add(new MinimumJerkTrajectory());
      }
      proportionalGain = new DoubleYoVariable("jointPGain", registry);
      derivativeGain = new DoubleYoVariable("jointDGain", registry);
      pdController = new PDController(proportionalGain, derivativeGain, "jointPDController", registry);
      layJointAngles = new ArrayList<>();
      sitJointAngles = new ArrayList<>();
      kneelJointAngles = new ArrayList<>();
      headJointAngles = new ArrayList<>();
      jointSpaceRestPosition.set(jointSpaceRestPositions.LAY);
      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   private void goToPosition()
   {
      timeInFallTrajectory += controlDT;
      fullRobotModel.updateFrames();
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         MinimumJerkTrajectory trajectory = trajectories.get(i);
         trajectory.computeTrajectory(timeInFallTrajectory);
         //PDController
         pdController.setProportionalGain(jointPGain.get());
         pdController.setDerivativeGain(jointDGain.get());
         double tau = pdController.compute(joint.getQ(), trajectory.getPosition(), joint.getQd(), trajectory.getVelocity());
         joint.setTau(tau);
      }
   }

   @Override public ControllerEvent process()
   {
      goToPosition();
      return null;
   }

   @Override public void onEntry()
   {
      layJointAngles.clear();
      sitJointAngles.clear();
      kneelJointAngles.clear();
      headJointAngles.clear() ;
      headJointAngles.addAll(Arrays.asList(0.0,0.0,0.0,0.0,0.0));
//      for (RobotQuadrant quadrant : RobotQuadrant.values)
//      {
//         if(quadrant.isQuadrantInFront()){
//            hipRoll = quadrant.getSide().negateIfRightSide(hipRollFrontParameter.get());
//            hipPitch = quadrant.getEnd().negateIfFrontEnd(hipRollFrontParameter.get());
//            kneePitch = quadrant.getEnd().negateIfHindEnd(hipRollFrontParameter.get());
//         }else{
//            hipRoll = quadrant.getSide().negateIfRightSide(hipRollHindParameter.get());
//            hipPitch = quadrant.getEnd().negateIfFrontEnd(hipRollHindParameter.get());
//            kneePitch = quadrant.getEnd().negateIfHindEnd(hipRollHindParameter.get());
//         }
//         sitJointAngles.addAll(Arrays.asList(hipRoll, hipPitch, kneePitch));
//      }
      layJointAngles.addAll(headJointAngles);
      sitJointAngles.addAll(headJointAngles);
      kneelJointAngles.addAll(headJointAngles);

      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         // Start the trajectory from the current pos/vel/acc.
         if (jointSpaceRestPosition.getEnumValue() == jointSpaceRestPositions.SIT)
         {
            trajectories.get(i).setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), sitJointAngles.get(i), 0.0, 0.0, jointTrajectoryTotalTime.get());
         }
         if (jointSpaceRestPosition.getEnumValue() == jointSpaceRestPositions.KNEEL)
         {
            trajectories.get(i).setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), kneelJointAngles.get(i),0.0,0.0, jointTrajectoryTotalTime.get());
         }
         if (jointSpaceRestPosition.getEnumValue() == jointSpaceRestPositions.LAY)
         {
            trajectories.get(i).setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), layJointAngles.get(i), 0.0, 0.0, jointTrajectoryTotalTime.get());
         }

      }
      timeInFallTrajectory = 0.0;
   }

   @Override public void onExit()
   {
   }

}
