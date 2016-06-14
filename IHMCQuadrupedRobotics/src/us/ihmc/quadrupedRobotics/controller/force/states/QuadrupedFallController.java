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
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class QuadrupedFallController implements QuadrupedController
{
   private final DoubleYoVariable robotTimestamp;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter hipPitchHomeParameter = parameterFactory.createDouble("hipPitchHome", 1);
   private final DoubleParameter hipRollHomeParameter = parameterFactory.createDouble("hipRollHome", 0.4);
   private final DoubleParameter kneePitchHomeParameter = parameterFactory.createDouble("kneePitchHome", -2.2);

   private final DoubleParameter kneePitchFrontSitParameter = parameterFactory.createDouble("kneePitchFrontSit", -1.1);
   private final DoubleParameter kneePitchHindSitParameter = parameterFactory.createDouble("kneePitchHindSit", -2.2);

   private final DoubleParameter fallTrajectoryTotalTime = parameterFactory.createDouble("fallTrajectoryTotalTime", 1);
   private final DoubleParameter fallPGain = parameterFactory.createDouble("fallPGain", 1000);
   private final DoubleParameter fallDGain = parameterFactory.createDouble("fallDGain", 10);

   // fall detection trajectories
   private final List<MinimumJerkTrajectory> trajectories;
   private final SDFFullQuadrupedRobotModel fullRobotModel;
   private final double controlDT;

   private double timeInFallTrajectory;
   private PDController pdController;
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private final List<Double> homePositions;
   private final List<Double> sitPositions;

   private final IntegerYoVariable fallBehavior;

   public QuadrupedFallController(QuadrupedRuntimeEnvironment runtimeEnvironment)
   {
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();

      // fall detection
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.trajectories = new ArrayList<>(fullRobotModel.getOneDoFJoints().length);
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         trajectories.add(new MinimumJerkTrajectory());
      }
      proportionalGain = new DoubleYoVariable("fallPGain", registry);
      derivativeGain = new DoubleYoVariable("fallDGain", registry);
      pdController = new PDController(proportionalGain, derivativeGain, "fallPDController", registry);

      fallBehavior = new  IntegerYoVariable("fallBehavior",registry);
      homePositions = new ArrayList<>();
      sitPositions = new ArrayList<>();
      // frames
      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   private void goToHomePosition()
   {
      timeInFallTrajectory += controlDT;
      fullRobotModel.updateFrames();
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         MinimumJerkTrajectory trajectory = trajectories.get(i);
         trajectory.computeTrajectory(timeInFallTrajectory);
         //PDController
         pdController.setProportionalGain(fallPGain.get());
         pdController.setDerivativeGain(fallDGain.get());
         double tau = pdController.compute(joint.getQ(), trajectory.getPosition(), joint.getQd(), trajectory.getVelocity());
         joint.setTau(tau);
      }
   }

   @Override public ControllerEvent process()
   {
      goToHomePosition();
      return null;
   }

   @Override public void onEntry()
   {
      homePositions.clear();
      homePositions.addAll(Arrays.asList(hipRollHomeParameter.get(), hipPitchHomeParameter.get(), kneePitchHomeParameter.get(),
            -hipRollHomeParameter.get(), hipPitchHomeParameter.get(), kneePitchHomeParameter.get(),
            hipRollHomeParameter.get(), -hipPitchHomeParameter.get(), -kneePitchHomeParameter.get(),
            -hipRollHomeParameter.get(), -hipPitchHomeParameter.get(), -kneePitchHomeParameter.get(),
            0.0, 0.0, 0.0, 0.0, 0.0));
      sitPositions.clear();
      sitPositions.addAll(Arrays.asList(hipRollHomeParameter.get(),-.5*kneePitchHindSitParameter.get(), kneePitchFrontSitParameter.get(),
            -hipRollHomeParameter.get(), -.5*kneePitchHindSitParameter.get(), kneePitchFrontSitParameter.get(),
            hipRollHomeParameter.get(), .5*kneePitchHindSitParameter.get(), -kneePitchHindSitParameter.get(),
            -hipRollHomeParameter.get(), .5*kneePitchHindSitParameter.get(), -kneePitchHindSitParameter.get(),
            0.0, 0.0, 0.0, 0.0, 0.0));
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         // Start the trajectory from the current pos/vel/acc.
//         trajectories.get(i).setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), homePositions.get(i), 0.0, 0.0, //deisred q, qd, qdd
//               fallTrajectoryTotalTime.get());
         trajectories.get(i).setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), sitPositions.get(i), 0.0, 0.0, //deisred q, qd, qdd
               fallTrajectoryTotalTime.get());
      }
      timeInFallTrajectory = 0.0;
   }

   @Override public void onExit()
   {
   }

}
