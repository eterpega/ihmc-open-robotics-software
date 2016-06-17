package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.quadrupedRobotics.providers.QuadrupedCartesianWaypointInputProvider;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedForceBasedCartesianController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedCartesianController.class.getSimpleName());
   private final DoubleYoVariable robotTime;
   private final QuadrupedReferenceFrames referenceFrames;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   MultipleWaypointsPositionTrajectoryGenerator waypointsPositionTrajectoryGenerator;

   //   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   //   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   //
   //   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   //   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   //   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   //   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   //
   //   // Sole trajectories
   //   private final TimeInterval trajectoryTimeInterval = new TimeInterval();
   //   private final FramePoint finalSolePosition = new FramePoint();
   //   private final QuadrantDependentList<ThreeDoFMinimumJerkTrajectory> solePositionTrajectories = new QuadrantDependentList<>();
   //
   //   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;
   //
   // Task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedCartesianWaypointInputProvider inputProvider;

   public QuadrupedForceBasedCartesianController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedCartesianWaypointInputProvider inputProvider)
   {
      this.inputProvider = inputProvider;
      this.robotTime = environment.getRobotTimestamp();
      this.referenceFrames = controllerToolbox.getReferenceFrames();
      waypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("leftFront", referenceFrames.getBodyFrame(), registry);
      for (int i = 0; i < inputProvider.getNumberOfWaypoints(); i++)
      {
         waypointsPositionTrajectoryGenerator
               .appendWaypoint(inputProvider.getTimeAtWayPoint(i), inputProvider.getWaypointPosition(i), inputProvider.getWaypointVelocity(i));
      }
      // Feedback controller
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      //      // Task space controller
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();
      environment.getParentRegistry().addChild(registry);
   }

   @Override public void onEntry()
   {
      updateEstimates();
      waypointsPositionTrajectoryGenerator.initialize(); // ?do we initialize before appending?
   }

   @Override public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();

      return waypointsPositionTrajectoryGenerator.isDone() ? ControllerEvent.DONE : null;
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      double currentTime = robotTime.getDoubleValue(); //do we have to subtract start time?
      waypointsPositionTrajectoryGenerator.compute(currentTime);
      //query motion generator given current time stamp
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         //Todo: make waypoints come in a quadrant manner
         waypointsPositionTrajectoryGenerator.getPosition(solePositionControllerSetpoints.getSolePosition(quadrant));
         waypointsPositionTrajectoryGenerator.getVelocity(solePositionControllerSetpoints.getSoleLinearVelocity(quadrant));
      }
      solePositionController.compute(taskSpaceControllerCommands.getSoleForce(), solePositionControllerSetpoints, taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }
   private void updateGains()
   {
//      for (RobotQuadrant quadrant : RobotQuadrant.values)
//      {
//         solePositionController.getGains(quadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
//         solePositionController.getGains(quadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
//         solePositionController.getGains(quadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
//      }
//      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
//      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
//      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
   }
   @Override public void onExit()
   {
   }
}
