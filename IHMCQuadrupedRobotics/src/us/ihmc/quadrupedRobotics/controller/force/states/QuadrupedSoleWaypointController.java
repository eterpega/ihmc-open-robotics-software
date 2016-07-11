package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.BooleanParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;

public class QuadrupedSoleWaypointController implements QuadrupedController
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable robotTime;

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final BooleanParameter taskFailedParameter = parameterFactory.createBoolean("taskFailed", false);

   // SoleWaypoint variables
   private QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;
   QuadrantDependentList<MultipleWaypointsPositionTrajectoryGenerator> quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;

   // Task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrupedReferenceFrames referenceFrames;
   private double taskStartTime;

   public QuadrupedSoleWaypointController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      soleWaypointInputProvider = inputProvider;
      robotTime = environment.getRobotTimestamp();
      referenceFrames = controllerToolbox.getReferenceFrames();
      quadrupedWaypointsPositionTrajectoryGenerator = new QuadrantDependentList<>();

      // Feedback controller
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      // Task space controller
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // Create waypoint trajectory for each quadrant
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator tempWaypointGenerator = new MultipleWaypointsPositionTrajectoryGenerator(
               quadrant.getCamelCaseName() + "SoleTrajectory", referenceFrames.getBodyFrame(), registry);
         quadrupedWaypointsPositionTrajectoryGenerator.set(quadrant, tempWaypointGenerator);
      }
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      updateEstimates();
      taskStartTime = robotTime.getDoubleValue();
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      solePositionController.reset();

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();
      if (soleWaypointInputProvider.get().isValid())
      {
         createSoleWaypointTrajectory(soleWaypointInputProvider);
         taskFailedParameter.set(false);
      }
      else
      {
         taskFailedParameter.set(true);
      }
   }

   @Override
   public ControllerEvent process()
   {
      updateGains();
      updateEstimates();
      updateSetpoints();
      if (taskFailedParameter.get())
      {
         return ControllerEvent.FAIL;
      }
      else
      {
         return (robotTime.getDoubleValue() - taskStartTime > quadrupedWaypointsPositionTrajectoryGenerator.get(RobotQuadrant.FRONT_LEFT)
               .getLastWaypointTime()) ? ControllerEvent.DONE : null;
      }
   }

   @Override
   public void onExit()
   {
   }

   public void createSoleWaypointTrajectory(QuadrupedSoleWaypointInputProvider soleWaypointInputProvider)
   {
      if (soleWaypointInputProvider.get().isValid())
      {
         for (RobotQuadrant quadrant : RobotQuadrant.values)
         {
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).clear();
            for (int i = 0; i < soleWaypointInputProvider.get().size(quadrant); ++i)
            {
               quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).appendWaypoint(soleWaypointInputProvider.get().get(quadrant).get(i).getTime(),
                     soleWaypointInputProvider.get().get(quadrant).get(i).getPosition(), soleWaypointInputProvider.get().get(quadrant).get(i).getVelocity());
            }
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).initialize();
         }
      }
   }

   protected void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).compute(currentTrajectoryTime);
         //query motion generator at the current time stamp within the trajectory
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).getPosition(solePositionControllerSetpoints.getSolePosition(quadrant));
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).getVelocity(solePositionControllerSetpoints.getSoleLinearVelocity(quadrant));

      }
      solePositionController.compute(taskSpaceControllerCommands.getSoleForce(), solePositionControllerSetpoints, taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

   }

   private void updateGains()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.getGains(quadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(quadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(quadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
   }

   public QuadrupedTaskSpaceEstimator.Estimates getTaskSpaceEstimates()
   {
      return taskSpaceEstimates;
   }

   public QuadrupedSoleWaypointInputProvider getSoleWaypointInputProvider()
   {
      return soleWaypointInputProvider;
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }
}
