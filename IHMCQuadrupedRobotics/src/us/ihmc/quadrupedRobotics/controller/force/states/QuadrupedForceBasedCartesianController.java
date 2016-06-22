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
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;


public class QuadrupedForceBasedCartesianController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedCartesianController.class.getSimpleName());
   private final DoubleYoVariable robotTime;
   private final QuadrupedReferenceFrames referenceFrames;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   QuadrantDependentList<MultipleWaypointsPositionTrajectoryGenerator> quadrupedWaypointsPositionTrajectoryGenerator;

   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);

   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

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
   private final QuadrupedSoleWaypointInputProvider inputProvider;

   public QuadrupedForceBasedCartesianController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      this.inputProvider = inputProvider;
      this.robotTime = environment.getRobotTimestamp();
      this.referenceFrames = controllerToolbox.getReferenceFrames();
      quadrupedWaypointsPositionTrajectoryGenerator = new QuadrantDependentList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         //Todo: is this how we initialize the waypoints within each quadrant
         quadrupedWaypointsPositionTrajectoryGenerator.set(quadrant, new MultipleWaypointsPositionTrajectoryGenerator(quadrant.getCamelCaseName(),
               referenceFrames.getBodyFrame(), registry));
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).initialize(); // ?do we initialize before appending?
      }
      // Feedback controller
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      // Task space controller
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
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < inputProvider.getNumberOfWaypoints(); i++)
         {
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant)
                  .appendWaypoint(inputProvider.getTimeAtWayPoint(quadrant, i), inputProvider.getWaypointPosition(quadrant, i),
                        inputProvider.getWaypointVelocity(quadrant, i));
         }
      }
      //Copy Pasted from stand prep
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.getGains(quadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(quadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(quadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }
      solePositionController.reset();

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();
   }

   @Override public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();

      return quadrupedWaypointsPositionTrajectoryGenerator.get(RobotQuadrant.FRONT_LEFT).isDone() ? ControllerEvent.DONE : null;
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      double currentTime = robotTime.getDoubleValue(); //do we have to subtract start time?

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).compute(currentTime);
         //query motion generator given current time stamp
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
//            taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
//            taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
   }

   @Override public void onExit()
   {
   }
}
