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
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypoint;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

import us.ihmc.robotics.geometry.FramePoint;

public class QuadrupedForceBasedCartesianController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedCartesianController.class.getSimpleName());
   private final DoubleYoVariable robotTime;
   private final QuadrupedReferenceFrames referenceFrames;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   QuadrantDependentList<MultipleWaypointsPositionTrajectoryGenerator> quadrupedWaypointsPositionTrajectoryGenerator;

   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);

   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
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
   private final QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;
   private ArrayList<Point3d> positionWaypoints = new ArrayList<>();
   private ArrayList<Vector3d> velocityWaypoints = new ArrayList<>();
   private ArrayList<Double> timingWaypoints = new ArrayList<>();
   private QuadrupedSoleWaypoint quadrupedSoleWaypoint;

   public QuadrupedForceBasedCartesianController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      this.soleWaypointInputProvider = inputProvider;
      this.robotTime = environment.getRobotTimestamp();
      this.referenceFrames = controllerToolbox.getReferenceFrames();
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
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator tempWaypointGenerator = new MultipleWaypointsPositionTrajectoryGenerator(
               quadrant.getCamelCaseName() + "SoleTrajectory", referenceFrames.getBodyFrame(), registry);
         quadrupedWaypointsPositionTrajectoryGenerator.set(quadrant, tempWaypointGenerator);
         if (quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant) == null)
         {
            System.out.print("Not Initialized\n");
         }
      }

      environment.getParentRegistry().addChild(registry);

      //debugging variables
      quadrupedSoleWaypoint = new QuadrupedSoleWaypoint();
   }

   @Override public void onEntry()
   {
      updateEstimates();

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
      //      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      //      {
      //         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      //      }
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         //         Point3d startPoint = new Point3d(taskSpaceEstimates.getSolePosition(quadrant).getPoint());
         taskSpaceEstimates.getSolePosition(quadrant).changeFrame(referenceFrames.getBodyFrame());
         Point3d startPoint = taskSpaceEstimates.getSolePosition(quadrant).getPoint();
         positionWaypoints.clear();
         positionWaypoints.add(new Point3d(startPoint.getX(), startPoint.getY(), startPoint.getZ()));
         positionWaypoints.add(new Point3d(startPoint.getX(), startPoint.getY(), startPoint.getZ() + .1));
         positionWaypoints.add(new Point3d(startPoint.getX(), startPoint.getY(), startPoint.getZ()));
         positionWaypoints.add(new Point3d(startPoint.getX(), startPoint.getY(), startPoint.getZ() + .1));
         positionWaypoints.add(new Point3d(startPoint.getX(), startPoint.getY(), startPoint.getZ()));
         velocityWaypoints.clear();
         velocityWaypoints.add(new Vector3d(0, 0, 0));
         velocityWaypoints.add(new Vector3d(0, 0, 0));
         velocityWaypoints.add(new Vector3d(0, 0, 0));
         velocityWaypoints.add(new Vector3d(0, 0, 0));
         velocityWaypoints.add(new Vector3d(0, 0, 0));
         timingWaypoints.clear();
         timingWaypoints.add(0.0);
         timingWaypoints.add(5.0);
         timingWaypoints.add(10.0);
         timingWaypoints.add(15.0);
         timingWaypoints.add(20.0);
         quadrupedSoleWaypoint.quadrantSolePositionList.set(quadrant, positionWaypoints);
         quadrupedSoleWaypoint.quadrantSoleVelocityList.set(quadrant, velocityWaypoints);
         quadrupedSoleWaypoint.quadrantSoleTimingList.set(quadrant, timingWaypoints);
//         quadrupedWaypointsPositionTrajectoryGenerator.clear();
         for (int i = 0; i < quadrupedSoleWaypoint.quadrantSoleTimingList.get(RobotQuadrant.FRONT_LEFT).size(); i++)
         {
//            System.out.print(quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).get(i));
//            System.out.print(quadrupedSoleWaypoint.quadrantSolePositionList.get(quadrant).get(i));
//            System.out.print(quadrupedSoleWaypoint.quadrantSoleVelocityList.get(quadrant).get(i));
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).appendWaypoint(quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).get(i),
                  quadrupedSoleWaypoint.quadrantSolePositionList.get(quadrant).get(i), quadrupedSoleWaypoint.quadrantSoleVelocityList.get(quadrant).get(i));
         }
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).initialize();
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
//      System.out.print(solePositionControllerSetpoints.getSolePosition(RobotQuadrant.FRONT_LEFT).getPoint());
//      System.out.print("\n");
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
