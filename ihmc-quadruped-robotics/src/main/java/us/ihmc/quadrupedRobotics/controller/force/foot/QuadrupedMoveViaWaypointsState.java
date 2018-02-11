package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState
{
   // Yo variables
   private final YoVariableRegistry registry;
   private final YoDouble robotTime;

   // SoleWaypoint variables
   private MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;
   private final FrameVector3D initialSoleForces;

   private final ReferenceFrame bodyFrame;
   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;
   private double taskStartTime;

   private final RobotQuadrant robotQuadrant;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, ReferenceFrame bodyFrame, QuadrupedSolePositionController solePositionController,
                                         YoDouble robotTimeStamp, YoVariableRegistry parentRegistry)
   {
      this.robotQuadrant = robotQuadrant;
      this.bodyFrame = bodyFrame;
      robotTime = robotTimeStamp;
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();

      registry = new YoVariableRegistry(robotQuadrant.getShortName() + getClass().getSimpleName());
      
      // Feedback controller
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);
      initialSoleForces = new FrameVector3D();

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       bodyFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void handleWaypointList(QuadrupedSoleWaypointList quadrupedSoleWaypointList)
   {
      this.quadrupedSoleWaypointList = quadrupedSoleWaypointList;
   }

   public void initialize(YoPID3DGains positionControllerGains, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      solePositionController.reset();
      if (useInitialSoleForceAsFeedforwardTerm)
      {
         this.initialSoleForces.setIncludingFrame(taskSpaceEstimates.getSoleVirtualForce(robotQuadrant));
         this.initialSoleForces.changeFrame(bodyFrame);
      }
      else
      {
         this.initialSoleForces.setToZero(bodyFrame);
      }
      updateGains(positionControllerGains);
      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   public void updateEstimates(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      this.taskSpaceEstimates.set(taskSpaceEstimates);
   }

   public boolean compute(FrameVector3D soleForceCommandToPack)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
      {
         soleForceCommandToPack.setToZero();
         return false;
      }
      else
      {
         quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
         quadrupedWaypointsPositionTrajectoryGenerator.getPosition(solePositionControllerSetpoints.getSolePosition());
         solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
         solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
         solePositionController.compute(soleForceCommandToPack, solePositionControllerSetpoints, taskSpaceEstimates);
         return true;
      }
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (int i = 0; i < quadrupedSoleWaypointList.size(); ++i)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoint(quadrupedSoleWaypointList.get(i).getTime(), quadrupedSoleWaypointList.get(i).getPosition(),
                     quadrupedSoleWaypointList.get(i).getVelocity());
      }
      if (quadrupedSoleWaypointList.size() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   private void updateGains(YoPID3DGains positionControllerGains)
   {
      solePositionController.getGains().set(positionControllerGains);
   }

}
