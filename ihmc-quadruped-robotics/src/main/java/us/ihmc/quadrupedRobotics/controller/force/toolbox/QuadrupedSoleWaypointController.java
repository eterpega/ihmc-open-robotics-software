package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.foot.QuadrupedFootControlModule;
import us.ihmc.quadrupedRobotics.controller.force.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.controller.force.foot.QuadrupedMoveViaWaypointsState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSoleWaypointController
{
   // Yo variables

   private final QuadrantDependentList<QuadrupedMoveViaWaypointsState> moveViaWaypointsStates = new QuadrantDependentList<>();

   public QuadrupedSoleWaypointController(ReferenceFrame bodyFrame, QuadrantDependentList<QuadrupedSolePositionController> solePositionController,
                                          YoDouble robotTimeStamp, QuadrupedFootControlModuleParameters parameters, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         moveViaWaypointsStates.set(robotQuadrant, new QuadrupedMoveViaWaypointsState(robotQuadrant, bodyFrame, solePositionController.get(robotQuadrant),
                                                                                      robotTimeStamp, parameters, parentRegistry));
      }
   }

   public void handleWaypointList(QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointList)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).handleWaypointList(quadrupedSoleWaypointList.get(robotQuadrant));
   }

   public void initialize(boolean useInitialSoleForceAsFeedforwardTerm)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).initialize(useInitialSoleForceAsFeedforwardTerm);
   }

   public void updateEstimates(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).updateEstimates(taskSpaceEstimates);
   }

   public boolean compute(QuadrantDependentList<FrameVector3D> soleForceCommand)
   {
      boolean done = true;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedFootControlModule.FootEvent event = moveViaWaypointsStates.get(robotQuadrant).process();
         soleForceCommand.get(robotQuadrant).set(moveViaWaypointsStates.get(robotQuadrant).getSoleForceCommand());
         if (event != QuadrupedFootControlModule.FootEvent.TIMEOUT)
            done = false;
      }

      return done;
   }
}
