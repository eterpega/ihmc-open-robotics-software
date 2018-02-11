package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
         YoDouble robotTimeStamp, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         moveViaWaypointsStates.set(robotQuadrant, new QuadrupedMoveViaWaypointsState(robotQuadrant, bodyFrame, solePositionController.get(robotQuadrant),
                                                                                      robotTimeStamp, parentRegistry));
      }
   }

   public void handleWaypointList(QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointList)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).handleWaypointList(quadrupedSoleWaypointList.get(robotQuadrant));
   }

   public void initialize(YoPID3DGains positionControllerGains, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).initialize(positionControllerGains, useInitialSoleForceAsFeedforwardTerm);
   }

   public void updateEstimates(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         moveViaWaypointsStates.get(robotQuadrant).updateEstimates(taskSpaceEstimates);
   }

   public boolean compute(QuadrantDependentList<FrameVector3D> soleForceCommand)
   {
      boolean hasValue = false;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         hasValue = hasValue || moveViaWaypointsStates.get(robotQuadrant).compute(soleForceCommand.get(robotQuadrant));

      return hasValue;
   }
}
