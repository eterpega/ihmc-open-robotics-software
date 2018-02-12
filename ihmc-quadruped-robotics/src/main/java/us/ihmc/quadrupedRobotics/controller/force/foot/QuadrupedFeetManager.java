package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedFeetManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<QuadrupedFootControlModule> footControlModules;

   public QuadrupedFeetManager(QuadrupedFootControlModuleParameters parameters, ReferenceFrame bodyFrame,
                               QuadrantDependentList<QuadrupedSolePositionController> solePositionController,
                               YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      footControlModules = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
        footControlModules.set(robotQuadrant,
                                new QuadrupedFootControlModule(parameters, robotQuadrant, bodyFrame, solePositionController.get(robotQuadrant), timestamp, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).attachStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists,
                                            QuadrupedTaskSpaceEstimates taskSpaceEstimates, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSoleWaypointList waypointList = quadrupedSoleWaypointLists.get(robotQuadrant);
         QuadrupedFootControlModule footControlModule = footControlModules.get(robotQuadrant);
         if (waypointList.size() > 0)
         {
            footControlModule.requestMoveViaWaypoints();
            footControlModule.initializeWaypointTrajectory(waypointList, taskSpaceEstimates, useInitialSoleForceAsFeedforwardTerm);
         }
         else
         {
            footControlModule.requestSupport();
         }
      }
   }

   public void triggerStep(RobotQuadrant robotQuadrant, QuadrupedTimedStep stepSequence)
   {
      footControlModules.get(robotQuadrant).triggerStep(stepSequence);
   }

   public void adjustStep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly adjustedStepGoalPosition)
   {
      footControlModules.get(robotQuadrant).adjustStep(adjustedStepGoalPosition);
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).reset();
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerStepTransitionCallback(stepTransitionCallback);
   }

   public void registerWaypointCallback(QuadrupedWaypointCallback waypointCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerWaypointCallback(waypointCallback);
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getContactState();
   }

   public void compute(QuadrantDependentList<FrameVector3D> soleForcesToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         compute(soleForcesToPack.get(quadrant), taskSpaceEstimates, quadrant);
   }

   public void compute(FrameVector3D soleForceToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates, RobotQuadrant robotQuadrant)
   {
      footControlModules.get(robotQuadrant).compute(soleForceToPack, taskSpaceEstimates);
   }

   public void setFullContact()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).requestSupport();
   }

}
