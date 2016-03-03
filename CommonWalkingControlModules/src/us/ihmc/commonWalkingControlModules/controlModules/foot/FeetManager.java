package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class FeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final WalkOnTheEdgesManager walkOnTheEdgesManager;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSwingLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSwingLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLeg",
         registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLegLocking = new DoubleYoVariable(
         "singularityEscapeNullspaceMultiplierSupportLegLocking", registry);

   // TODO Needs to be cleaned up someday... (Sylvain)
   public FeetManager(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      double singularityEscapeMultiplierForSwing = walkingControllerParameters.getSwingSingularityEscapeMultiplier();
      singularityEscapeNullspaceMultiplierSwingLeg.set(singularityEscapeMultiplierForSwing);
      singularityEscapeNullspaceMultiplierSupportLeg.set(walkingControllerParameters.getSupportSingularityEscapeMultiplier());
      singularityEscapeNullspaceMultiplierSupportLegLocking.set(0.0); // -0.5);

      feet = momentumBasedController.getContactableFeet();
      walkOnTheEdgesManager = new WalkOnTheEdgesManager(momentumBasedController, walkingControllerParameters, feet, footControlModules, registry);

      this.footSwitches = momentumBasedController.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      YoSE3PIDGainsInterface swingFootControlGains = walkingControllerParameters.createSwingFootControlGains(registry);
      YoSE3PIDGainsInterface holdPositionFootControlGains = walkingControllerParameters.createHoldPositionFootControlGains(registry);
      YoSE3PIDGainsInterface toeOffFootControlGains = walkingControllerParameters.createToeOffFootControlGains(registry);
      YoSE3PIDGainsInterface edgeTouchdownFootControlGains = walkingControllerParameters.createEdgeTouchdownFootControlGains(registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide, walkingControllerParameters, swingFootControlGains,
               holdPositionFootControlGains, toeOffFootControlGains, edgeTouchdownFootControlGains, momentumBasedController, registry);
         footControlModule.setNullspaceMultiplier(singularityEscapeMultiplierForSwing);

         footControlModules.put(robotSide, footControlModule);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGround(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      if (!footstep.getTrustHeight())
      {
         FramePoint supportAnklePosition = new FramePoint(ankleZUpFrames.get(upcomingSwingSide.getOppositeSide()));
         supportAnklePosition.changeFrame(footstep.getParentFrame());
         double newHeight = supportAnklePosition.getZ();
         footstep.setZ(newHeight);
      }

      FootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstep, swingTime);
      setContactStateForSwing(upcomingSwingSide);
   }

   public void handleFootTrajectoryMessage(ModifiableFootTrajectoryMessage footTrajectoryMessage)
   {
      RobotSide robotSide = footTrajectoryMessage.getRobotSide();
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setFootTrajectoryMessage(footTrajectoryMessage);

      if (footControlModule.getCurrentConstraintType() == ConstraintType.MOVE_VIA_WAYPOINTS)
         footControlModule.resetCurrentState();
      else
         setContactStateForMoveViaWaypoints(robotSide);
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void replanSwingTrajectory(RobotSide swingSide, Footstep footstep, double swingTime)
   {
      footControlModules.get(swingSide).replanTrajectory(footstep, swingTime);
   }

   public void requestMoveStraightTouchdownForDisturbanceRecovery(RobotSide swingSide)
   {
      footControlModules.get(swingSide).requestTouchdownForDisturbanceRecovery();
   }

   public void handleStopAllTrajectoryMessage(ModifiableStopAllTrajectoryMessage message)
   {
      if (!message.isStopAllTrajectory())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.requestStopTrajectoryIfPossible();
      }
   }

   public boolean isInSingularityNeighborhood(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInSingularityNeighborhood();
   }

   public void doSingularityEscape(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(true);
   }

   public void doSingularityEscape(RobotSide robotSide, double temporarySingularityEscapeNullspaceMultiplier)
   {
      footControlModules.get(robotSide).doSingularityEscape(temporarySingularityEscapeNullspaceMultiplier);
   }

   public boolean isInFlatSupportState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInFlatSupportState();
   }

   public void correctCoMHeight(RobotSide trailingLeg, FrameVector2d desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData)
   {
      RobotSide[] leadingLegFirst;
      if (trailingLeg != null)
         leadingLegFirst = new RobotSide[] {trailingLeg.getOppositeSide(), trailingLeg};
      else
         leadingLegFirst = RobotSide.values;

      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).updateLegSingularityModule();
      }

      // Correct, if necessary, the CoM height trajectory to avoid the knee to collapse
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).correctCoMHeightTrajectoryForCollapseAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame,
               footSwitches.get(robotSide).computeFootLoadPercentage());
      }

      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      for (RobotSide robotSide : leadingLegFirst)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame);
      }

      // Do that after to make sure the swing foot will land
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).correctCoMHeightTrajectoryForUnreachableFootStep(comHeightData);
      }
   }

   public void initializeContactStatesForDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide.getOppositeSide()) == ConstraintType.SWING) // That case happens when doing 2 steps on same side
            setFlatFootContactState(transferToSide.getOppositeSide());
         setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
      }

      reset();
   }

   public void updateContactStatesInDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (getCurrentConstraintType(robotSide) == ConstraintType.TOES)
               setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide) == ConstraintType.TOES)
            setFlatFootContactState(transferToSide);
      }
   }

   private final FrameVector footNormalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);

   public void setOnToesContactState(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      if (footControlModule.isInFlatSupportState())
      {
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
         footNormalContactVector.changeFrame(worldFrame);
      }
      else
      {
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      }

      footControlModule.setContactState(ConstraintType.TOES, footNormalContactVector);
   }

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);
   }

   private void setContactStateForMoveViaWaypoints(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.doSingularityEscapeBeforeTransitionToNextState();
      footControlModule.setContactState(ConstraintType.MOVE_VIA_WAYPOINTS);
   }

   public WalkOnTheEdgesManager getWalkOnTheEdgesManager()
   {
      return walkOnTheEdgesManager;
   }

   public boolean willDoToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      return walkOnTheEdgesManager.willDoToeOff(nextFootstep, transferToSide);
   }

   public boolean checkIfToeOffSafe(RobotSide trailingLeg, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      walkOnTheEdgesManager.updateToeOffStatus(trailingLeg, desiredECMP, desiredICP, currentICP);

      return walkOnTheEdgesManager.doToeOff();
   }

   public void requestToeOff(RobotSide trailingLeg, double predictedToeOffDuration)
   {
      if (footControlModules.get(trailingLeg).isInToeOff())
         return;
      footControlModules.get(trailingLeg).setPredictedToeOffDuration(predictedToeOffDuration);
      setOnToesContactState(trailingLeg);
   }

   public void registerDesiredContactPointForToeOff(RobotSide robotSide, FramePoint2d desiredContactPoint)
   {
      footControlModules.get(robotSide).registerDesiredContactPointForToeOff(desiredContactPoint);
   }

   public void reset()
   {
      walkOnTheEdgesManager.reset();
   }

   public boolean doToeOffIfPossible()
   {
      return walkOnTheEdgesManager.doToeOffIfPossible();
   }

   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return walkOnTheEdgesManager.doToeOffIfPossibleInSingleSupport();
   }

   public void lockKnee(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLegLocking.getDoubleValue());
   }

   public void doSupportSingularityEscape(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLeg.getDoubleValue());
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).resetHeightCorrectionParametersForSingularityAvoidance();
   }

   public void requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }
}
