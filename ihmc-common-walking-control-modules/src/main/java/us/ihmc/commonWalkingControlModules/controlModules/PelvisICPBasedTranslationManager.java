package us.ihmc.commonWalkingControlModules.controlModules;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class PelvisICPBasedTranslationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble supportPolygonSafeMargin = new YoDouble("supportPolygonSafeMargin", registry);
   private final YoDouble frozenOffsetDecayAlpha = new YoDouble("frozenOffsetDecayAlpha", registry);

   private final YoFramePoint2d desiredPelvisPosition = new YoFramePoint2d("desiredPelvis", worldFrame, registry);
   private final YoFrameVector2d desiredPelvisLinearVelocity = new YoFrameVector2d("desiredPelvisLinearVelocity", worldFrame, registry);
   private final YoFrameVector2d desiredPelvisLinearAcceleration = new YoFrameVector2d("desiredPelvisLinearAcceleration", worldFrame, registry);

   private final YoFrameVector2d pelvisLinearVelocity = new YoFrameVector2d("pelvisLinearVelocity", worldFrame, registry);

   private final YoDouble initialPelvisPositionTime = new YoDouble("initialPelvisPositionTime", registry);

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final YoFrameVector2d pelvisPositionError = new YoFrameVector2d("pelvisPositionError", worldFrame, registry);
   private final YoDouble derivativeGain = new YoDouble("pelvisPositionDerivativeGain", registry);
   private final YoDouble proportionalGain = new YoDouble("pelvisPositionProportionalGain", registry);

   private final YoDouble maxCMPRate = new YoDouble("pelvisPositionMaxCMPRate", registry);
   private final YoDouble maxICPError = new YoDouble("pelvisPositionMaxICPError", registry);

   private final YoFrameVector2d desiredICPOffset = new YoFrameVector2d("desiredICPOffset", worldFrame, registry);

   private final YoBoolean isEnabled = new YoBoolean("isPelvisTranslationManagerEnabled", registry);
   private final YoBoolean isRunning = new YoBoolean("isPelvisTranslationManagerRunning", registry);
   private final YoBoolean isFrozen = new YoBoolean("isPelvisTranslationManagerFrozen", registry);

   private final YoBoolean manualMode = new YoBoolean("manualModeICPOffset", registry);

   private final YoDouble yoTime;
   private final double controlDT;
   private final double omega0;

   private final YoBoolean isTrajectoryStopped = new YoBoolean("isPelvisTranslationalTrajectoryStopped", registry);

   private final CenterOfMassJacobian centerOfMassJacobian;

   private ReferenceFrame supportFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<MovingReferenceFrame> ankleZUpFrames;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final BipedSupportPolygons bipedSupportPolygons;
   private FrameConvexPolygon2d supportPolygon;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameVector3D tempAcceleration = new FrameVector3D();
   private final FramePoint2D tempPosition2d = new FramePoint2D();
   private final FrameVector2D tempError2d = new FrameVector2D();
   private final FrameVector2D tempICPOffset = new FrameVector2D();
   private final FrameVector2D icpOffsetForFreezing = new FrameVector2D();

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisTrajectoryCommand.class);

   public PelvisICPBasedTranslationManager(HighLevelHumanoidControllerToolbox controllerToolbox, double pelvisTranslationICPSupportPolygonSafeMargin,
                                           BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      supportPolygonSafeMargin.set(pelvisTranslationICPSupportPolygonSafeMargin);
      frozenOffsetDecayAlpha.set(0.998);

      yoTime = controllerToolbox.getYoTime();
      controlDT = controllerToolbox.getControlDT();
      omega0 = controllerToolbox.getOmega0();

      pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      midFeetZUpFrame = controllerToolbox.getReferenceFrames().getMidFeetZUpFrame();
      ankleZUpFrames = controllerToolbox.getReferenceFrames().getAnkleZUpReferenceFrames();

      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      centerOfMassJacobian = controllerToolbox.getCenterOfMassJacobian();
      rootJoint = controllerToolbox.getFullRobotModel().getRootJoint();
      this.bipedSupportPolygons = bipedSupportPolygons;

      boolean allowMultipleFrames = true;
      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvisOffset", allowMultipleFrames, worldFrame, registry);
      positionTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         positionTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

      proportionalGain.set(10.0);
      derivativeGain.set(10.0);
      maxCMPRate.set(1.0);
      maxICPError.set(0.015);

      manualMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            initialize();
         }
      });

      String namePrefix = "PelvisXYTranslation";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedPelvisTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      if (supportLeg == null)
      {
         supportFrame = midFeetZUpFrame;
         supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      }
      else
      {
         supportFrame = ankleZUpFrames.get(supportLeg);
         supportPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      }
   }

   public void computeICPOffset(FramePoint2D previousCMP, FramePoint2D capturePoint2d, FramePoint2D desiredICPToModify,
                                FrameVector2D desiredICPVelocityToModify)
   {
      if (!isEnabled.getBooleanValue() || (!isRunning.getBooleanValue() && !manualMode.getBooleanValue()))
      {
         desiredICPOffset.setToZero();
         icpOffsetForFreezing.setToZero();
         desiredICPToModify.changeFrame(worldFrame);
         desiredICPVelocityToModify.changeFrame(worldFrame);
         return;
      }

      convexPolygonScaler.scaleConvexPolygon(supportPolygon, supportPolygonSafeMargin.getDoubleValue(), safeSupportPolygonToConstrainICPOffset);

      if (manualMode.getBooleanValue())
      {
         // Ignore the desiredICPOffset frame assuming the user wants to control the ICP in the supportFrame
         tempICPOffset.setIncludingFrame(supportFrame, desiredICPOffset.getX(), desiredICPOffset.getY());
      }
      else if (isRunning.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            if (positionTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
            {
               double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
               double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
               PelvisTrajectoryCommand command = commandQueue.poll();
               numberOfQueuedCommands.decrement();
               initializeTrajectoryGenerator(command, firstTrajectoryPointTime, true);
               positionTrajectoryGenerator.compute(deltaTime);
            }
         }
         
         computeDesiredICPForPelvisPositionControl(capturePoint2d, desiredICP);
         desiredICPOffset.sub(desiredICP, desiredICPToModify);
         desiredICPOffset.getFrameTuple2dIncludingFrame(tempICPOffset);
      }

      if (!isRunning.getBooleanValue())
      {
         this.desiredCMPPrevious.set(previousCMP);
      }

      if (isFrozen.getBooleanValue())
      {
         icpOffsetForFreezing.scale(frozenOffsetDecayAlpha.getDoubleValue());
         icpOffsetForFreezing.changeFrame(desiredICPToModify.getReferenceFrame());

         desiredICPOffset.setAndMatchFrame(icpOffsetForFreezing);
         desiredICPToModify.changeFrame(icpOffsetForFreezing.getReferenceFrame());
         desiredICPToModify.add(icpOffsetForFreezing);
      }
      else
      {
         updateDesiredICPAndOffsetForFrozenState(desiredICPToModify, tempICPOffset);
         desiredICPVelocityToModify.set(desiredICPVelocity);
      }

      desiredICPPrevious.set(desiredICPToModify);
   }

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D desiredICPPrevious = new FramePoint2D();
   private final FrameVector2D icpError = new FrameVector2D();
   private final Twist rootJointTwist = new Twist();

   private final FramePoint2D desiredCMP = new FramePoint2D();
   private final FramePoint2D desiredCMPPrevious = new FramePoint2D();
   private final FrameVector2D desiredCMPDisplacement = new FrameVector2D();
   private final FrameVector2D desiredCoMAcceleration2d = new FrameVector2D();
   private final FramePoint2D comPosition2d = new FramePoint2D();
   private final FrameVector2D comVelocity2d = new FrameVector2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();

   private final FrameVector2D tempProportionalPart = new FrameVector2D();
   private final FrameVector2D tempDerivativePart = new FrameVector2D();

   private void computeDesiredICPForPelvisPositionControl(FramePoint2D capturePoint2d, FramePoint2D desiredICPToPack)
   {
      if (!isTrajectoryStopped.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
         positionTrajectoryGenerator.compute(deltaTime);
      }
      positionTrajectoryGenerator.getLinearData(tempPosition, tempVelocity, tempAcceleration);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.changeFrame(worldFrame);
      tempAcceleration.changeFrame(worldFrame);
      desiredPelvisPosition.setByProjectionOntoXYPlane(tempPosition);
      desiredPelvisLinearVelocity.setByProjectionOntoXYPlane(tempVelocity);
      desiredPelvisLinearAcceleration.setByProjectionOntoXYPlane(tempAcceleration);

      rootJoint.getJointTwist(rootJointTwist);
      rootJointTwist.getLinearPart(tempVelocity);
      tempVelocity.changeFrame(worldFrame);
      pelvisLinearVelocity.setByProjectionOntoXYPlane(tempVelocity);

      pelvisPositionError.set(desiredPelvisPosition);
      tempPosition2d.setToZero(pelvisZUpFrame);
      tempPosition2d.changeFrame(worldFrame);
      pelvisPositionError.sub(tempPosition2d);

      pelvisPositionError.getFrameTuple2dIncludingFrame(tempError2d);

      comPosition2d.setToZero(centerOfMassFrame);
      comPosition2d.changeFrameAndProjectToXYPlane(worldFrame);
      centerOfMassJacobian.getCenterOfMassVelocity(tempVelocity);
      tempVelocity.changeFrame(worldFrame);
      comVelocity2d.set(tempVelocity);

      pelvisPositionError.getFrameTuple2dIncludingFrame(tempProportionalPart);
      tempProportionalPart.scale(proportionalGain.getDoubleValue());

      desiredPelvisLinearVelocity.getFrameTuple2dIncludingFrame(tempDerivativePart);
//      tempDerivativePart.sub(comVelocity2d);
      tempDerivativePart.sub(pelvisLinearVelocity.getFrameTuple2d());
      tempDerivativePart.scale(derivativeGain.getDoubleValue());

      desiredPelvisLinearAcceleration.getFrameTuple2dIncludingFrame(desiredCoMAcceleration2d);
      desiredCoMAcceleration2d.add(tempProportionalPart);
      desiredCoMAcceleration2d.add(tempDerivativePart);

      desiredCMP.set(desiredCoMAcceleration2d);
      desiredCMP.scale(-1.0 / (omega0 * omega0));
      desiredCMP.add(comPosition2d);
//      desiredCMPDisplacement.sub(desiredCMP, desiredCMPPrevious);
//      desiredCMPDisplacement.clipToMaxLength(maxCMPRate.getDoubleValue() * controlDT);
//      desiredCMP.add(desiredCMPPrevious, desiredCMPDisplacement);

      bipedSupportPolygons.getSupportPolygonInWorld().orthogonalProjection(desiredCMP);

      desiredICPVelocity.sub(desiredICPPrevious, desiredCMP);
      desiredICPVelocity.scale(omega0 / (1.0 - controlDT * omega0));

      desiredICP.scaleAdd(controlDT, desiredICPVelocity, desiredICPPrevious);

//      icpError.sub(desiredICP, capturePoint2d);
//      icpError.clipToMaxLength(maxICPError.getDoubleValue());
//      desiredICP.add(capturePoint2d, icpError);
      desiredICP.changeFrame(supportFrame);
      safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredICP);
      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.sub(desiredICP, desiredICPPrevious);
      desiredICPVelocity.scale(1.0 / controlDT);

      desiredCMPPrevious.setAndScale(-1.0 / omega0, desiredICPVelocity);
      desiredCMPPrevious.add(desiredICP);
   }

   private void updateDesiredICPAndOffsetForFrozenState(FramePoint2D desiredICP, FrameVector2D desiredICPOffset)
   {
      icpOffsetForFreezing.setIncludingFrame(desiredICP);
      desiredICPOffset.changeFrame(worldFrame);
      desiredICP.add(desiredICPOffset);

      desiredICP.changeFrame(supportFrame);
      safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredICP);
      desiredICP.changeFrame(worldFrame);

      icpOffsetForFreezing.sub(desiredICP, icpOffsetForFreezing);
   }

   private final ConvexPolygonScaler convexPolygonScaler = new ConvexPolygonScaler();
   private final FrameConvexPolygon2d safeSupportPolygonToConstrainICPOffset = new FrameConvexPolygon2d();

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   public void goToHome()
   {
      freeze();
   }

   public void holdCurrentPosition()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());

      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);

      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.changeFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SelectionMatrix3D linearSelectionMatrix = command.getSelectionMatrix().getLinearPart();

      if (!linearSelectionMatrix.isXSelected() && !linearSelectionMatrix.isYSelected())
         return; // The user does not want to control the x and y of the pelvis, do nothing.

      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(command.getCommandId());
         initialPelvisPositionTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0, false);
         return;
      case QUEUE:
         boolean success = queuePelvisTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdCurrentPosition();
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         break;
      }
   }

   private boolean queuePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         PrintTools.warn(this,
                         "The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      long previousCommandId = command.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         PrintTools.warn(this, "Previous command ID mismatch: previous ID from command = " + previousCommandId
               + ", last message ID received by the controller = " + lastCommandId.getLongValue() + ". Aborting motion.");
         return false;
      }

      if (command.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getCommandId());

      return true;
   }
   
   private final FrameSE3TrajectoryPoint lastPointAdded = new FrameSE3TrajectoryPoint();
   
   private void initializeTrajectoryGenerator(PelvisTrajectoryCommand command, double firstTrajectoryPointTime, Boolean useLastTrajectoryPointAsInitial)
   {
      command.addTimeOffset(firstTrajectoryPointTime);

      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         if(useLastTrajectoryPointAsInitial)
         {
            positionTrajectoryGenerator.clear();
            positionTrajectoryGenerator.changeFrame(worldFrame);
            positionTrajectoryGenerator.appendWaypoint(lastPointAdded);
         }
         else 
         {
            if (isRunning.getBooleanValue())
            {
               positionTrajectoryGenerator.getPosition(tempPosition);
            }
            else
            {
               tempPosition.setToZero(pelvisZUpFrame);
            }
            
            tempPosition.changeFrame(worldFrame);
            tempVelocity.setToZero(worldFrame);

            positionTrajectoryGenerator.clear();
            positionTrajectoryGenerator.changeFrame(worldFrame);
            positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
         }
      }
      else
      {
         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = command.getTrajectoryPoint(trajectoryPointIndex);
         positionTrajectoryGenerator.appendWaypoint(trajectoryPoint);
         lastPointAdded.setIncludingFrame(trajectoryPoint);
      }

      if (supportFrame != null)
         positionTrajectoryGenerator.changeFrame(supportFrame);
      else
         positionTrajectoryGenerator.changeFrame(worldFrame);

      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = positionTrajectoryGenerator.getMaximumNumberOfWaypoints() - positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.setPropertiesOnly(command);

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.addTrajectoryPoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);

      pelvisPositionError.setToZero();

      desiredICPOffset.setToZero();
   }

   public void enable()
   {
      if (isEnabled.getBooleanValue())
         return;
      isEnabled.set(true);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);
      initialize();
   }

   public void freeze()
   {
      isFrozen.set(true);
   }

   private void initialize()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);
      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
   }
}
