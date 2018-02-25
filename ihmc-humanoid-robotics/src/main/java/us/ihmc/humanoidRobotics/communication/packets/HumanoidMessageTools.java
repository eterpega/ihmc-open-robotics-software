package us.ihmc.humanoidRobotics.communication.packets;

import static us.ihmc.communication.packets.Packet.VALID_MESSAGE_DEFAULT_ID;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlModeMessage;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorCommandPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeResponsePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DoorLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WallPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeLeafMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorAutoEnableFlagPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPowerCyclePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.momentum.CenterOfMassTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.momentum.MomentumTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.FisheyePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPointMapPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.RequestWristForceSensorCalibrationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorModePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.UIConnectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PrepareForLocomotionMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.ClearDelayQueueMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class HumanoidMessageTools
{
   private HumanoidMessageTools()
   {
   }

   public static AbortWalkingMessage createAbortWalkingMessage()
   {
      AbortWalkingMessage message = new AbortWalkingMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage()
   {
      AdjustFootstepMessage message = new AdjustFootstepMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.location.setToNaN();
      message.orientation.setToNaN();
      message.executionDelayTime = 0.0;
      return message;
   }

   public static ArmDesiredAccelerationsMessage createArmDesiredAccelerationsMessage()
   {
      ArmDesiredAccelerationsMessage message = new ArmDesiredAccelerationsMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.desiredAccelerations.set(createDesiredAccelerationsMessage());
      return message;
   }

   public static ArmTrajectoryMessage createArmTrajectoryMessage()
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.jointspaceTrajectory.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static AtlasDesiredPumpPSIPacket createAtlasDesiredPumpPSIPacket()
   {
      AtlasDesiredPumpPSIPacket message = new AtlasDesiredPumpPSIPacket();
      message.desiredPumpPsi = -1;
      return message;
   }

   public static AtlasElectricMotorAutoEnableFlagPacket createAtlasElectricMotorAutoEnableFlagPacket()
   {
      AtlasElectricMotorAutoEnableFlagPacket message = new AtlasElectricMotorAutoEnableFlagPacket();
      message.shouldAutoEnable = false;
      message.setDestination(PacketDestination.CONTROLLER);
      return message;
   }

   public static AtlasElectricMotorEnablePacket createAtlasElectricMotorEnablePacket()
   {
      AtlasElectricMotorEnablePacket message = new AtlasElectricMotorEnablePacket();
      message.enable = false;
      message.setDestination(PacketDestination.CONTROLLER);
      return message;
   }

   public static AtlasLowLevelControlModeMessage createAtlasLowLevelControlModeMessage()
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.requestedAtlasLowLevelControlMode = -1;
      return message;
   }

   public static AtlasWristSensorCalibrationRequestPacket createAtlasWristSensorCalibrationRequestPacket()
   {
      AtlasWristSensorCalibrationRequestPacket message = new AtlasWristSensorCalibrationRequestPacket();
      message.setDestination(PacketDestination.CONTROLLER);
      return message;
   }

   public static AutomaticManipulationAbortMessage createAutomaticManipulationAbortMessage()
   {
      AutomaticManipulationAbortMessage message = new AutomaticManipulationAbortMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket()
   {
      BDIBehaviorCommandPacket message = new BDIBehaviorCommandPacket();
      message.stop = false;
      return message;
   }

   public static BDIBehaviorStatusPacket createBDIBehaviorStatusPacket()
   {
      BDIBehaviorStatusPacket message = new BDIBehaviorStatusPacket();
      return message;
   }

   public static BehaviorControlModePacket createBehaviorControlModePacket()
   {
      BehaviorControlModePacket message = new BehaviorControlModePacket();
      return message;
   }

   public static BehaviorControlModeResponsePacket createBehaviorControlModeResponsePacket()
   {
      BehaviorControlModeResponsePacket message = new BehaviorControlModeResponsePacket();
      return message;
   }

   public static BehaviorStatusPacket createBehaviorStatusPacket()
   {
      BehaviorStatusPacket message = new BehaviorStatusPacket();
      return message;
   }

   public static BlackFlyParameterPacket createBlackFlyParameterPacket()
   {
      BlackFlyParameterPacket message = new BlackFlyParameterPacket();
      return message;
   }

   public static CapturabilityBasedStatus createCapturabilityBasedStatus()
   {
      CapturabilityBasedStatus message = new CapturabilityBasedStatus();
      message.capturePoint.setToNaN();
      message.desiredCapturePoint.setToNaN();
      message.centerOfMass.setToNaN();
      return message;
   }

   public static CenterOfMassTrajectoryMessage createCenterOfMassTrajectoryMessage()
   {
      CenterOfMassTrajectoryMessage message = new CenterOfMassTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.euclideanTrajectory.set(createEuclideanTrajectoryMessage());
      return message;
   }

   public static ChestHybridJointspaceTaskspaceTrajectoryMessage createChestHybridJointspaceTaskspaceTrajectoryMessage()
   {
      ChestHybridJointspaceTaskspaceTrajectoryMessage message = new ChestHybridJointspaceTaskspaceTrajectoryMessage();
      message.taskspaceTrajectoryMessage.set(createSO3TrajectoryMessage());
      message.jointspaceTrajectoryMessage.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage()
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.so3Trajectory.set(createSO3TrajectoryMessage());
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static ClearDelayQueueMessage createClearDelayQueueMessage()
   {
      ClearDelayQueueMessage message = new ClearDelayQueueMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static DesiredAccelerationsMessage createDesiredAccelerationsMessage()
   {
      DesiredAccelerationsMessage message = new DesiredAccelerationsMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static DetectedObjectPacket createDetectedObjectPacket()
   {
      DetectedObjectPacket message = new DetectedObjectPacket();
      message.pose.setToNaN();
      return message;
   }

   public static DoorLocationPacket createDoorLocationPacket()
   {
      DoorLocationPacket message = new DoorLocationPacket();
      message.doorTransformToWorld.setToNaN();
      return message;
   }

   public static DrillDetectionPacket createDrillDetectionPacket()
   {
      DrillDetectionPacket message = new DrillDetectionPacket();
      message.isDrillOn = false;
      message.setDestination(PacketDestination.UI);
      return message;
   }

   public static EndOfScriptCommand createEndOfScriptCommand()
   {
      EndOfScriptCommand message = new EndOfScriptCommand();
      return message;
   }

   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage()
   {
      EuclideanTrajectoryMessage message = new EuclideanTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.selectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.frameInformation.set(createFrameInformation());
      message.weightMatrix.set(MessageTools.createWeightMatrix3DMessage());
      message.useCustomControlFrame = false;
      message.controlFramePose.setToNaN();
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static EuclideanTrajectoryPointMessage createEuclideanTrajectoryPointMessage()
   {
      EuclideanTrajectoryPointMessage message = new EuclideanTrajectoryPointMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.time = Double.NaN;
      message.position.setToNaN();
      message.linearVelocity.setToZero();
      return message;
   }

   public static FisheyePacket createFisheyePacket()
   {
      FisheyePacket message = new FisheyePacket();
      message.videoPacket.set(createVideoPacket());
      return message;
   }

   public static FootLoadBearingMessage createFootLoadBearingMessage()
   {
      FootLoadBearingMessage message = new FootLoadBearingMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.loadBearingRequest = -1;
      message.executionDelayTime = 0.0;
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage()
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.se3Trajectory.set(createSE3TrajectoryMessage());
      return message;
   }

   public static FootstepDataListMessage createFootstepDataListMessage()
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.executionTiming = ExecutionTiming.CONTROL_DURATIONS.toByte();
      message.defaultSwingDuration = 0.0;
      message.defaultTransferDuration = -1.0;
      message.finalTransferDuration = -1.0;
      message.trustHeightOfFootsteps = true;
      message.areFootstepsAdjustable = true;
      message.offsetFootstepsWithExecutionError = false;
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage()
   {
      FootstepDataMessage message = new FootstepDataMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.location.setToNaN();
      message.orientation.setToNaN();
      message.trajectoryType = TrajectoryType.DEFAULT.toByte();
      message.swingHeight = 0.0;
      message.swingTrajectoryBlendDuration = 0.0;
      message.swingDuration = -1.0;
      message.transferDuration = -1.0;
      message.touchdownDuration = -1.0;
      message.executionDelayTime = 0.0;
      return message;
   }

   public static FootstepPathPlanPacket createFootstepPathPlanPacket()
   {
      FootstepPathPlanPacket message = new FootstepPathPlanPacket();
      message.pathCost = Double.POSITIVE_INFINITY;
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket()
   {
      FootstepPlanRequestPacket message = new FootstepPlanRequestPacket();
      message.maxSuboptimality = 1.0;
      return message;
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket()
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.stanceFootPositionInWorld.setToNaN();
      message.stanceFootOrientationInWorld.setToNaN();
      message.goalPositionInWorld.setToNaN();
      message.goalOrientationInWorld.setToNaN();
      message.planarRegionsListMessage.set(MessageTools.createPlanarRegionsListMessage());
      message.plannerRequestId = FootstepPlanningRequestPacket.NO_PLAN_ID;
      return message;
   }

   public static FootstepPlanningToolboxOutputStatus createFootstepPlanningToolboxOutputStatus()
   {
      FootstepPlanningToolboxOutputStatus message = new FootstepPlanningToolboxOutputStatus();
      message.planId = FootstepPlanningRequestPacket.NO_PLAN_ID;
      message.planarRegionsList.set(MessageTools.createPlanarRegionsListMessage());
      message.lowLevelPlannerGoal.setToNaN();
      return message;
   }

   public static FootstepStatusMessage createFootstepStatusMessage()
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.footstepStatus = -1;
      message.robotSide = -1;
      message.desiredFootPositionInWorld.setToNaN();
      message.desiredFootOrientationInWorld.setToNaN();
      message.actualFootPositionInWorld.setToNaN();
      message.actualFootOrientationInWorld.setToNaN();
      return message;
   }

   public static FrameInformation createFrameInformation()
   {
      FrameInformation message = new FrameInformation();
      message.trajectoryReferenceFrameId = ReferenceFrame.getWorldFrame().getNameBasedHashCode();
      message.dataReferenceFrameId = NameBasedHashCodeTools.DEFAULT_HASHCODE;
      return message;
   }

   public static GoHomeMessage createGoHomeMessage()
   {
      GoHomeMessage message = new GoHomeMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.humanoidBodyPart = -1;
      message.robotSide = -1;
      message.trajectoryTime = Double.NaN;
      message.executionDelayTime = 0.0;
      return message;
   }

   public static HandCollisionDetectedPacket createHandCollisionDetectedPacket()
   {
      HandCollisionDetectedPacket message = new HandCollisionDetectedPacket();
      message.robotSide = -1;
      message.collisionSeverityLevelOneToThree = -1;
      return message;
   }

   public static HandDesiredConfigurationMessage createHandDesiredConfigurationMessage()
   {
      HandDesiredConfigurationMessage message = new HandDesiredConfigurationMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.desiredHandConfiguration = -1;
      return message;
   }

   public static HandHybridJointspaceTaskspaceTrajectoryMessage createHandHybridJointspaceTaskspaceTrajectoryMessage()
   {
      HandHybridJointspaceTaskspaceTrajectoryMessage message = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.taskspaceTrajectoryMessage.set(createSE3TrajectoryMessage());
      message.jointspaceTrajectoryMessage.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static HandJointAnglePacket createHandJointAnglePacket()
   {
      HandJointAnglePacket message = new HandJointAnglePacket();
      message.robotSide = -1;
      return message;
   }

   public static HandLoadBearingMessage createHandLoadBearingMessage()
   {
      HandLoadBearingMessage message = new HandLoadBearingMessage();
      message.robotSide = -1;
      message.jointspaceTrajectory.set(createJointspaceTrajectoryMessage());
      message.executionDelayTime = 0.0;
      message.loadBearingMessage.set(createLoadBearingMessage());
      return message;
   }

   public static HandPowerCyclePacket createHandPowerCyclePacket()
   {
      HandPowerCyclePacket message = new HandPowerCyclePacket();
      message.setDestination(PacketDestination.CONTROLLER);
      message.robotSide = -1;
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage()
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.robotSide = -1;
      message.se3Trajectory.set(createSE3TrajectoryMessage());
      return message;
   }

   public static HeadHybridJointspaceTaskspaceTrajectoryMessage createHeadHybridJointspaceTaskspaceTrajectoryMessage()
   {
      HeadHybridJointspaceTaskspaceTrajectoryMessage message = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.taskspaceTrajectoryMessage.set(createSO3TrajectoryMessage());
      message.jointspaceTrajectoryMessage.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage()
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.so3Trajectory.set(createSO3TrajectoryMessage());
      return message;
   }

   public static HeightQuadTreeLeafMessage createHeightQuadTreeLeafMessage()
   {
      HeightQuadTreeLeafMessage message = new HeightQuadTreeLeafMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.height = Float.NaN;
      message.center.setToNaN();
      return message;
   }

   public static HeightQuadTreeMessage createHeightQuadTreeMessage()
   {
      HeightQuadTreeMessage message = new HeightQuadTreeMessage();
      message.defaultHeight = Float.NaN;
      message.resolution = Float.NaN;
      message.sizeX = Float.NaN;
      message.sizeY = Float.NaN;
      return message;
   }

   public static HeightQuadTreeToolboxRequestMessage createHeightQuadTreeToolboxRequestMessage()
   {
      HeightQuadTreeToolboxRequestMessage message = new HeightQuadTreeToolboxRequestMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static HighLevelStateChangeStatusMessage createHighLevelStateChangeStatusMessage()
   {
      HighLevelStateChangeStatusMessage message = new HighLevelStateChangeStatusMessage();
      message.setDestination(PacketDestination.ROS_API);
      return message;
   }

   public static HighLevelStateMessage createHighLevelStateMessage()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static HumanoidBehaviorTypePacket createHumanoidBehaviorTypePacket()
   {
      HumanoidBehaviorTypePacket message = new HumanoidBehaviorTypePacket();
      return message;
   }

   public static IntrinsicParametersMessage createIntrinsicParametersMessage()
   {
      IntrinsicParametersMessage message = new IntrinsicParametersMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage()
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static LegCompliancePacket createLegCompliancePacket()
   {
      LegCompliancePacket message = new LegCompliancePacket();
      message.robotSide = -1;
      return message;
   }

   public static LoadBearingMessage createLoadBearingMessage()
   {
      LoadBearingMessage message = new LoadBearingMessage();
      message.load = false;
      message.coefficientOfFriction = 0.0;
      message.bodyFrameToContactFrame.setToNaN();
      message.contactNormalInWorldFrame.setToNaN();
      return message;
   }

   public static LocalizationPacket createLocalizationPacket()
   {
      LocalizationPacket message = new LocalizationPacket();
      return message;
   }

   public static LocalizationPointMapPacket createLocalizationPointMapPacket()
   {
      LocalizationPointMapPacket message = new LocalizationPointMapPacket();
      message.setDestination(PacketDestination.UI);
      return message;
   }

   public static LocalizationStatusPacket createLocalizationStatusPacket()
   {
      LocalizationStatusPacket message = new LocalizationStatusPacket();
      return message;
   }

   public static ManipulationAbortedStatus createManipulationAbortedStatus()
   {
      ManipulationAbortedStatus message = new ManipulationAbortedStatus();
      return message;
   }

   public static ManualHandControlPacket createManualHandControlPacket()
   {
      ManualHandControlPacket message = new ManualHandControlPacket();
      message.robotSide = -1;
      message.controlType = -1;
      return message;
   }

   public static MomentumTrajectoryMessage createMomentumTrajectoryMessage()
   {
      MomentumTrajectoryMessage message = new MomentumTrajectoryMessage();
      message.angularMomentumTrajectory.set(createEuclideanTrajectoryMessage());
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static MultisenseParameterPacket createMultisenseParameterPacket()
   {
      MultisenseParameterPacket message = new MultisenseParameterPacket();
      return message;
   }

   public static NeckDesiredAccelerationsMessage createNeckDesiredAccelerationsMessage()
   {
      NeckDesiredAccelerationsMessage message = new NeckDesiredAccelerationsMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.desiredAccelerations.set(createDesiredAccelerationsMessage());
      return message;
   }

   public static NeckTrajectoryMessage createNeckTrajectoryMessage()
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.jointspaceTrajectory.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static ObjectWeightPacket createObjectWeightPacket()
   {
      ObjectWeightPacket message = new ObjectWeightPacket();
      message.robotSide = -1;
      return message;
   }

   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage()
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.weight = Double.NaN;
      return message;
   }

   public static PauseWalkingMessage createPauseWalkingMessage()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage()
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.enableUserPelvisControlDuringWalking = false;
      message.so3Trajectory.set(createSO3TrajectoryMessage());
      return message;
   }

   public static PelvisPoseErrorPacket createPelvisPoseErrorPacket()
   {
      PelvisPoseErrorPacket message = new PelvisPoseErrorPacket();
      return message;
   }

   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage()
   {
      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.enableUserPelvisControl = false;
      message.enableUserPelvisControlDuringWalking = false;
      message.se3Trajectory.set(createSE3TrajectoryMessage());
      return message;
   }

   public static PlanOffsetStatus createPlanOffsetStatus()
   {
      PlanOffsetStatus message = new PlanOffsetStatus();
      message.offsetVector.setToZero();
      return message;
   }

   public static PointCloudWorldPacket createPointCloudWorldPacket()
   {
      PointCloudWorldPacket message = new PointCloudWorldPacket();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.setDestination(PacketDestination.BROADCAST);
      return message;
   }

   public static PrepareForLocomotionMessage createPrepareForLocomotionMessage()
   {
      PrepareForLocomotionMessage message = new PrepareForLocomotionMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static ReachingManifoldMessage createReachingManifoldMessage()
   {
      ReachingManifoldMessage message = new ReachingManifoldMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      message.manifoldOriginPosition.setToNaN();
      message.manifoldOriginOrientation.setToNaN();
      return message;
   }

   public static RequestWristForceSensorCalibrationPacket createRequestWristForceSensorCalibrationPacket()
   {
      RequestWristForceSensorCalibrationPacket message = new RequestWristForceSensorCalibrationPacket();
      return message;
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage()
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static SCSListenerPacket createSCSListenerPacket()
   {
      SCSListenerPacket message = new SCSListenerPacket();
      message.isStopped = true;
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage()
   {
      SE3TrajectoryMessage message = new SE3TrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.angularSelectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.linearSelectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.frameInformation.set(createFrameInformation());
      message.angularWeightMatrix.set(MessageTools.createWeightMatrix3DMessage());
      message.linearWeightMatrix.set(MessageTools.createWeightMatrix3DMessage());
      message.useCustomControlFrame = false;
      message.controlFramePose.setToNaN();
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static SE3TrajectoryPointMessage createSE3TrajectoryPointMessage()
   {
      SE3TrajectoryPointMessage message = new SE3TrajectoryPointMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.time = Double.NaN;
      message.position.setToNaN();
      message.orientation.setToNaN();
      message.linearVelocity.setToZero();
      message.angularVelocity.setToZero();
      return message;
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage()
   {
      SO3TrajectoryMessage message = new SO3TrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.frameInformation.set(createFrameInformation());
      message.selectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.weightMatrix.set(MessageTools.createWeightMatrix3DMessage());
      message.useCustomControlFrame = false;
      message.controlFramePose.setToNaN();
      message.queueingProperties.set(MessageTools.createQueueableMessage());
      return message;
   }

   public static SO3TrajectoryPointMessage createSO3TrajectoryPointMessage()
   {
      SO3TrajectoryPointMessage message = new SO3TrajectoryPointMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.time = Double.NaN;
      message.orientation.setToNaN();
      message.angularVelocity.setToZero();
      return message;
   }

   public static SimpleCoactiveBehaviorDataPacket createSimpleCoactiveBehaviorDataPacket()
   {
      SimpleCoactiveBehaviorDataPacket message = new SimpleCoactiveBehaviorDataPacket();
      return message;
   }

   public static SnapFootstepPacket createSnapFootstepPacket()
   {
      SnapFootstepPacket message = new SnapFootstepPacket();
      return message;
   }

   public static SpineDesiredAccelerationsMessage createSpineDesiredAccelerationsMessage()
   {
      SpineDesiredAccelerationsMessage message = new SpineDesiredAccelerationsMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.desiredAccelerations.set(createDesiredAccelerationsMessage());
      return message;
   }

   public static SpineTrajectoryMessage createSpineTrajectoryMessage()
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.jointspaceTrajectory.set(createJointspaceTrajectoryMessage());
      return message;
   }

   public static StampedPosePacket createStampedPosePacket()
   {
      StampedPosePacket message = new StampedPosePacket();
      message.pose.setToNaN();
      message.timeStamp = -1;
      return message;
   }

   public static StateEstimatorModePacket createStateEstimatorModePacket()
   {
      StateEstimatorModePacket message = new StateEstimatorModePacket();
      message.requestedStateEstimatorMode = -1;
      return message;
   }

   public static StopAllTrajectoryMessage createStopAllTrajectoryMessage()
   {
      StopAllTrajectoryMessage message = new StopAllTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage()
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.time = Double.NaN;
      message.position = Double.NaN;
      message.velocity = 0.0;
      return message;
   }

   public static UIConnectedPacket createUIConnectedPacket()
   {
      UIConnectedPacket message = new UIConnectedPacket();
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket()
   {
      ValveLocationPacket message = createValveLocationPacket();
      message.valvePoseInWorld.setToNaN();
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket()
   {
      VehiclePosePacket message = new VehiclePosePacket();
      message.position.setToNaN();
      message.orientation.setToNaN();
      return message;
   }

   public static VideoPacket createVideoPacket()
   {
      VideoPacket message = new VideoPacket();
      message.videoSource = -1;
      message.timeStamp = -1;
      message.position.setToNaN();
      message.orientation.setToNaN();
      message.intrinsicParameters.set(createIntrinsicParametersMessage());
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket()
   {
      WalkToGoalBehaviorPacket message = new WalkToGoalBehaviorPacket();
      message.walkToGoalAction = -1;
      message.goalRobotSide = -1;
      return message;
   }

   public static WalkingControllerFailureStatusMessage createWalkingControllerFailureStatusMessage()
   {
      WalkingControllerFailureStatusMessage message = new WalkingControllerFailureStatusMessage();
      message.fallingDirection.setToNaN();
      return message;
   }

   public static WalkingStatusMessage createWalkingStatusMessage()
   {
      WalkingStatusMessage message = new WalkingStatusMessage();
      message.walkingStatus = -1;
      return message;
   }

   public static WallPosePacket createWallPosePacket()
   {
      WallPosePacket message = new WallPosePacket();
      message.cuttingRadius = 0.20;
      message.centerPosition.setToNaN();
      message.centerOrientation.setToNaN();
      return message;
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage()
   {
      WaypointBasedTrajectoryMessage message = new WaypointBasedTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      message.angularSelectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.linearSelectionMatrix.set(MessageTools.createSelectionMatrix3DMessage());
      message.controlFramePositionInEndEffector.setToNaN();
      message.controlFrameOrientationInEndEffector.setToNaN();
      message.weight = Double.NaN;
      return message;
   }

   public static WholeBodyTrajectoryMessage createWholeBodyTrajectoryMessage()
   {
      WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.leftHandTrajectoryMessage.set(createHandTrajectoryMessage(RobotSide.LEFT));
      message.rightHandTrajectoryMessage.set(createHandTrajectoryMessage(RobotSide.RIGHT));
      message.leftArmTrajectoryMessage.set(createArmTrajectoryMessage(RobotSide.LEFT));
      message.rightArmTrajectoryMessage.set(createArmTrajectoryMessage(RobotSide.RIGHT));
      message.chestTrajectoryMessage.set(createChestTrajectoryMessage());
      message.pelvisTrajectoryMessage.set(createPelvisTrajectoryMessage());
      message.leftFootTrajectoryMessage.set(createFootTrajectoryMessage(RobotSide.LEFT));
      message.rightFootTrajectoryMessage.set(createFootTrajectoryMessage(RobotSide.RIGHT));
      message.headTrajectoryMessage.set(createHeadTrajectoryMessage());
      message.executionDelayTime = 0.0;
      return message;
   }

   public static WholeBodyTrajectoryToolboxConfigurationMessage createWholeBodyTrajectoryToolboxConfigurationMessage()
   {
      WholeBodyTrajectoryToolboxConfigurationMessage message = new WholeBodyTrajectoryToolboxConfigurationMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.numberOfInitialGuesses = -1;
      message.maximumExpansionSize = -1;
      message.initialConfiguration.set(MessageTools.createKinematicsToolboxOutputStatus());
      return message;
   }

   public static WholeBodyTrajectoryToolboxMessage createWholeBodyTrajectoryToolboxMessage()
   {
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage();
      message.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      message.configuration.set(createWholeBodyTrajectoryToolboxConfigurationMessage());
      return message;
   }

   public static WholeBodyTrajectoryToolboxOutputStatus createWholeBodyTrajectoryToolboxOutputStatus()
   {
      WholeBodyTrajectoryToolboxOutputStatus message = new WholeBodyTrajectoryToolboxOutputStatus();
      message.planningResult = 0;
      return message;
   }

   public static BlackFlyParameterPacket createBlackFlyParameterPacket(boolean fromUI, double gain, double exposure, double frameRate, double shutter,
                                                                       boolean autoExposure, boolean autoGain, boolean autoShutter, RobotSide side)
   {
      BlackFlyParameterPacket message = createBlackFlyParameterPacket();
      message.fromUI = fromUI;
      message.exposure = exposure;
      message.shutter = shutter;
      message.gain = gain;
      message.frameRate = frameRate;
      message.autoExposure = autoExposure;
      message.autoGain = autoGain;
      message.autoShutter = autoShutter;
      message.robotSide = side.toByte();
      return message;
   }

   public static DesiredAccelerationsMessage createDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      DesiredAccelerationsMessage message = createDesiredAccelerationsMessage();
      message.desiredJointAccelerations.add(desiredJointAccelerations);
      return message;
   }

   public static NeckDesiredAccelerationsMessage createNeckDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      NeckDesiredAccelerationsMessage message = createNeckDesiredAccelerationsMessage();
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations);
      return message;
   }

   public static ChestHybridJointspaceTaskspaceTrajectoryMessage createChestHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                       JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ChestHybridJointspaceTaskspaceTrajectoryMessage message = createChestHybridJointspaceTaskspaceTrajectoryMessage();
      message.taskspaceTrajectoryMessage = new SO3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static HeadHybridJointspaceTaskspaceTrajectoryMessage createHeadHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HeadHybridJointspaceTaskspaceTrajectoryMessage message = createHeadHybridJointspaceTaskspaceTrajectoryMessage();
      message.taskspaceTrajectoryMessage = new SO3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static HandHybridJointspaceTaskspaceTrajectoryMessage createHandHybridJointspaceTaskspaceTrajectoryMessage(RobotSide robotSide,
                                                                                                                     SE3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HandHybridJointspaceTaskspaceTrajectoryMessage message = createHandHybridJointspaceTaskspaceTrajectoryMessage();
      message.robotSide = robotSide.toByte();
      message.taskspaceTrajectoryMessage = new SE3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ArmTrajectoryMessage message = createArmTrajectoryMessage(robotSide);
      message.jointspaceTrajectory.set(jointspaceTrajectoryMessage);
      message.setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of arm joints.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      ArmTrajectoryMessage message = createArmTrajectoryMessage(robotSide);
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of arm joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      ArmTrajectoryMessage message = createArmTrajectoryMessage(robotSide);
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      ArmTrajectoryMessage message = createArmTrajectoryMessage(robotSide);
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide)
   {
      ArmTrajectoryMessage message = createArmTrajectoryMessage();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      HandTrajectoryMessage message = createHandTrajectoryMessage(robotSide);
      message.se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      HandTrajectoryMessage message = createHandTrajectoryMessage(robotSide);
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrameId);
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = createHandTrajectoryMessage(robotSide);
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame);
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide)
   {
      HandTrajectoryMessage message = createHandTrajectoryMessage();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static BehaviorStatusPacket createBehaviorStatusPacket(CurrentBehaviorStatus requestedControl)
   {
      BehaviorStatusPacket message = createBehaviorStatusPacket();
      message.currentBehaviorStatus = requestedControl.toByte();
      return message;
   }

   public static LegCompliancePacket createLegCompliancePacket(float[] maxVelocityDeltas, RobotSide robotSide)
   {
      LegCompliancePacket message = createLegCompliancePacket();
      message.maxVelocityDeltas.add(maxVelocityDeltas);
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static SnapFootstepPacket createSnapFootstepPacket(List<FootstepDataMessage> data, int[] footstepOrder, byte[] flag)
   {
      SnapFootstepPacket message = createSnapFootstepPacket();
      MessageTools.copyData(data, message.footstepData);
      message.footstepOrder.add(footstepOrder);
      message.flag.add(flag);
      return message;
   }

   public static BehaviorControlModePacket createBehaviorControlModePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModePacket message = createBehaviorControlModePacket();
      message.behaviorControlModeEnumRequest = requestedControl.toByte();
      return message;
   }

   /**
    * Create a message to request one end-effector to switch to load bearing. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide refers to the side of the end-effector if necessary.
    */
   public static FootLoadBearingMessage createFootLoadBearingMessage(RobotSide robotSide, LoadBearingRequest request)
   {
      FootLoadBearingMessage message = createFootLoadBearingMessage();
      message.robotSide = robotSide.toByte();
      message.loadBearingRequest = request.toByte();
      return message;
   }

   // joint values should be in the range [0,1]
   public static ManualHandControlPacket createManualHandControlPacket(RobotSide robotSide, double index, double middle, double thumb, double spread,
                                                                       int controlType)
   {
      ManualHandControlPacket message = createManualHandControlPacket();
      message.robotSide = robotSide.toByte();
      message.index = index;
      message.middle = middle;
      message.thumb = thumb;
      message.spread = spread;
      message.controlType = controlType;
      return message;
   }

   public static MultisenseParameterPacket createMultisenseParameterPacket(boolean initialize, double gain, double motorSpeed, double dutyCycle,
                                                                           boolean ledEnable, boolean flashEnable, boolean autoExposure,
                                                                           boolean autoWhiteBalance)
   {
      MultisenseParameterPacket message = createMultisenseParameterPacket();
      message.initialize = initialize;
      message.gain = gain;
      message.flashEnable = flashEnable;
      message.motorSpeed = motorSpeed;
      message.ledEnable = ledEnable;
      message.dutyCycle = dutyCycle;
      message.autoExposure = autoExposure;
      message.autoWhiteBalance = autoWhiteBalance;
      return message;
   }

   public static DoorLocationPacket createDoorLocationPacket(RigidBodyTransform doorTransformToWorld)
   {
      return createDoorLocationPacket(new Pose3D(doorTransformToWorld));
   }

   public static DoorLocationPacket createDoorLocationPacket(Pose3D doorTransformToWorld)
   {
      DoorLocationPacket message = createDoorLocationPacket();
      message.doorTransformToWorld = doorTransformToWorld;
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(Point3D position, Quaternion orientation)
   {
      VehiclePosePacket message = createVehiclePosePacket();
      message.position = position;
      message.orientation = orientation;
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(RigidBodyTransform transformFromVehicleToWorld)
   {
      VehiclePosePacket message = createVehiclePosePacket();
      message.orientation = new Quaternion(transformFromVehicleToWorld.getRotationMatrix());
      message.position = new Point3D(transformFromVehicleToWorld.getTranslationVector());
      return message;
   }

   public static HighLevelStateChangeStatusMessage createHighLevelStateChangeStatusMessage(HighLevelControllerName initialState,
                                                                                           HighLevelControllerName endState)
   {
      HighLevelStateChangeStatusMessage message = createHighLevelStateChangeStatusMessage();
      message.setInitialHighLevelControllerName(initialState == null ? -1 : initialState.toByte());
      message.setEndHighLevelControllerName(endState == null ? -1 : endState.toByte());
      return message;
   }

   /**
    * To set disable exploration on this rigid body.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody)
   {
      ConfigurationSpaceName[] configurations = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.YAW,
            ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL};
      double[] regionAmplitude = new double[] {0, 0, 0, 0, 0, 0};

      return createRigidBodyExplorationConfigurationMessage(rigidBody, configurations, regionAmplitude);
   }

   /**
    * To set enable exploration on this rigid body with following order of ConfigurationSpaceName.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      return createRigidBodyExplorationConfigurationMessage(rigidBody, degreesOfFreedomToExplore,
                                                            WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationAmplitudeArray(degreesOfFreedomToExplore));
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeAmplitudes)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      byte[] degreesOfFreedomToExplore1 = ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore);
      if (degreesOfFreedomToExplore1.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore1.length
               + ", explorationRangeLowerLimits.length = ");

      message.configurationSpaceNamesToExplore.reset();
      message.explorationRangeUpperLimits.reset();
      message.explorationRangeLowerLimits.reset();

      message.configurationSpaceNamesToExplore.add(degreesOfFreedomToExplore1);

      for (int i = 0; i < degreesOfFreedomToExplore1.length; i++)
      {
         message.explorationRangeUpperLimits.add(explorationRangeAmplitudes[i]);
         message.explorationRangeLowerLimits.add(-explorationRangeAmplitudes[i]);
      }

      return message;
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeUpperLimits,
                                                                                                         double[] explorationRangeLowerLimits)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      byte[] degreesOfFreedomToExplore1 = ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore);
      if (degreesOfFreedomToExplore1.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore1.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore1.length
               + ", explorationRangeLowerLimits.length = ");

      message.configurationSpaceNamesToExplore.reset();
      message.explorationRangeUpperLimits.reset();
      message.explorationRangeLowerLimits.reset();

      message.configurationSpaceNamesToExplore.add(degreesOfFreedomToExplore1);
      message.explorationRangeUpperLimits.add(explorationRangeUpperLimits);
      message.explorationRangeLowerLimits.add(explorationRangeLowerLimits);

      return message;
   }

   public static FootstepPathPlanPacket createFootstepPathPlanPacket(boolean goalsValid, FootstepDataMessage start, List<FootstepDataMessage> originalGoals,
                                                                     List<FootstepDataMessage> ADStarPathPlan, List<Boolean> footstepUnknown,
                                                                     double subOptimality, double cost)
   {
      FootstepPathPlanPacket message = createFootstepPathPlanPacket();
      message.goalsValid = goalsValid;
      message.start = start;
      MessageTools.copyData(originalGoals, message.originalGoals);
      MessageTools.copyData(ADStarPathPlan, message.pathPlan);
      footstepUnknown.stream().map(b -> b ? 1 : 0).forEach(message.footstepUnknown::add);
      message.subOptimality = subOptimality;
      message.pathCost = cost;
      return message;
   }

   public static ObjectWeightPacket createObjectWeightPacket(RobotSide robotSide, double weight)
   {
      ObjectWeightPacket message = createObjectWeightPacket();
      message.robotSide = robotSide.toByte();
      message.weight = weight;
      return message;
   }

   public static HandPowerCyclePacket createHandPowerCyclePacket(RobotSide robotSide)
   {
      HandPowerCyclePacket message = createHandPowerCyclePacket();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      return createWaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints);
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints,
                                                                                     SelectionMatrix6D selectionMatrix)
   {
      WaypointBasedTrajectoryMessage message = new WaypointBasedTrajectoryMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      if (waypointTimes.length != waypoints.length)
         throw new RuntimeException("Inconsistent array lengths.");

      message.waypointTimes.reset();
      message.waypointTimes.add(waypointTimes);
      MessageTools.copyData(waypoints, message.waypoints);
      if (selectionMatrix != null)
      {
         message.angularSelectionMatrix.selectionFrameId = MessageTools.toFrameId(selectionMatrix.getAngularSelectionFrame());
         message.angularSelectionMatrix.xSelected = selectionMatrix.isAngularXSelected();
         message.angularSelectionMatrix.ySelected = selectionMatrix.isAngularYSelected();
         message.angularSelectionMatrix.zSelected = selectionMatrix.isAngularZSelected();
         message.linearSelectionMatrix.selectionFrameId = MessageTools.toFrameId(selectionMatrix.getLinearSelectionFrame());
         message.linearSelectionMatrix.xSelected = selectionMatrix.isLinearXSelected();
         message.linearSelectionMatrix.ySelected = selectionMatrix.isLinearYSelected();
         message.linearSelectionMatrix.zSelected = selectionMatrix.isLinearZSelected();
      }
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired pelvis position expressed in world frame.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                       QuaternionReadOnly desiredOrientation)
   {
      PelvisTrajectoryMessage message = createPelvisTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, ReferenceFrame.getWorldFrame());
      return message;
   }

   public static PelvisPoseErrorPacket createPelvisPoseErrorPacket(float residualError, float totalError, boolean hasMapBeenReset)
   {
      PelvisPoseErrorPacket message = createPelvisPoseErrorPacket();
      message.residualError = residualError;
      message.totalError = totalError;
      message.hasMapBeenReset = hasMapBeenReset;
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   List<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      AdjustFootstepMessage message = createAdjustFootstepMessage();
      message.robotSide = robotSide.toByte();
      message.location = location;
      message.orientation = orientation;
      MessageTools.copyData(predictedContactPoints, message.predictedContactPoints);
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType,
                                                                   double swingHeight)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   List<Point2D> predictedContactPoints)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, null);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(Footstep footstep)
   {
      AdjustFootstepMessage message = createAdjustFootstepMessage();
      message.robotSide = footstep.getRobotSide().toByte();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.location = new Point3D(location);
      message.orientation = new Quaternion(orientation);
      MessageTools.copyData(footstep.getPredictedContactPoints(), message.predictedContactPoints);
      return message;
   }

   public static NeckTrajectoryMessage createNeckTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      NeckTrajectoryMessage message = createNeckTrajectoryMessage();
      message.jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of joints.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      NeckTrajectoryMessage message = createNeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      NeckTrajectoryMessage message = createNeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      NeckTrajectoryMessage message = createNeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      return message;
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose)
   {
      return createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose, FootstepPlannerType.PLANAR_REGION_BIPEDAL);
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose, FootstepPlannerType requestedPlannerType)
   {
      FootstepPlanningRequestPacket message = createFootstepPlanningRequestPacket();
      message.initialStanceRobotSide = initialStanceSide.toByte();

      FramePoint3D initialFramePoint = new FramePoint3D(initialStanceFootPose.getPosition());
      initialFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.stanceFootPositionInWorld = new Point3D32(initialFramePoint);

      FrameQuaternion initialFrameOrientation = new FrameQuaternion(initialStanceFootPose.getOrientation());
      initialFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.stanceFootOrientationInWorld = new Quaternion32(initialFrameOrientation);

      FramePoint3D goalFramePoint = new FramePoint3D(goalPose.getPosition());
      goalFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.goalPositionInWorld = new Point3D32(goalFramePoint);

      FrameQuaternion goalFrameOrientation = new FrameQuaternion(goalPose.getOrientation());
      goalFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.goalOrientationInWorld = new Quaternion32(goalFrameOrientation);

      message.requestedFootstepPlannerType = requestedPlannerType.toByte();
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation towards the given endpoint. Set the id
    * of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation)
   {
      PelvisOrientationTrajectoryMessage message = createPelvisOrientationTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame());
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                                             ReferenceFrame trajectoryFrame)
   {
      PelvisOrientationTrajectoryMessage message = createPelvisOrientationTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   public static WholeBodyTrajectoryToolboxMessage createWholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                                                                           List<WaypointBasedTrajectoryMessage> endEffectorTrajectories,
                                                                                           List<ReachingManifoldMessage> reachingManifolds,
                                                                                           List<RigidBodyExplorationConfigurationMessage> explorationConfigurations)
   {
      WholeBodyTrajectoryToolboxMessage message = createWholeBodyTrajectoryToolboxMessage();
      message.configuration = configuration;
      MessageTools.copyData(endEffectorTrajectories, message.endEffectorTrajectories);
      MessageTools.copyData(reachingManifolds, message.reachingManifolds);
      MessageTools.copyData(explorationConfigurations, message.explorationConfigurations);
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(BDIRobotBehavior atlasRobotBehavior)
   {
      BDIBehaviorCommandPacket message = createBDIBehaviorCommandPacket();
      message.atlasBDIRobotBehavior = atlasRobotBehavior.toByte();
      return message;
   }

   public static AtlasElectricMotorEnablePacket createAtlasElectricMotorEnablePacket(AtlasElectricMotorPacketEnum motorEnableEnum, boolean enable)
   {
      AtlasElectricMotorEnablePacket message = createAtlasElectricMotorEnablePacket();
      message.atlasElectricMotorPacketEnumEnable = motorEnableEnum.toByte();
      message.enable = enable;
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(SO3TrajectoryMessage so3Trajectory)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage();
      message.so3Trajectory = new SO3TrajectoryMessage(so3Trajectory);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     long trajectoryReferenceFrameID)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryReferenceFrameID);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed the supplied frame.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly quaternion, ReferenceFrame dataFrame,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage(trajectoryTime, quaternion, trajectoryFrame);
      message.so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataFrame));
      return message;
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame dataFrame,
                                                                   ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = createHeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      message.so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataFrame));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = createHeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                   long trajectoryReferenceFrameId)
   {
      HeadTrajectoryMessage message = createHeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryReferenceFrameId);
      return message;
   }

   /**
    * set a single point
    * 
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrameId the frame id the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             long trajectoryReferenceFrameId)
   {
      EuclideanTrajectoryMessage message = createEuclideanTrajectoryMessage();
      Vector3D zeroLinearVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints.add().set(createEuclideanTrajectoryPointMessage(trajectoryTime, desiredPosition, zeroLinearVelocity));
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   /**
    * set a single point
    * 
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrame the frame the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, trajectoryReferenceFrame.getNameBasedHashCode());
   }

   public static MessageOfMessages createMessageOfMessages(List<Packet<?>> messages)
   {
      MessageOfMessages message = new MessageOfMessages();
      message.packets.clear();
      message.packets.addAll(messages);
      return message;
   }

   public static MessageOfMessages createMessageOfMessages(Packet<?>... messages)
   {
      MessageOfMessages message = new MessageOfMessages();
      message.packets.clear();
      for (Packet<?> packet : messages)
      {
         message.packets.add(packet);
      }
      return message;
   }

   public static LocalizationPacket createLocalizationPacket(boolean reset, boolean toggle)
   {
      LocalizationPacket message = createLocalizationPacket();
      message.reset = reset;
      message.toggle = toggle;
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in data frame
    * @param trajectoryReferenceFrame the frame in which the height will be executed
    * @param dataReferenceFrame the frame the desiredHeight is expressed in, the height will be
    *           changed to the trajectory frame on the controller side
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight,
                                                                                   ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                          trajectoryReferenceFrame.getNameBasedHashCode());
      message.euclideanTrajectory.frameInformation.setDataReferenceFrameId(MessageTools.toFrameId(dataReferenceFrame));
      message.euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      message.euclideanTrajectory.selectionMatrix.xSelected = false;
      message.euclideanTrajectory.selectionMatrix.ySelected = false;
      message.euclideanTrajectory.selectionMatrix.zSelected = true;
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are
    * set to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                          ReferenceFrame.getWorldFrame());
      message.euclideanTrajectory.frameInformation.setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      message.euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      message.euclideanTrajectory.selectionMatrix.xSelected = false;
      message.euclideanTrajectory.selectionMatrix.ySelected = false;
      message.euclideanTrajectory.selectionMatrix.zSelected = true;
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint}
    * for each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage()
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory.selectionMatrix.xSelected = false;
      message.euclideanTrajectory.selectionMatrix.ySelected = false;
      message.euclideanTrajectory.selectionMatrix.zSelected = true;
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex)
   {
      FootstepStatusMessage message = createFootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = null;
      message.actualFootOrientationInWorld = null;
      message.robotSide = -1;
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld)
   {
      FootstepStatusMessage message = createFootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;

      message.robotSide = -1;
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = createFootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D desiredFootPositionInWorld,
                                                            Quaternion desiredFootOrientationInWorld, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = createFootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = desiredFootPositionInWorld;
      message.desiredFootOrientationInWorld = desiredFootOrientationInWorld;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      return createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame.getNameBasedHashCode());
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      SO3TrajectoryMessage message = createSO3TrajectoryMessage();
      Vector3D zeroAngularVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints.add().set(createSO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, zeroAngularVelocity));
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static HighLevelStateMessage createHighLevelStateMessage(HighLevelControllerName highLevelControllerName)
   {
      HighLevelStateMessage message = createHighLevelStateMessage();
      message.highLevelControllerName = highLevelControllerName.toByte();
      return message;
   }

   public static WallPosePacket createWallPosePacket(WallPosePacket other)
   {
      WallPosePacket message = createWallPosePacket();
      message.setCuttingRadius(other.getCuttingRadius());
      message.setCenterPosition(other.getCenterPosition());
      message.setCenterOrientation(other.getCenterOrientation());
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, QuaternionReadOnly centerOrientation)
   {
      WallPosePacket message = createWallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.setCenterPosition(centerPosition);
      message.setCenterOrientation(centerOrientation);
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, RotationMatrixReadOnly rotationMatrix)
   {
      WallPosePacket message = createWallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.setCenterPosition(centerPosition);
      message.setCenterOrientation(new Quaternion(rotationMatrix));
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep,
                                                                           double thetaStart, List<FootstepDataMessage> goals)
   {
      FootstepPlanRequestPacket message = createFootstepPlanRequestPacket();
      message.footstepPlanRequestType = requestType.toByte();
      message.startFootstep = startFootstep;
      message.thetaStart = thetaStart;
      MessageTools.copyData(goals, message.goals);
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep,
                                                                           double thetaStart, List<FootstepDataMessage> goals, double maxSuboptimality)
   {
      FootstepPlanRequestPacket message = createFootstepPlanRequestPacket();
      message.footstepPlanRequestType = requestType.toByte();
      message.startFootstep = startFootstep;
      message.thetaStart = thetaStart;
      MessageTools.copyData(goals, message.goals);
      message.maxSuboptimality = maxSuboptimality;
      return message;
   }

   public static HandJointAnglePacket createHandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      HandJointAnglePacket message = createHandJointAnglePacket();
      message.robotSide = robotSide.toByte();
      message.jointAngles.add(jointAngles);
      message.connected = connected;
      message.calibrated = calibrated;
      return message;
   }

   public static HandCollisionDetectedPacket createHandCollisionDetectedPacket(RobotSide robotSide, int collisionSeverityLevelZeroToThree)
   {
      HandCollisionDetectedPacket message = createHandCollisionDetectedPacket();
      message.robotSide = robotSide.toByte();
      message.collisionSeverityLevelOneToThree = MathTools.clamp(collisionSeverityLevelZeroToThree, 1, 3);
      return message;
   }

   public static AtlasLowLevelControlModeMessage createAtlasLowLevelControlModeMessage(AtlasLowLevelControlMode request)
   {
      AtlasLowLevelControlModeMessage message = createAtlasLowLevelControlModeMessage();
      message.requestedAtlasLowLevelControlMode = request.toByte();
      return message;
   }

   public static HandLoadBearingMessage createHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage message = createHandLoadBearingMessage();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static BehaviorControlModeResponsePacket createBehaviorControlModeResponsePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModeResponsePacket message = createBehaviorControlModeResponsePacket();
      message.behaviorControlModeEnumRequest = requestedControl.toByte();
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.time = trajectoryPoint.getTime();
      message.position = trajectoryPoint.getPosition();
      message.velocity = trajectoryPoint.getVelocity();
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(double time, double position, double velocity)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.time = time;
      message.position = position;
      message.velocity = velocity;
      return message;
   }

   public static StateEstimatorModePacket createStateEstimatorModePacket(StateEstimatorMode requestedOperatingMode)
   {
      StateEstimatorModePacket message = createStateEstimatorModePacket();
      message.requestedStateEstimatorMode = requestedOperatingMode.toByte();
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FullHumanoidRobotModel fullRobotModel)
   {
      return MessageTools.createKinematicsToolboxOutputStatus(fullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel),
                                                              false);
   }

   public static HumanoidBehaviorTypePacket createHumanoidBehaviorTypePacket(HumanoidBehaviorType behaviorType)
   {
      HumanoidBehaviorTypePacket message = createHumanoidBehaviorTypePacket();
      message.humanoidBehaviorType = behaviorType.toByte();
      return message;
   }

   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double finalTransferDuration)
   {
      return createFootstepDataListMessage(footstepDataList, 0.0, 0.0, finalTransferDuration, ExecutionMode.OVERRIDE);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, ExecutionMode executionMode)
   {
      return createFootstepDataListMessage(footstepDataList, defaultSwingDuration, defaultTransferDuration, defaultTransferDuration, executionMode);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, double finalTransferDuration,
                                                                       ExecutionMode executionMode)
   {
      FootstepDataListMessage message = createFootstepDataListMessage();
      MessageTools.copyData(footstepDataList, message.footstepDataList);
      message.defaultSwingDuration = defaultSwingDuration;
      message.defaultTransferDuration = defaultTransferDuration;
      message.finalTransferDuration = finalTransferDuration;
      message.queueingProperties.setExecutionMode(executionMode.toByte());
      message.queueingProperties.setPreviousMessageId(Packet.VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to
    * OVERRIDE
    * 
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    */
   public static FootstepDataListMessage createFootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration)
   {
      return createFootstepDataListMessage(defaultSwingDuration, defaultTransferDuration, defaultTransferDuration);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to
    * OVERRIDE
    * 
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    */
   public static FootstepDataListMessage createFootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration,
                                                                       double finalTransferDuration)
   {
      FootstepDataListMessage message = createFootstepDataListMessage();
      message.defaultSwingDuration = defaultSwingDuration;
      message.defaultTransferDuration = defaultTransferDuration;
      message.finalTransferDuration = finalTransferDuration;
      message.queueingProperties.setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      message.queueingProperties.setPreviousMessageId(Packet.VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   /**
    * Creates a message with the desired grasp to be performed. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide refers to which hand will perform the grasp.
    * @param handDesiredConfiguration refers to the desired grasp.
    */
   public static HandDesiredConfigurationMessage createHandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration handDesiredConfiguration)
   {
      HandDesiredConfigurationMessage message = createHandDesiredConfigurationMessage();
      message.robotSide = robotSide.toByte();
      message.desiredHandConfiguration = handDesiredConfiguration.toByte();
      return message;
   }

   public static FisheyePacket createFisheyePacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position,
                                                   QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      FisheyePacket message = createFisheyePacket();
      message.videoPacket = createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters);
      return message;
   }

   public static VideoPacket createVideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation,
                                               IntrinsicParameters intrinsicParameters)
   {
      return createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters, null);
   }

   public static VideoPacket createVideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation,
                                               IntrinsicParameters intrinsicParameters, PacketDestination packetDestination)
   {
      VideoPacket message = createVideoPacket();
      if (packetDestination != null)
         message.setDestination(packetDestination);
      message.videoSource = videoSource.toByte();
      message.timeStamp = timeStamp;
      message.data.add(data);
      message.position = new Point3D(position);
      message.orientation = new Quaternion(orientation);
      message.intrinsicParameters = toIntrinsicParametersMessage(intrinsicParameters);
      return message;
   }

   public static LocalVideoPacket createLocalVideoPacket(long timeStamp, BufferedImage image, IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket message = new LocalVideoPacket();
      message.timeStamp = timeStamp;
      message.image = image;
      message.intrinsicParameters = toIntrinsicParametersMessage(intrinsicParameters);
      return message;
   }

   /**
    * 
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param pause
    */
   public static PauseWalkingMessage createPauseWalkingMessage(boolean pause)
   {
      PauseWalkingMessage message = createPauseWalkingMessage();
      message.pause = pause;
      return message;
   }

   public static ReachingManifoldMessage createReachingManifoldMessage(RigidBody rigidBody)
   {
      ReachingManifoldMessage message = new ReachingManifoldMessage();
      message.endEffectorNameBasedHashCode = rigidBody.getNameBasedHashCode();
      return message;
   }

   public static WholeBodyTrajectoryToolboxConfigurationMessage createWholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses)
   {
      return createWholeBodyTrajectoryToolboxConfigurationMessage(numberOfInitialGuesses, -1);
   }

   public static WholeBodyTrajectoryToolboxConfigurationMessage createWholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses,
                                                                                                                     int maximumExpansionSize)
   {
      WholeBodyTrajectoryToolboxConfigurationMessage message = createWholeBodyTrajectoryToolboxConfigurationMessage();
      message.numberOfInitialGuesses = numberOfInitialGuesses;
      message.maximumExpansionSize = maximumExpansionSize;
      return message;
   }

   public static SO3TrajectoryPointMessage createSO3TrajectoryPointMessage(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      SO3TrajectoryPointMessage message = new SO3TrajectoryPointMessage();
      message.time = time;
      message.orientation = new Quaternion(orientation);
      message.angularVelocity = new Vector3D(angularVelocity);
      return message;
   }

   public static ArmDesiredAccelerationsMessage createArmDesiredAccelerationsMessage(RobotSide robotSide, double[] armDesiredJointAccelerations)
   {
      ArmDesiredAccelerationsMessage message = createArmDesiredAccelerationsMessage();
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(armDesiredJointAccelerations);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    * 
    * @param desiredJointAccelerations
    */
   public static SpineDesiredAccelerationsMessage createSpineDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      SpineDesiredAccelerationsMessage message = createSpineDesiredAccelerationsMessage();
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of controlled joints.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      JointspaceTrajectoryMessage message = createJointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
         message.jointTrajectoryMessages.add().set(createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]));
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of controlled joints.
    * @param weights the qp weights for the joint accelerations
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      JointspaceTrajectoryMessage message = createJointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
         oneDoFJointTrajectoryMessage.setWeight(weights[jointIndex]);
         message.jointTrajectoryMessages.add().set(oneDoFJointTrajectoryMessage);
      }
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param oneDoFJointTrajectoryMessages joint trajectory points to be executed.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(OneDoFJointTrajectoryMessage[] oneDoFJointTrajectoryMessages)
   {
      JointspaceTrajectoryMessage message = createJointspaceTrajectoryMessage();
      MessageTools.copyData(oneDoFJointTrajectoryMessages, message.jointTrajectoryMessages);
      return message;
   }

   public static SimpleCoactiveBehaviorDataPacket createSimpleCoactiveBehaviorDataPacket(String key, double value)
   {
      SimpleCoactiveBehaviorDataPacket message = createSimpleCoactiveBehaviorDataPacket();
      message.key.append(key);
      message.value = value;
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(boolean stop)
   {
      BDIBehaviorCommandPacket message = createBDIBehaviorCommandPacket();
      message.stop = stop;
      return message;
   }

   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(SimpleTrajectoryPoint1DList trajectoryData)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      int numberOfPoints = trajectoryData.getNumberOfTrajectoryPoints();

      for (int i = 0; i < numberOfPoints; i++)
      {
         SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(i);
         message.trajectoryPoints.add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
      }
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point.
    * 
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.trajectoryPoints.add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryTime, desiredPosition, 0.0));
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point.
    * 
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    * @param weight the weight for the qp
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition, double weight)
   {
      OneDoFJointTrajectoryMessage message = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredPosition);
      message.weight = weight;
      return message;
   }

   public static AtlasDesiredPumpPSIPacket createAtlasDesiredPumpPSIPacket(int desiredPumpPsi)
   {
      AtlasDesiredPumpPSIPacket message = createAtlasDesiredPumpPSIPacket();
      message.desiredPumpPsi = desiredPumpPsi;
      return message;
   }

   public static AtlasElectricMotorAutoEnableFlagPacket createAtlasElectricMotorAutoEnableFlagPacket(boolean shouldAutoEnable)
   {
      AtlasElectricMotorAutoEnableFlagPacket message = createAtlasElectricMotorAutoEnableFlagPacket();
      message.shouldAutoEnable = shouldAutoEnable;
      return message;
   }

   public static EuclideanTrajectoryPointMessage createEuclideanTrajectoryPointMessage(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      EuclideanTrajectoryPointMessage message = new EuclideanTrajectoryPointMessage();
      message.time = time;
      message.position = new Point3D(position);
      message.linearVelocity = new Vector3D(linearVelocity);
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(WalkToGoalAction action)
   {
      WalkToGoalBehaviorPacket message = createWalkToGoalBehaviorPacket();
      message.walkToGoalAction = action.toByte();
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
   {
      WalkToGoalBehaviorPacket message = createWalkToGoalBehaviorPacket();
      message.walkToGoalAction = WalkToGoalAction.FIND_PATH.toByte();
      message.xGoal = xGoal;
      message.yGoal = yGoal;
      message.thetaGoal = thetaGoal;
      message.goalRobotSide = goalSide.toByte();
      return message;
   }

   public static BDIBehaviorStatusPacket createBDIBehaviorStatusPacket(BDIRobotBehavior currentBehavior)
   {
      BDIBehaviorStatusPacket message = createBDIBehaviorStatusPacket();
      message.currentBDIRobotBehavior = currentBehavior.toByte();
      return message;
   }

   public static SpineTrajectoryMessage createSpineTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      SpineTrajectoryMessage message = createSpineTrajectoryMessage();
      message.jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      message.setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number
    *           of joints.
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds)
   {
      SpineTrajectoryMessage message = createSpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, jointDesireds);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number
    *           of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds, double[] weights)
   {
      SpineTrajectoryMessage message = createSpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, jointDesireds, weights);
      return message;
   }

   public static AutomaticManipulationAbortMessage createAutomaticManipulationAbortMessage(boolean enable)
   {
      AutomaticManipulationAbortMessage message = createAutomaticManipulationAbortMessage();
      message.enable = enable;
      return message;
   }

   public static StampedPosePacket createStampedPosePacket(String frameId, TimeStampedTransform3D transform, double confidenceFactor)
   {
      StampedPosePacket message = createStampedPosePacket();
      message.frameId.append(frameId);
      message.pose = new Pose3D(transform.getTransform3D());
      message.timeStamp = transform.getTimeStamp();
      message.confidenceFactor = confidenceFactor;
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 long trajectoryReferenceFrameId)
   {
      SE3TrajectoryMessage message = createSE3TrajectoryMessage();
      Vector3D zeroLinearVelocity = new Vector3D();
      Vector3D zeroAngularVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints.add().set(createSE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity,
                                                                                  zeroAngularVelocity));
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame.getNameBasedHashCode());
   }

   public static DetectedObjectPacket createDetectedObjectPacket(Pose3D pose, int id)
   {
      DetectedObjectPacket message = createDetectedObjectPacket();
      message.pose = pose;
      message.id = id;
      return message;
   }

   public static WalkingControllerFailureStatusMessage createWalkingControllerFailureStatusMessage(Vector2D fallingDirection)
   {
      WalkingControllerFailureStatusMessage message = createWalkingControllerFailureStatusMessage();
      message.fallingDirection = new Vector2D32(fallingDirection);
      return message;
   }

   public static AtlasWristSensorCalibrationRequestPacket createAtlasWristSensorCalibrationRequestPacket(RobotSide robotSide)
   {
      AtlasWristSensorCalibrationRequestPacket message = createAtlasWristSensorCalibrationRequestPacket();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, double trajectoryTime)
   {
      GoHomeMessage message = createGoHomeMessage();
      HumanoidMessageTools.checkRobotSide(bodyPart);
      message.humanoidBodyPart = bodyPart.toByte();
      message.trajectoryTime = trajectoryTime;
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, RobotSide robotSide, double trajectoryTime)
   {
      GoHomeMessage message = createGoHomeMessage();
      if (robotSide == null)
         HumanoidMessageTools.checkRobotSide(bodyPart);
      message.humanoidBodyPart = bodyPart.toByte();
      message.robotSide = robotSide.toByte();
      message.trajectoryTime = trajectoryTime;
      return message;
   }

   public static SE3TrajectoryPointMessage createSE3TrajectoryPointMessage(double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                                                           Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      SE3TrajectoryPointMessage message = new SE3TrajectoryPointMessage();
      message.time = time;
      message.position = new Point3D(position);
      message.orientation = new Quaternion(orientation);
      message.linearVelocity = new Vector3D(linearVelocity);
      message.angularVelocity = new Vector3D(angularVelocity);
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation)
   {
      return createFootstepDataMessage(robotSide, new Point3D(location), new Quaternion(orientation), null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               ArrayList<Point2D> predictedContactPoints)
   {
      return createFootstepDataMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType,
                                                               double swingHeight)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               List<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      FootstepDataMessage message = createFootstepDataMessage();
      message.robotSide = robotSide.toByte();
      message.location = location;
      message.orientation = orientation;
      MessageTools.copyData(predictedContactPoints, message.predictedContactPoints);
      message.trajectoryType = trajectoryType.toByte();
      message.swingHeight = swingHeight;
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(Footstep footstep)
   {
      FootstepDataMessage message = createFootstepDataMessage();

      message.robotSide = footstep.getRobotSide().toByte();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.location = new Point3D(location);
      message.orientation = new Quaternion(orientation);
      MessageTools.copyData(footstep.getPredictedContactPoints(), message.predictedContactPoints);
      message.trajectoryType = footstep.getTrajectoryType().toByte();
      message.swingHeight = footstep.getSwingHeight();
      message.swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();

      if (footstep.getCustomPositionWaypoints().size() != 0)
      {
         for (int i = 0; i < footstep.getCustomPositionWaypoints().size(); i++)
         {
            FramePoint3D framePoint = footstep.getCustomPositionWaypoints().get(i);
            framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            message.customPositionWaypoints.add().set(framePoint);
         }
      }

      return message;
   }

   public static PlanOffsetStatus createPlanOffsetStatus(Vector3DReadOnly offsetVector)
   {
      PlanOffsetStatus message = createPlanOffsetStatus();
      message.offsetVector.set(offsetVector);
      return message;
   }

   /**
    * set the class you want to clear
    * 
    * @param clazz the class you want to clear
    */
   public static ClearDelayQueueMessage createClearDelayQueueMessage(Class<? extends Packet<?>> clazz)
   {
      ClearDelayQueueMessage message = createClearDelayQueueMessage();
      message.classSimpleNameBasedHashCode = clazz.getSimpleName().hashCode();
      return message;
   }

   public static LocalizationStatusPacket createLocalizationStatusPacket(double overlap, String status)
   {
      LocalizationStatusPacket message = createLocalizationStatusPacket();
      message.overlap = overlap;
      message.status.append(status);
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket(RigidBodyTransform valveTransformToWorld, double valveRadius)
   {
      ValveLocationPacket message = createValveLocationPacket();
      message.valvePoseInWorld = new Pose3D(valveTransformToWorld);
      message.valveRadius = valveRadius;
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket(Pose3D valvePoseInWorld, double valveRadius)
   {
      ValveLocationPacket message = createValveLocationPacket();
      message.valvePoseInWorld = valvePoseInWorld;
      message.valveRadius = valveRadius;
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      FootTrajectoryMessage message = createFootTrajectoryMessage(robotSide);
      message.se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation)
   {
      FootTrajectoryMessage message = createFootTrajectoryMessage(robotSide);
      message.se3Trajectory = HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation,
                                                                              ReferenceFrame.getWorldFrame());
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, FramePose3D desiredPose)
   {
      return createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation());
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide)
   {
      FootTrajectoryMessage message = createFootTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static PrepareForLocomotionMessage createPrepareForLocomotionMessage(boolean prepareManipulation, boolean preparePelvis)
   {
      PrepareForLocomotionMessage message = createPrepareForLocomotionMessage();
      message.prepareManipulation = prepareManipulation;
      message.preparePelvis = preparePelvis;
      return message;
   }

   public static void checkRobotSide(HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }

   public static IntrinsicParametersMessage toIntrinsicParametersMessage(IntrinsicParameters intrinsicParameters)
   {
      IntrinsicParametersMessage intrinsicParametersMessage = createIntrinsicParametersMessage();
      intrinsicParametersMessage.width = intrinsicParameters.width;
      intrinsicParametersMessage.height = intrinsicParameters.height;
      intrinsicParametersMessage.fx = intrinsicParameters.fx;
      intrinsicParametersMessage.fy = intrinsicParameters.fy;
      intrinsicParametersMessage.skew = intrinsicParameters.skew;
      intrinsicParametersMessage.cx = intrinsicParameters.cx;
      intrinsicParametersMessage.cy = intrinsicParameters.cy;
      if (intrinsicParameters.radial != null)
         intrinsicParametersMessage.radial.add(intrinsicParameters.radial);
      intrinsicParametersMessage.t1 = intrinsicParameters.t1;
      intrinsicParametersMessage.t2 = intrinsicParameters.t2;
      return intrinsicParametersMessage;
   }

   public static IntrinsicParameters toIntrinsicParameters(IntrinsicParametersMessage message)
   {
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = message.width;
      intrinsicParameters.height = message.height;
      intrinsicParameters.fx = message.fx;
      intrinsicParameters.fy = message.fy;
      intrinsicParameters.skew = message.skew;
      intrinsicParameters.cx = message.cx;
      intrinsicParameters.cy = message.cy;
      if (!message.radial.isEmpty())
         intrinsicParameters.radial = message.radial.toArray();
      intrinsicParameters.t1 = message.t1;
      intrinsicParameters.t2 = message.t2;
      return intrinsicParameters;
   }

   public static void setGroundQuadTreeSupport(PointCloudWorldPacket pointCloudWorldPacket, Point3DReadOnly[] pointCloud)
   {
      pointCloudWorldPacket.groundQuadTreeSupport.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         pointCloudWorldPacket.groundQuadTreeSupport.add((float) point.getX());
         pointCloudWorldPacket.groundQuadTreeSupport.add((float) point.getY());
         pointCloudWorldPacket.groundQuadTreeSupport.add((float) point.getZ());
      }
   }

   public static void setDecayingWorldScan(PointCloudWorldPacket pointCloudWorldPacket, Point3DReadOnly[] pointCloud)
   {
      pointCloudWorldPacket.decayingWorldScan.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         pointCloudWorldPacket.decayingWorldScan.add((float) point.getX());
         pointCloudWorldPacket.decayingWorldScan.add((float) point.getY());
         pointCloudWorldPacket.decayingWorldScan.add((float) point.getZ());
      }
   }

   public static Point3D32[] getDecayingWorldScan(PointCloudWorldPacket pointCloudWorldPacket)
   {
      int numberOfPoints = pointCloudWorldPacket.decayingWorldScan.size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(pointCloudWorldPacket.decayingWorldScan.get(3 * i));
         point.setY(pointCloudWorldPacket.decayingWorldScan.get(3 * i + 1));
         point.setZ(pointCloudWorldPacket.decayingWorldScan.get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   public static HeightQuadTreeToolboxRequestMessage clearRequest(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage clearMessage = createHeightQuadTreeToolboxRequestMessage();
      clearMessage.setDestination(destination);
      clearMessage.requestClearQuadTree = true;
      clearMessage.requestQuadTreeUpdate = false;
      return clearMessage;
   }

   public static HeightQuadTreeToolboxRequestMessage requestQuadTreeUpdate(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage requestMessage = createHeightQuadTreeToolboxRequestMessage();
      requestMessage.setDestination(destination);
      requestMessage.requestClearQuadTree = false;
      requestMessage.requestQuadTreeUpdate = true;
      return requestMessage;
   }

   public static void packLocalizationPointMap(LocalizationPointMapPacket localizationPointMapPacket, Point3DReadOnly[] pointCloud)
   {
      localizationPointMapPacket.localizationPointMap.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         localizationPointMapPacket.localizationPointMap.add((float) point.getX());
         localizationPointMapPacket.localizationPointMap.add((float) point.getY());
         localizationPointMapPacket.localizationPointMap.add((float) point.getZ());
      }
   }

   public static Point3D32[] unpackLocalizationPointMap(LocalizationPointMapPacket localizationPointMapPacket)
   {
      int numberOfPoints = localizationPointMapPacket.localizationPointMap.size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(localizationPointMapPacket.localizationPointMap.get(3 * i));
         point.setY(localizationPointMapPacket.localizationPointMap.get(3 * i + 1));
         point.setZ(localizationPointMapPacket.localizationPointMap.get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, ReferenceFrame referenceFrame)
   {
      long expectedId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != referenceFrame.getNameBasedHashCode() && expectedId != referenceFrame.getAdditionalNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " + referenceFrame.getNameBasedHashCode() + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, long otherReferenceFrameId)
   {
      long expectedId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != otherReferenceFrameId)
      {
         String msg = "Argument's hashcode " + otherReferenceFrameId + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static long getDataFrameIDConsideringDefault(FrameInformation frameInformation)
   {
      long dataId = frameInformation.getDataReferenceFrameId();
      if (dataId == NameBasedHashCodeTools.DEFAULT_HASHCODE)
      {
         dataId = frameInformation.getTrajectoryReferenceFrameId();
      }
      return dataId;
   }

   public static double unpackJointAngle(HandJointAnglePacket handJointAnglePacket, HandJointName jointName)
   {
      int index = jointName.getIndex(RobotSide.fromByte(handJointAnglePacket.robotSide));
      if (index == -1)
      {
         return 0;
      }

      return handJointAnglePacket.jointAngles.get(index);
   }

   public static void packFootSupportPolygon(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotSide, FrameConvexPolygon2d footPolygon)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();

      if (numberOfVertices > CapturabilityBasedStatus.MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = CapturabilityBasedStatus.MAXIMUM_NUMBER_OF_VERTICES;
      }

      if (robotSide == RobotSide.LEFT)
      {
         capturabilityBasedStatus.leftFootSupportPolygon.clear();
      }
      else
      {
         capturabilityBasedStatus.rightFootSupportPolygon.clear();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         if (robotSide == RobotSide.LEFT)
         {
            footPolygon.getVertex(i, capturabilityBasedStatus.leftFootSupportPolygon.add());
         }
         else
         {
            footPolygon.getVertex(i, capturabilityBasedStatus.rightFootSupportPolygon.add());
         }
      }
   }

   public static FrameConvexPolygon2d unpackFootSupportPolygon(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && capturabilityBasedStatus.leftFootSupportPolygon.size() > 0)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), capturabilityBasedStatus.leftFootSupportPolygon.toArray());
      else if (capturabilityBasedStatus.rightFootSupportPolygon != null)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), capturabilityBasedStatus.rightFootSupportPolygon.toArray());
      else
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame());
   }

   public static boolean unpackIsInDoubleSupport(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      return capturabilityBasedStatus.leftFootSupportPolygon.size() != 0 & capturabilityBasedStatus.rightFootSupportPolygon.size() != 0;
   }

   public static boolean unpackIsSupportFoot(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotside)
   {
      if (robotside == RobotSide.LEFT)
         return capturabilityBasedStatus.leftFootSupportPolygon.size() != 0;
      else
         return capturabilityBasedStatus.rightFootSupportPolygon.size() != 0;
   }

   public static void packManifold(ReachingManifoldMessage reachingManifoldMessage, byte[] configurationSpaces, double[] lowerLimits, double[] upperLimits)
   {
      if (configurationSpaces.length != lowerLimits.length || configurationSpaces.length != upperLimits.length || lowerLimits.length != upperLimits.length)
         throw new RuntimeException("Inconsistent array lengths: configurationSpaces = " + configurationSpaces.length);

      reachingManifoldMessage.manifoldConfigurationSpaceNames.reset();
      reachingManifoldMessage.manifoldLowerLimits.reset();
      reachingManifoldMessage.manifoldUpperLimits.reset();
      reachingManifoldMessage.manifoldConfigurationSpaceNames.add(configurationSpaces);
      reachingManifoldMessage.manifoldLowerLimits.add(lowerLimits);
      reachingManifoldMessage.manifoldUpperLimits.add(upperLimits);
   }

   public static Pose3D unpackPose(WaypointBasedTrajectoryMessage waypointBasedTrajectoryMessage, double time)
   {
      if (time <= 0.0)
         return waypointBasedTrajectoryMessage.waypoints.get(0);

      else if (time >= waypointBasedTrajectoryMessage.waypointTimes.get(waypointBasedTrajectoryMessage.waypointTimes.size() - 1))
         return waypointBasedTrajectoryMessage.waypoints.getLast();

      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = waypointBasedTrajectoryMessage.waypointTimes.size();

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - waypointBasedTrajectoryMessage.waypointTimes.get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         Pose3D poseOne = waypointBasedTrajectoryMessage.waypoints.get(indexOfFrame - 1);
         Pose3D poseTwo = waypointBasedTrajectoryMessage.waypoints.get(indexOfFrame);

         double timeOne = waypointBasedTrajectoryMessage.waypointTimes.get(indexOfFrame - 1);
         double timeTwo = waypointBasedTrajectoryMessage.waypointTimes.get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         Pose3D ret = new Pose3D();
         ret.interpolate(poseOne, poseTwo, alpha);

         return ret;
      }
   }

   public static double unpackTrajectoryTime(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      double trajectoryTime = 0.0;
      for (int i = 0; i < jointspaceTrajectoryMessage.jointTrajectoryMessages.size(); i++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.jointTrajectoryMessages.get(i);
         if (oneDoFJointTrajectoryMessage != null)
         {
            trajectoryTime = Math.max(trajectoryTime, oneDoFJointTrajectoryMessage.trajectoryPoints.getLast().time);
         }
      }
      return trajectoryTime;
   }
}
