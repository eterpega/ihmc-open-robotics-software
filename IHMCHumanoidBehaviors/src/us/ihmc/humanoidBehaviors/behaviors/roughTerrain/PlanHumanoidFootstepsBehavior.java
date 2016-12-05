package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.YoTimer;
import us.ihmc.tools.io.printing.PrintTools;

public class PlanHumanoidFootstepsBehavior extends AbstractBehavior
{
   private final String prefix = getClass().getSimpleName();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private FootstepStatus latestFootstepStatus = null;
   private final EnumYoVariable<FootstepStatus.Status> latestFootstepStatusEnum;

   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(40);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(40);

   private final IntegerYoVariable planarRegionsListCount = new IntegerYoVariable(prefix + "PlanarRegionsListCount", registry);
   private final BooleanYoVariable requestedPlanarRegion = new BooleanYoVariable(prefix + "RequestedPlanarRegion", registry);
   private final DoubleYoVariable shorterGoalLength = new DoubleYoVariable(prefix + "ShorterGoalLength", registry);
   private final IntegerYoVariable failIndex = new IntegerYoVariable(prefix + "FailIndex", registry);
   private final IntegerYoVariable indexOfNextFootstepToSendFromCurrentPlan = new IntegerYoVariable(prefix + "NextFootstepToSendFromCurrentPlan", registry);
   private final IntegerYoVariable lengthOfCurrentPlan = new IntegerYoVariable(prefix + "LengthOfCurrentPlan", registry);
   private final BooleanYoVariable firstFootstepSent = new BooleanYoVariable(prefix + "FirstFootstepSent", registry);

   private final EnumYoVariable<RobotSide> currentlySwingingFoot;
   private final EnumYoVariable<RobotSide> nextSideToSwing;

   private final SimplePlanarRegionBipedalAnytimeFootstepPlanner footstepPlanner;
   private FootstepPlan currentPlan = null;

   private final YoFramePose footstepPlannerInitialStepPose;
   private final YoFramePose footstepPlannerGoalPose;

   private final FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
   private final FramePose goalPose = new FramePose();
   private final BooleanYoVariable doneTakingSteps;

   private final FramePose leftFootPose = new FramePose();
   private final FramePose rightFootPose = new FramePose();
   private final FramePose tempStanceFootPose = new FramePose();
   private final FramePose tempFirstFootstepPose = new FramePose();
   private final Point3d tempFootstepPosePosition = new Point3d();
   private final Quat4d tempFirstFootstepPoseOrientation = new Quat4d();
   private final YoTimer timeSinceNewPlanarRegionsTimer, timeInState;
   private boolean plannerThreadStarted = false;

   private final DoubleYoVariable swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);
   private final IntegerYoVariable maxNumberOfStepsToTake = new IntegerYoVariable(prefix + "NumberOfStepsToTake", registry);

   public PlanHumanoidFootstepsBehavior(DoubleYoVariable yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
                                        HumanoidReferenceFrames referenceFrames, FiducialDetectorBehaviorService fiducialDetectorBehaviorService)
   {
      super(PlanHumanoidFootstepsBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      shorterGoalLength.set(Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      footstepPlanner = createFootstepPlanner();

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);
      currentlySwingingFoot = new EnumYoVariable<>("currentlySwingingFoot", registry, RobotSide.class, true);

      timeInState = new YoTimer(yoTime);
      timeSinceNewPlanarRegionsTimer = new YoTimer(yoTime);

      footstepPlannerGoalPose = new YoFramePose(prefix + "FootstepGoalPose", ReferenceFrame.getWorldFrame(), registry);
      footstepPlannerInitialStepPose = new YoFramePose(prefix + "InitialStepPose", ReferenceFrame.getWorldFrame(), registry);
      doneTakingSteps = new BooleanYoVariable(prefix + "DoneTakingSteps", registry);

      latestFootstepStatusEnum = new EnumYoVariable<FootstepStatus.Status>("footstepStatus", registry, FootstepStatus.Status.class);

      behaviorCommunicationBridge.attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);

      requestedPlanarRegion.set(false);

      swingTime.set(1.5);
      transferTime.set(0.3);
      maxNumberOfStepsToTake.set(3);
      firstFootstepSent.set(false);
   }

   private SimplePlanarRegionBipedalAnytimeFootstepPlanner createFootstepPlanner()
   {
      SimplePlanarRegionBipedalAnytimeFootstepPlanner planner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(registry);
      //      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner(registry);
      BipedalFootstepPlannerParameters parameters = planner.getParameters();

      parameters.setMaximumStepReach(0.65); //0.55); //(0.4);
      parameters.setMaximumStepZ(0.25); //0.4); //0.25);

      // Atlas has ankle pitch range of motion limits, which hit when taking steps forward and down. Similar to a human.
      // Whereas a human gets on its toes nicely to avoid the limits, this is challenging with a robot. 
      // So for now, have really conservative forward and down limits on height.
      parameters.setMaximumStepXWhenForwardAndDown(0.2);
      parameters.setMaximumStepZWhenForwardAndDown(0.10);

      parameters.setMaximumStepYaw(0.15); //0.25);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.95);

      parameters.setWiggleInsideDelta(0.02); //0.08);
      parameters.setMaximumXYWiggleDistance(1.0);
      parameters.setMaximumYawWiggle(0.1);

      parameters.setRejectIfCannotFullyWiggleInside(true);

      double idealFootstepLength = 0.45; //0.3; //0.4;
      double idealFootstepWidth = 0.26; //0.2; //0.25;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      planner.setMaximumNumberOfNodesToExpand(500);
      return planner;
   }

   public void createAndAttachYoVariableServerListenerToPlanner(LogModelProvider logModelProvider, FullRobotModel fullRobotModel)
   {
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = footstepPlanner.getFootPolygonsInSoleFrame();
      PlanarRegionBipedalFootstepPlannerVisualizer listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory
            .createWithYoVariableServer(0.01, fullRobotModel, logModelProvider, footPolygonsInSoleFrame);

      footstepPlanner.setBipedalFootstepPlannerListener(listener);
   }

   public void setGoalPoseAndFirstSwingSide(FramePose goalPose, RobotSide swingSide)
   {
      this.nextSideToSwing.set(swingSide);
      this.currentPlan = null;
      this.goalPose.set(goalPose);
      setGoalAndInitialStanceFootToBeClosestToGoal(goalPose);
   }

   public FootstepDataListMessage getFootstepDataListMessageForPlan(int startIndex, int maxNumberOfStepsToTake, double swingTime, double transferTime)
   {
      FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(currentPlan, startIndex, maxNumberOfStepsToTake, swingTime, transferTime);
      return footstepDataListMessage;
   }

   private boolean checkForNewFootstepStatus()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         latestFootstepStatus = footstepStatusQueue.getLatestPacket();

         RobotSide statusSide = latestFootstepStatus.getRobotSide();
         if(statusSide != null)
            nextSideToSwing.set(statusSide.getOppositeSide());

         if (latestFootstepStatus.getStatus() == FootstepStatus.Status.STARTED)
         {
            currentlySwingingFoot.set(latestFootstepStatus.getRobotSide());
         }
         else
         {
            currentlySwingingFoot.set(null);
         }

         PrintTools.info("New footstep status, " + latestFootstepStatus.getStatus());
         return true;
      }

      return false;
   }

   private void notifyPlannerOfNewFootstepStatus()
   {
      SimpleFootstep footstepFromLatest = currentPlan.getFootstep(indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue() - 1);
      footstepPlanner.executingFootstep(footstepFromLatest);
   }

   private void requestAndUpdatePlanarRegions()
   {
      if (!requestedPlanarRegion.getBooleanValue() || (timeSinceNewPlanarRegionsTimer.totalElapsed() > 5.0))
      {
         clearAndRequestPlanarRegionsList();
         requestedPlanarRegion.set(true);
         timeSinceNewPlanarRegionsTimer.reset();
      }

      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         PrintTools.info("Settings new planar regions");
         planarRegionsListCount.increment();

         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         footstepPlanner.setPlanarRegions(planarRegionsList);

         timeSinceNewPlanarRegionsTimer.reset();
      }
   }

   private void clearAndRequestPlanarRegionsList()
   {
      planarRegionsListQueue.getLatestPacket();

      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
   }

   private void checkForBetterPlanAndSendFootsteps()
   {
      FootstepPlan latestPlan = footstepPlanner.getBestPlanYet();

      if (latestPlan != null)
      {
         PrintTools.info("Got better plan!");
         currentPlan = latestPlan;
         indexOfNextFootstepToSendFromCurrentPlan.set(0);
      }

      TextToSpeechPacket p1 = new TextToSpeechPacket("Sending a footstep, " + indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue() + " of current plan");
      sendPacket(p1);

      FootstepDataListMessage footstepDataListMessage = getFootstepDataListMessageForPlan(indexOfNextFootstepToSendFromCurrentPlan.getIntegerValue(),
                                                                                          1, swingTime.getDoubleValue(),
                                                                                          transferTime.getDoubleValue());
      indexOfNextFootstepToSendFromCurrentPlan.increment();
      walkingStatusQueue.clear();

      footstepDataListMessage.setDestination(PacketDestination.UI);
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(footstepDataListMessage);
   }

   @Override
   public void doControl()
   {
      if (!plannerThreadStarted)
      {
         PrintTools.info("Starting planning thread");
         new Thread(footstepPlanner).start();
         plannerThreadStarted = true;
      }

      requestAndUpdatePlanarRegions();
      boolean newFootstepStatus = checkForNewFootstepStatus();

      if(!firstFootstepSent.getBooleanValue() && timeInState.totalElapsed() > 2.0)
      {
         checkForBetterPlanAndSendFootsteps();
         firstFootstepSent.set(true);
      }

      if (newFootstepStatus)
      {
         switch (latestFootstepStatus.getStatus())
         {
         case STARTED:
            notifyPlannerOfNewFootstepStatus();
            break;
         case COMPLETED:
            checkForBetterPlanAndSendFootsteps();
            break;
         }
      }
   }

   private void setGoalAndInitialStanceFootToBeClosestToGoal(FramePose goalPose)
   {
      leftFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.LEFT));
      rightFootPose.setToZero(referenceFrames.getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3d temp = new Point3d();
      Point3d pointBetweenFeet = new Point3d();
      Point3d goalPosition = new Point3d();
      Point3d shorterGoalPosition = new Point3d();
      Vector3d vectorFromFeetToGoal = new Vector3d();

      leftFootPose.getPosition(temp);
      pointBetweenFeet.set(temp);
      rightFootPose.getPosition(temp);
      pointBetweenFeet.add(temp);
      pointBetweenFeet.scale(0.5);

      goalPose.getPosition(goalPosition);
      vectorFromFeetToGoal.sub(goalPosition, pointBetweenFeet);

      if (vectorFromFeetToGoal.length() > shorterGoalLength.getDoubleValue())
      {
         vectorFromFeetToGoal.scale(shorterGoalLength.getDoubleValue() / vectorFromFeetToGoal.length());
      }

      shorterGoalPosition.set(pointBetweenFeet);
      shorterGoalPosition.add(vectorFromFeetToGoal);
      goalPose.setPosition(shorterGoalPosition);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle4d goalOrientation = new AxisAngle4d(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);

      RobotSide stanceSide = nextSideToSwing.getEnumValue().getOppositeSide();

      if (stanceSide == RobotSide.LEFT)
      {
         tempStanceFootPose.set(leftFootPose);
         goalPose.setZ(leftFootPose.getZ());
      }
      else
      {
         tempStanceFootPose.set(rightFootPose);
         goalPose.setZ(rightFootPose.getZ());
      }

      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);

      Point2d xyGoal = new Point2d();
      xyGoal.setX(goalPose.getX());
      xyGoal.setY(goalPose.getY());
      double distanceFromXYGoal = 1.0;
      footstepPlannerGoal.setXYGoal(xyGoal, distanceFromXYGoal);
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.CLOSE_TO_XY_POSITION);

      sendPacketToUI(new UIPositionCheckerPacket(new Point3d(xyGoal.getX(), xyGoal.getY(), leftFootPose.getZ()), new Quat4d()));

      footstepPlanner.setGoal(footstepPlannerGoal);
      footstepPlanner.setInitialStanceFoot(tempStanceFootPose, stanceSide);

      footstepPlannerGoalPose.set(goalPose);
      footstepPlannerInitialStepPose.set(tempStanceFootPose);
   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int startIndex, int maxNumberOfStepsToTake, double swingTime,
                                                                  double transferTime)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setSwingTime(swingTime);
      footstepDataListMessage.setTransferTime(transferTime);
      int numSteps = plan.getNumberOfSteps();
      int lastStepIndex = Math.min(startIndex + maxNumberOfStepsToTake + 1, numSteps);
      for (int i = 1 + startIndex; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
         tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3d(tempFootstepPosePosition),
                                                                            new Quat4d(tempFirstFootstepPoseOrientation));
         firstFootstepMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         footstepDataListMessage.add(firstFootstepMessage);
      }

      footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
      return footstepDataListMessage;
   }

   @Override
   public void onBehaviorEntered()
   {
      timeInState.start();
      timeSinceNewPlanarRegionsTimer.start();
      requestedPlanarRegion.set(false);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   private static ConvexPolygon2d createDefaultFootPolygon()
   {
      //TODO: Get this from the robot model itself.
      double footLength = 0.26;
      double footWidth = 0.18;

      ConvexPolygon2d footPolygon = new ConvexPolygon2d();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   private static SideDependentList<ConvexPolygon2d> createDefaultFootPolygons()
   {
      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createDefaultFootPolygon());
      return footPolygons;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
      footstepPlanner.requestStop();
      plannerThreadStarted = false;
   }
}
