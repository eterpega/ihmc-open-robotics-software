package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gnu.trove.map.hash.THashMap;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlannerStateMachine.PlannerState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

// FIXME this entire class is a hack since the walking high level state is never passed down to the planner
/**
 * This class determines the planner states based on the footstep plan that it receives
 * Also remembers the planner state (CoM, ICP, CMP and CoP) so that re-planning is possible 
 * without screwing up stuff
 * @author Apoorv S
 */
public class SmoothCMPBasedICPPlannerStateMachine
{
   // TODO sync the planner states with the walking high level state machine 
   public enum PlannerState
   {
      STANDING, INITIAL_TRANSFER, SWING, TRANSFER, FINAL_TRANSFER, FLAMINGO_STANCE
   }

   private final RecyclingArrayList<FootstepData> footstepDataList;
   private final ArrayList<PlannerState> stateArray = new ArrayList<>();
   private final YoInteger numberOfFootstepsToPlan;
   private final YoInteger numberOfPlannedFootsteps;

   private final FramePoint3D copPosition = new FramePoint3D();
   private final FrameVector3D copVelocity = new FrameVector3D();
   private final FrameVector3D copAcceleration = new FrameVector3D();
   private final FrameVector3D centroidalAngularMomentum = new FrameVector3D();
   private final FrameVector3D centroidalTorque = new FrameVector3D();
   private final FramePoint3D cmpPosition = new FramePoint3D();
   private final FrameVector3D cmpVelocity = new FrameVector3D();
   private final FrameVector3D cmpAcceleration = new FrameVector3D();
   private final FramePoint3D icpPosition = new FramePoint3D();
   private final FrameVector3D icpVelocity = new FrameVector3D();
   private final FrameVector3D icpAcceleration = new FrameVector3D();
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();

   private final YoBoolean isStanding;
   private final YoBoolean isInitialTransfer;
   private final YoBoolean isDoubleSupport;
   private final YoInteger numberOfFootstepsToConsider;
   
   private final YoEnum<PlannerState> previousPlanState;
   private final YoEnum<PlannerState> currentPlanState;
   private final YoEnum<PlannerState> nextPlanState;
   private final THashMap<PlannerState, ArrayList<PlannerState>> feasibleNextStateMap = new THashMap<>();

   public SmoothCMPBasedICPPlannerStateMachine(String namePrefix, YoBoolean isStanding, YoBoolean isInitialTransfer, YoBoolean isDoubleSupport, YoInteger numberFootstepsToConsider,
                                               RecyclingArrayList<FootstepData> footstepDataList, YoVariableRegistry registry)
   {
      String fullPrefix = namePrefix + "StateMachine";
      this.numberOfFootstepsToPlan = new YoInteger(fullPrefix + "NumberOfFootstepsToPlan", registry);
      this.numberOfPlannedFootsteps = new YoInteger(fullPrefix + "NumberOfFootstepPlanned", registry);

      this.isStanding = isStanding;
      this.isInitialTransfer = isInitialTransfer;
      this.isDoubleSupport = isDoubleSupport;
      this.numberOfFootstepsToConsider = numberFootstepsToConsider;
      this.footstepDataList = footstepDataList;
      
      this.previousPlanState = new YoEnum<>(fullPrefix + "PreviousPlannerState", registry, PlannerState.class);
      this.currentPlanState = new YoEnum<>(fullPrefix + "CurrentPlannerState", registry, PlannerState.class);
      this.nextPlanState = new YoEnum<>(fullPrefix + "NextPlannerState", registry, PlannerState.class);

      this.feasibleNextStateMap.put(PlannerState.STANDING, new ArrayList<>(Arrays.asList(PlannerState.INITIAL_TRANSFER)));
      this.feasibleNextStateMap.put(PlannerState.INITIAL_TRANSFER, new ArrayList<>(Arrays.asList(PlannerState.FLAMINGO_STANCE, PlannerState.SWING)));
      this.feasibleNextStateMap.put(PlannerState.SWING, new ArrayList<>(Arrays.asList(PlannerState.TRANSFER, PlannerState.FINAL_TRANSFER)));
      this.feasibleNextStateMap.put(PlannerState.TRANSFER, new ArrayList<>(Arrays.asList(PlannerState.FLAMINGO_STANCE, PlannerState.SWING)));
      this.feasibleNextStateMap.put(PlannerState.FINAL_TRANSFER, new ArrayList<>(Arrays.asList(PlannerState.STANDING)));
      this.feasibleNextStateMap.put(PlannerState.FLAMINGO_STANCE, new ArrayList<>(Arrays.asList(PlannerState.TRANSFER, PlannerState.FINAL_TRANSFER)));
   }

   public void initializeStateMachine()
   {
      previousPlanState.set(PlannerState.STANDING);
      currentPlanState.set(PlannerState.STANDING);
      nextPlanState.set(PlannerState.INITIAL_TRANSFER);
   }

   public void clear()
   {
      footstepDataList.clear();
      stateArray.clear();
      numberOfFootstepsToPlan.set(0);
   }

   /**
    * Get current desired CoP position
    * @return
    */
   public FramePoint3D getCoPPosition()
   {
      return copPosition;
   }

   public FrameVector3D getCoPVelocity()
   {
      return copVelocity;
   }

   public FrameVector3D getCoPAcceleration()
   {
      return comAcceleration;
   }

   public FramePoint3D getCMPPosition()
   {
      return cmpPosition;
   }

   public FrameVector3D getCMPVelocity()
   {
      return cmpVelocity;
   }

   public FrameVector3D getCMPAcceleration()
   {
      return cmpAcceleration;
   }

   public FrameVector3D getAM()
   {
      return centroidalAngularMomentum;
   }

   public FrameVector3D getTorque()
   {
      return centroidalTorque;
   }

   public FramePoint3D getICPPosition()
   {
      return icpPosition;
   }

   public FrameVector3D getICPVelocity()
   {
      return icpVelocity;
   }

   public FrameVector3D getICPAcceleration()
   {
      return icpAcceleration;
   }

   public FramePoint3D getCoMPosition()
   {
      return comPosition;
   }

   public FrameVector3D getCoMVelocity()
   {
      return comVelocity;
   }

   public FrameVector3D getCoMAcceleration()
   {
      return comAcceleration;
   }

   public void updateStateFromGenerators()
   {
      //copTrajectoryGenerator.getDesiredCenterOfPressure(copPosition, copVelocity, copAcceleration);
      //amTrajectoryGenerator.getDesiredAngularMomentum(centroidalAngularMomentum, centroidalTorque);
      //cmpTrajectoryGenerator.getLinearData(cmpPosition, cmpVelocity, cmpAcceleration);
      //icpTrajectoryGenerator.getLinearData(icpPosition, icpVelocity, icpAcceleration);
      //comTrajectoryGenerator.getLinearData(comPosition, comVelocity, comAcceleration);
   }

   public PlannerState deduceNextFootStepFootstepData(PlannerState currentState, int nextFootstepIndex, List<FootstepData> footstepDataList)
   {
      FootstepData nextFootstepData = footstepDataList.get(nextFootstepIndex);
      int numberOfFutureFootsteps = footstepDataList.size() - nextFootstepIndex;
      switch (currentState)
      {
      case STANDING:
         if(numberOfFutureFootsteps != 0)
            return PlannerState.INITIAL_TRANSFER;
         else 
            return PlannerState.STANDING;
      case INITIAL_TRANSFER:
      case TRANSFER:
         if(Double.isFinite(nextFootstepData.getSwingTime()))
            return PlannerState.SWING;
         else
            return PlannerState.FLAMINGO_STANCE;
      case SWING:
      case FLAMINGO_STANCE:
         if(numberOfFutureFootsteps != 1)
            return PlannerState.TRANSFER;
         else
            return PlannerState.FINAL_TRANSFER;
      case FINAL_TRANSFER:
         return PlannerState.STANDING;
      default:
         throw new RuntimeException("Unhandled planner state");
      }
   }
   
   public void updateStateList()
   {
      PlannerState currentState = getCurrentState();
      stateArray.add(currentState);
      numberOfFootstepsToPlan.set(Math.min(footstepDataList.size(), numberOfFootstepsToConsider.getIntegerValue()));
      for(int i = 0; i < numberOfFootstepsToPlan.getIntegerValue(); i++)
      {
         // This assumes two states per footstep
         currentState = deduceNextFootStepFootstepData(currentState, i, footstepDataList);
         currentState = deduceNextFootStepFootstepData(currentState, i, footstepDataList);
      }
      currentState = deduceNextFootStepFootstepData(currentState, (numberOfFootstepsToPlan.getIntegerValue() - 1), footstepDataList);
   }
   
   public void setPlannerFlagsFromState()
   {
      PlannerState currentState = currentPlanState.getEnumValue();
      if (currentState == PlannerState.STANDING)
         isStanding.set(true);
      if (currentState == PlannerState.INITIAL_TRANSFER)
         isInitialTransfer.set(true);
      if (currentState != PlannerState.FLAMINGO_STANCE && currentState != PlannerState.SWING)
         isDoubleSupport.set(true);
   }
   
   public void setPreviousState(PlannerState previousStateToSet)
   {
      previousPlanState.set(previousStateToSet);
   }
   
   public void setCurrentState(PlannerState currentStateToSet)
   {
      currentPlanState.set(currentStateToSet);
   }
   
   public void setNextState(PlannerState nextStateToSet)
   {
      nextPlanState.set(nextStateToSet);
   }

   public PlannerState getPreviousState()
   {
      return previousPlanState.getEnumValue();
   }

   public PlannerState getCurrentState()
   {
      return currentPlanState.getEnumValue();
   }

   public PlannerState getNextState()
   {
      return nextPlanState.getEnumValue();
   }

   public void transitionToNextState()
   {
      setPreviousState(getCurrentState());
      setCurrentState(getNextState());
      setNextState(stateArray.get(1));
   }
   
   public int getNumberOfFootstepsToPlan()
   {
      return numberOfFootstepsToPlan.getIntegerValue();
   }
}
