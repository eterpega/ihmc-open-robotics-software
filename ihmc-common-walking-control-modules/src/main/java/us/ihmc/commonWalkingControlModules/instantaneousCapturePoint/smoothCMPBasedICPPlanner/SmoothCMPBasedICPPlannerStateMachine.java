package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<FootstepData> footstepDataList;

   private final YoBoolean isStanding;
   private final YoBoolean isInitialTransfer;
   private final YoBoolean isDoubleSupport;
   private final YoBoolean planForDoubleSupport;
   private final YoBoolean planForSingleSupport;
   private final YoDouble timeInCurrentState;

   private final GenericStateMachine<PlannerState, ICPPlannerState> stateMachine;
   private final ReferenceCoPTrajectoryGenerator copTrajectoryGenerator;
   private final ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator;
   private final FootstepAngularMomentumPredictor amTrajectoryGenerator;
   private final ReferenceICPTrajectoryGenerator icpTrajectoryGenerator;
   private final ReferenceCoMTrajectoryGenerator comTrajectoryGenerator;

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
   private final FramePoint3D icpInitialPosition = new FramePoint3D();
   private final FrameVector3D icpInitialVelocity = new FrameVector3D();
   private final FrameVector3D icpInitialAcceleration = new FrameVector3D();
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   private double plannerTimeInState = 0.0;

   public SmoothCMPBasedICPPlannerStateMachine(String namePrefix, YoBoolean isStanding, YoBoolean isInitialTransfer, YoBoolean isDoubleSupport,
                                               YoBoolean planForDoubleSupport, YoBoolean planForSingleSupport, YoDouble timeInCurrentState,
                                               ReferenceCoPTrajectoryGenerator copTrajectoryGenerator, ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator,
                                               FootstepAngularMomentumPredictor amTrajectoryGenerator, ReferenceICPTrajectoryGenerator icpTrajectoryGenerator,
                                               ReferenceCoMTrajectoryGenerator comTrajectoryGenerator, RecyclingArrayList<FootstepData> footstepDataList,
                                               YoVariableRegistry registry)
   {
      String fullPrefix = namePrefix + "StateMachine";
      this.isStanding = isStanding;
      this.isInitialTransfer = isInitialTransfer;
      this.isDoubleSupport = isDoubleSupport;
      this.planForDoubleSupport = planForDoubleSupport;
      this.planForSingleSupport = planForSingleSupport;
      this.timeInCurrentState = timeInCurrentState;
      this.footstepDataList = footstepDataList;
      this.copTrajectoryGenerator = copTrajectoryGenerator;
      this.cmpTrajectoryGenerator = cmpTrajectoryGenerator;
      this.amTrajectoryGenerator = amTrajectoryGenerator;
      this.icpTrajectoryGenerator = icpTrajectoryGenerator;
      this.comTrajectoryGenerator = comTrajectoryGenerator;
      this.stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "TimeForPlanner", PlannerState.class, PlannerState.STANDING,
                                                    timeInCurrentState, registry);
      initializeStateMachine();
   }

   private void initializeStateMachine()
   {
      stateMachine.addState(new ICPPlannerStandingState());
      stateMachine.addState(new ICPPlannerToSingleSupportState(PlannerState.INITIAL_TRANSFER));
      stateMachine.addState(new ICPPlannerToSingleSupportState(PlannerState.TRANSFER));
      stateMachine.addState(new ICPPlannerFinalTransfer());
      stateMachine.addState(new ICPPlannerToDoubleSupportState(PlannerState.FLAMINGO_STANCE));
      stateMachine.addState(new ICPPlannerToDoubleSupportState(PlannerState.SWING));
   }

   private void updateStateFromGenerators()
   {
      plannerTimeInState = MathTools.clamp(this.timeInCurrentState.getDoubleValue(), 0.0, copTrajectoryGenerator.getCurrentStateFinalTime());
      copTrajectoryGenerator.update(plannerTimeInState);
      amTrajectoryGenerator.update(plannerTimeInState);
      cmpTrajectoryGenerator.update(plannerTimeInState);
      icpTrajectoryGenerator.compute(plannerTimeInState);
      comTrajectoryGenerator.compute(plannerTimeInState);

      copTrajectoryGenerator.getDesiredCenterOfPressure(copPosition, copVelocity, copAcceleration);
      amTrajectoryGenerator.getDesiredAngularMomentum(centroidalAngularMomentum, centroidalTorque);
      cmpTrajectoryGenerator.getLinearData(cmpPosition, cmpVelocity, cmpAcceleration);
      icpTrajectoryGenerator.getLinearData(icpPosition, icpVelocity, icpAcceleration);
      comTrajectoryGenerator.getLinearData(comPosition, comVelocity, comAcceleration);
   }

   private void setPlannerFlagsFromState()
   {
      PlannerState currentState = stateMachine.getCurrentStateEnum();
      if (currentState == PlannerState.STANDING)
         isStanding.set(true);
      else
         isStanding.set(false);
      if (currentState == PlannerState.INITIAL_TRANSFER)
         isInitialTransfer.set(true);
      else
         isInitialTransfer.set(false);
      if (currentState != PlannerState.FLAMINGO_STANCE && currentState != PlannerState.SWING)
         isDoubleSupport.set(true);
      else
         isDoubleSupport.set(false);
   }

   public PlannerState getPreviousState()
   {
      return stateMachine.getPreviousStateEnum();
   }

   public PlannerState getCurrentState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public void setCurrentState(PlannerState stateToSet)
   {
      stateMachine.setCurrentState(stateToSet);
   }

   public void checkStateTransitions()
   {
      stateMachine.checkTransitionConditions();
      setPlannerFlagsFromState();
   }

   private void setGeneratorInitialStates()
   {
      copTrajectoryGenerator.setInitialCoPPositionForPlan(copPosition);
   }

   public void updateState()
   {
      stateMachine.doAction();
      checkStateTransitions();
   }

   private class ICPPlannerState extends State<PlannerState>
   {
      public ICPPlannerState(PlannerState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         updateStateFromGenerators();
      }

      @Override
      public void doTransitionIntoAction()
      {
         icpTrajectoryGenerator.setICPInitialConditionsForAdjustment(icpInitialPosition, icpInitialVelocity, icpInitialAcceleration);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         PrintTools.debug(getStateEnum() + " Transitioning out time: " + plannerTimeInState);
         icpTrajectoryGenerator.compute(plannerTimeInState);
         icpTrajectoryGenerator.getPosition(icpInitialPosition);
         icpTrajectoryGenerator.getVelocity(icpInitialVelocity);
         icpTrajectoryGenerator.getAcceleration(icpInitialAcceleration);
         PrintTools.debug("Pos: " + icpInitialPosition.toString() + " Vel: " + icpInitialVelocity.toString() + " Acc: " + icpAcceleration.toString());
      }
   }

   private class ICPPlannerStandingState extends ICPPlannerState
   {

      public ICPPlannerStandingState()
      {
         super(PlannerState.STANDING);
         addStateTransition(PlannerState.INITIAL_TRANSFER, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               boolean transitionCondition = footstepDataList.size() != 0;
               if (transitionCondition)
                  planForDoubleSupport.set(false);
               return transitionCondition;
            }
         });
      }

      @Override
      public void doAction()
      {
         copTrajectoryGenerator.getCenterOfDoubleSupportPolygon(copPosition);
         copVelocity.setToZero(worldFrame);
         copAcceleration.setToZero(worldFrame);
         cmpPosition.setIncludingFrame(copPosition);
         cmpVelocity.setToZero(worldFrame);
         cmpAcceleration.setToZero(worldFrame);
         icpPosition.setIncludingFrame(copPosition);
         icpVelocity.setToZero(worldFrame);
         icpAcceleration.setToZero(worldFrame);
         comPosition.setIncludingFrame(copPosition);
         comVelocity.setToZero(worldFrame);
         comAcceleration.setToZero(worldFrame);
         centroidalAngularMomentum.setToZero(worldFrame);
         centroidalTorque.setToZero(worldFrame);
      }
   }

   private class ICPPlannerToSingleSupportState extends ICPPlannerState
   {
      public ICPPlannerToSingleSupportState(PlannerState stateEnum)
      {
         super(stateEnum);
         addStateTransition(PlannerState.SWING, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               boolean transitionToSwing = planForSingleSupport.getBooleanValue() && Double.isFinite(footstepDataList.get(0).getSwingTime());
               if (transitionToSwing)
                  planForSingleSupport.set(false);
               return transitionToSwing;
            }
         });
         addStateTransition(PlannerState.FLAMINGO_STANCE, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               boolean transitionCondition = planForSingleSupport.getBooleanValue() && !Double.isFinite(footstepDataList.get(0).getSwingTime());
               if (transitionCondition)
                  planForSingleSupport.set(false);
               return transitionCondition;
            }
         });
      }
   }

   private class ICPPlannerToDoubleSupportState extends ICPPlannerState
   {
      public ICPPlannerToDoubleSupportState(PlannerState stateEnum)
      {
         super(stateEnum);
         addStateTransition(PlannerState.TRANSFER, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               boolean transitionCondition = planForDoubleSupport.getBooleanValue() && (footstepDataList.size() > 0);
               if (transitionCondition)
                  planForDoubleSupport.set(false);
               return transitionCondition;
            }
         });
         addStateTransition(PlannerState.FINAL_TRANSFER, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               boolean transitionCondition = planForDoubleSupport.getBooleanValue() && (footstepDataList.size() == 0);
               if (transitionCondition)
                  planForDoubleSupport.set(false);
               return transitionCondition;
            }
         });
      }
   }

   private class ICPPlannerFinalTransfer extends ICPPlannerState
   {
      public ICPPlannerFinalTransfer()
      {
         super(PlannerState.FINAL_TRANSFER);
         addStateTransition(PlannerState.STANDING, new StateTransitionCondition()
         {
            @Override
            public boolean checkCondition()
            {
               copTrajectoryGenerator.getDesiredCenterOfPressure(copPosition);
               cmpTrajectoryGenerator.getPosition(cmpPosition);
               icpTrajectoryGenerator.getPosition(icpPosition);
               comTrajectoryGenerator.getPosition(comPosition);
               return copPosition.epsilonEquals(cmpPosition, Epsilons.ONE_THOUSANDTH) && copPosition.epsilonEquals(icpPosition, Epsilons.ONE_THOUSANDTH)
                     && copPosition.epsilonEquals(comPosition, Epsilons.ONE_THOUSANDTH);
            }
         });
      }
   }

   public void getCoPData(YoFramePoint desiredCoPPosition, YoFrameVector desiredCoPVelocity)
   {
      desiredCoPPosition.set(copPosition);
      desiredCoPVelocity.set(copVelocity);
   }

   public void getCentroidalMomentumData(YoFrameVector desiredCentroidalAngularMomentum, YoFrameVector desiredCentroidalTorque)
   {
      desiredCentroidalAngularMomentum.set(centroidalAngularMomentum);
      desiredCentroidalTorque.set(centroidalTorque);
   }

   public void getCMPData(YoFramePoint desiredCMPPosition, YoFrameVector desiredCMPVelocity)
   {
      desiredCMPPosition.set(cmpPosition);
      desiredCMPVelocity.set(cmpVelocity);
   }

   public void getICPData(YoFramePoint desiredICPPosition, YoFrameVector desiredICPVelocity, YoFrameVector desiredICPAcceleration)
   {
      desiredICPPosition.set(icpPosition);
      desiredICPVelocity.set(icpVelocity);
      desiredICPAcceleration.set(icpAcceleration);
   }

   public void getCoMData(YoFramePoint desiredCoMPosition, YoFrameVector desiredCoMVelocity, YoFrameVector desiredCoMAcceleration)
   {
      desiredCoMPosition.set(comPosition);
      desiredCoMVelocity.set(comVelocity);
      desiredCoMAcceleration.set(comAcceleration);
   }
}
