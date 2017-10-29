package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlannerStateMachine.PlannerState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SmoothCMPBasedICPPlannerStateMachineTest
{
   private YoVariableRegistry registry;
   private SmoothCMPBasedICPPlannerStateMachine plannerStateMachine;

   private YoBoolean isStanding;
   private YoBoolean isInitialTransfer;
   private YoBoolean isDoubleSupport;
   private YoBoolean planForDoubleSupport;
   private YoBoolean planForSingleSupport;

   private YoDouble timeInCurrentState;
   private RecyclingArrayList<FootstepData> footstepDataList;

   private ReferenceCoPTrajectoryGenerator copTrajectoryGenerator;
   private ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator;
   private ReferenceICPTrajectoryGenerator icpTrajectoryGenerator;

   @Before
   public void setupTest()
   {
      String namePrefix = "ICPPlannerStateMachineTest";
      this.registry = new YoVariableRegistry(namePrefix);
      this.isStanding = new YoBoolean(namePrefix + "IsStanding", registry);
      this.isInitialTransfer = new YoBoolean(namePrefix + "IsInitialTransfer", registry);
      this.isDoubleSupport = new YoBoolean(namePrefix + "isDoubleSupport", registry);

      this.planForDoubleSupport = new YoBoolean(namePrefix + "PlanForDoubleSupport", registry);
      this.planForSingleSupport = new YoBoolean(namePrefix + "PlanForSingleSupport", registry);

      this.timeInCurrentState = new YoDouble(namePrefix + "TimeInCurrentState", registry);
      this.footstepDataList = new RecyclingArrayList<>(FootstepData.class);
      
      YoInteger numberOfFootstepsToConsider = new YoInteger(namePrefix + "NumberOfFootsteps", registry);
      this.copTrajectoryGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, 10, 5, (BipedSupportPolygons)null, new SideDependentList<>(), numberOfFootstepsToConsider, new ArrayList<YoDouble>(), new ArrayList<YoDouble>(),
                                                                        new ArrayList<YoDouble>(), new ArrayList<YoDouble>(), new ArrayList<YoDouble>(),
                                                                        registry);

      this.plannerStateMachine = new SmoothCMPBasedICPPlannerStateMachine(namePrefix + "StateMachine", isStanding, isInitialTransfer, isDoubleSupport,
                                                                          planForDoubleSupport, planForSingleSupport, timeInCurrentState,
                                                                          copTrajectoryGenerator, cmpTrajectoryGenerator, null, icpTrajectoryGenerator, null,
                                                                          footstepDataList, registry);
   }

   @Test
   public void testStandingToInitialTransferTransition()
   {
      footstepDataList.clear();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.STANDING);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.STANDING);
      assertTrue(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      footstepDataList.add().set(new Footstep(), new FootstepTiming());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.INITIAL_TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertTrue(isInitialTransfer.getBooleanValue());
   }

   @Test
   public void testInitialTransferToFlamingoTransition()
   {
      footstepDataList.clear();
      plannerStateMachine.setCurrentState(PlannerState.INITIAL_TRANSFER);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.INITIAL_TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertTrue(isInitialTransfer.getBooleanValue());
      planForSingleSupport.set(true);
      footstepDataList.add().set(new Footstep(), new FootstepTiming(Double.POSITIVE_INFINITY, 0.6));
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.FLAMINGO_STANCE);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.FLAMINGO_STANCE);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
   }

   @Test
   public void testInitialTransferToSwingTransition()
   {
      footstepDataList.clear();
      plannerStateMachine.setCurrentState(PlannerState.INITIAL_TRANSFER);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.INITIAL_TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertTrue(isInitialTransfer.getBooleanValue());
      planForSingleSupport.set(true);
      footstepDataList.add().set(new Footstep(), new FootstepTiming(0.6, 0.2));
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.SWING);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.SWING);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
   }

   @Test
   public void testSwingToTransferTransition()
   {
      footstepDataList.clear();
      plannerStateMachine.setCurrentState(PlannerState.SWING);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.SWING);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      planForDoubleSupport.set(true);
      footstepDataList.add().set(new Footstep(), new FootstepTiming());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
   }

   @Test
   public void testSwingToFinalTransferTransition()
   {
      footstepDataList.clear();
      plannerStateMachine.setCurrentState(PlannerState.SWING);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.SWING);
      assertFalse(isStanding.getBooleanValue());
      assertFalse(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      planForDoubleSupport.set(true);
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.FINAL_TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
      plannerStateMachine.checkStateTransitions();
      assertTrue(plannerStateMachine.getCurrentState() == PlannerState.FINAL_TRANSFER);
      assertFalse(isStanding.getBooleanValue());
      assertTrue(isDoubleSupport.getBooleanValue());
      assertFalse(isInitialTransfer.getBooleanValue());
   }

}
