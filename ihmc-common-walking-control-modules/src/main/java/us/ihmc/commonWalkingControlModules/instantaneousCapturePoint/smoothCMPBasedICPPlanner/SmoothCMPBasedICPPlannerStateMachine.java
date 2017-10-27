package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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
      STANDING, INITIAL_TRANSFER, SWING, TRANSFER, FINAL_TRANSFER, FLAMINGO
   }

   private final RecyclingArrayList<FootstepData> footstepDataList = new RecyclingArrayList<>(FootstepData.class);
   private final ArrayList<PlannerState> stateArray = new ArrayList<>();
   private final YoInteger numberOfFootstepsToPlan;
   private final YoInteger numberOfPlannedFootsteps;
   private final YoEnum<PlannerState> plannerState;
   private final YoEnum<PlannerState> previousPlannerState;
   private final YoEnum<PlannerState> nextPlannerState;
   private final ReferenceCoPTrajectoryGenerator copTrajectoryGenerator;
   private final FootstepAngularMomentumPredictor amTrajectoryGenerator;
   private final ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator;
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
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   
   
   public SmoothCMPBasedICPPlannerStateMachine(ReferenceCoPTrajectoryGenerator copTrajectoryGenerator, FootstepAngularMomentumPredictor amTrajectoryGenerator,
                                               ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator, ReferenceICPTrajectoryGenerator icpTrajectoryGenerator,
                                               ReferenceCoMTrajectoryGenerator comTrajectoryGenerator, String namePrefix, YoVariableRegistry registry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      String fullPrefix = namePrefix + "StateMachine";
      this.copTrajectoryGenerator = copTrajectoryGenerator;
      this.amTrajectoryGenerator = amTrajectoryGenerator;
      this.cmpTrajectoryGenerator = cmpTrajectoryGenerator;
      this.icpTrajectoryGenerator = icpTrajectoryGenerator;
      this.comTrajectoryGenerator = comTrajectoryGenerator;
      this.numberOfFootstepsToPlan = new YoInteger(fullPrefix + "NumberOfFootstepsToPlan", registry);
      this.numberOfPlannedFootsteps = new YoInteger(fullPrefix + "NumberOfFootstepPlanned", registry);
      this.plannerState = new YoEnum<>(fullPrefix + "PlannerState", registry, PlannerState.class);
      this.previousPlannerState = new YoEnum<>(fullPrefix + "PreviousPlannerState", registry, PlannerState.class);
      this.nextPlannerState = new YoEnum<>(fullPrefix + "NextPlannerState", registry, PlannerState.class);
   }

   public void initializeStateMachine()
   {
      previousPlannerState.set(PlannerState.STANDING);
      plannerState.set(PlannerState.STANDING);
      nextPlannerState.set(PlannerState.STANDING);
   }

   public void clear()
   {
      footstepDataList.clear();
      stateArray.clear();
      numberOfFootstepsToPlan.set(0);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null && timing != null)
      {
         if (!footstep.getFootstepPose().getReferenceFrame().getTransformToRoot().containsNaN())
         {
            footstepDataList.add().set(footstep, timing);
         }
         else
            PrintTools.warn("Received bad footstep " + footstep.toString() + " " + timing.toString());
      }
      else
         PrintTools.warn("Receieved null footstep / timing");
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
      copTrajectoryGenerator.getDesiredCenterOfPressure(copPosition, copVelocity, copAcceleration);
      amTrajectoryGenerator.getDesiredAngularMomentum(centroidalAngularMomentum, centroidalTorque);
      cmpTrajectoryGenerator.getLinearData(cmpPosition, cmpVelocity, cmpAcceleration);
      icpTrajectoryGenerator.getLinearData(icpPosition, icpVelocity, icpAcceleration);
      comTrajectoryGenerator.getLinearData(comPosition, comVelocity, comAcceleration);
   }
   
   public void setStateForGenerators()
   {
      
   }
}
