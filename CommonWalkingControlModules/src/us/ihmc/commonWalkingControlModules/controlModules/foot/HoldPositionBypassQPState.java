package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class HoldPositionBypassQPState extends HoldPositionAbstractState
{
   private final FullyConstrainedState supportState;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelJointDataToAdd = new LowLevelOneDoFJointDesiredDataHolder();

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameOrientation orientationError = new FrameOrientation();
   private final Twist footTwist = new Twist();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame soleFrame;
   private final ReferenceFrame ankleFrame;

   private final OneDoFJoint anklePitchJoint;
   private final OneDoFJoint ankleRollJoint;

   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;

   private final DoubleYoVariable pitchError;
   private final DoubleYoVariable rollError;

   private final TwistCalculator twistCalculator;

   public HoldPositionBypassQPState(FootControlHelper footControlHelper, YoVariableRegistry registry, FullyConstrainedState supportState)
   {
      super(footControlHelper, registry);
      this.supportState = supportState;

      soleFrame = contactableFoot.getSoleFrame();
      ankleFrame = contactableFoot.getFrameAfterParentJoint();

      RobotSide robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getMomentumBasedController().getFullRobotModel();
      anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      ankleRollJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);
      lowLevelJointDataToAdd.registerJointWithEmptyData(anklePitchJoint);
      lowLevelJointDataToAdd.registerJointWithEmptyData(ankleRollJoint);

      String prefix = robotSide.getLowerCaseName() + "Foot";
      proportionalGain = new DoubleYoVariable(prefix + "OrientationGainP", registry);
      derivativeGain = new DoubleYoVariable(prefix + "OrientationGainD", registry);
      pitchError = new DoubleYoVariable(prefix + "PitchError", registry);
      rollError = new DoubleYoVariable(prefix + "RollError", registry);

      twistCalculator = footControlHelper.getMomentumBasedController().getTwistCalculator();
      proportionalGain.set(15.0);
      derivativeGain.set(0.1);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionOutOfAction();
      supportState.doTransitionIntoAction();
      setZeroTorques();

      desiredOrientation.setToZero(soleFrame);
      desiredOrientation.changeFrame(worldFrame);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      supportState.doTransitionOutOfAction();
      setZeroTorques();
   }

   private void setZeroTorques()
   {
      lowLevelJointDataToAdd.setDesiredJointTorque(anklePitchJoint, 0.0);
      lowLevelJointDataToAdd.setDesiredJointTorque(ankleRollJoint, 0.0);
      pitchError.set(0.0);
      rollError.set(0.0);
   }


   @Override
   public void doSpecificAction()
   {
      supportState.doAction();

      orientationError.setIncludingFrame(desiredOrientation);
      orientationError.changeFrame(ankleFrame);

      double pitchError = orientationError.getPitch();
      double rollError = orientationError.getRoll();
      this.pitchError.set(pitchError);
      this.rollError.set(rollError);

      twistCalculator.getTwistOfBody(footTwist, contactableFoot.getRigidBody());
      double pitchSpeed = footTwist.getAngularPartY();
      double rollSpeed = footTwist.getAngularPartX();

      double kp = proportionalGain.getDoubleValue();
      double kd = derivativeGain.getDoubleValue();

      lowLevelJointDataToAdd.setDesiredJointTorque(anklePitchJoint, kp * pitchError - kd * pitchSpeed);
      lowLevelJointDataToAdd.setDesiredJointTorque(ankleRollJoint, kp * rollError - kd * rollSpeed);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return supportState.getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return supportState.getFeedbackControlCommand();
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDataToAdd()
   {
      return lowLevelJointDataToAdd;
   }

   @Override
   public void setWeight(double weight)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void setWeights(Vector3d angular, Vector3d linear)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void setDoSmartHoldPosition(boolean doSmartHold)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void doFootholdAdjustments(boolean doAdjustments)
   {
      // TODO Auto-generated method stub
   }

}
