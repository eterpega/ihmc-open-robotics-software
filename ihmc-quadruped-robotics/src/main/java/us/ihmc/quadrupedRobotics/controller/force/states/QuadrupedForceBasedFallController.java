package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSoleWaypointController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedForceBasedFallController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public enum FallBehaviorType
   {
      FREEZE, GO_HOME_Z, GO_HOME_XYZ, DO_NOTHING
   }

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.35);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.5);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.00);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final BooleanParameter useForceFeedbackControlParameter = parameterFactory.createBoolean("useForceFeedbackControl", false);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

   // YoVariables
   private final YoBoolean yoUseForceFeedbackControl;
   private final YoEnum<FallBehaviorType> fallBehaviorType = YoEnum.create("fallBehaviorType", FallBehaviorType.class, registry);
   private final YoPID3DGains yoPositionControllerGains;

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists = new QuadrantDependentList<>();
   private final QuadrupedSoleWaypointController quadrupedSoleWaypointController;
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedReferenceFrames referenceFrames;
   private final FramePoint3D solePositionSetpoint;
   private final Vector3D zeroVelocity;
   private FullQuadrupedRobotModel fullRobotModel;

   public QuadrupedForceBasedFallController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.fallBehaviorType.set(FallBehaviorType.GO_HOME_XYZ);
      quadrupedSoleWaypointController = controllerToolbox.getSoleWaypointController();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      referenceFrames = controllerToolbox.getReferenceFrames();
      solePositionSetpoint = new FramePoint3D();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         QuadrupedSoleWaypointList quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
         quadrupedSoleWaypointList.add(new SoleWaypoint());
         quadrupedSoleWaypointList.add(new SoleWaypoint());
         quadrupedSoleWaypointLists.set(quadrant, quadrupedSoleWaypointList);
      }
      zeroVelocity = new Vector3D(0, 0, 0);
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      this.taskSpaceController = controllerToolbox.getTaskSpaceController();
      fullRobotModel = environment.getFullRobotModel();

      yoPositionControllerGains = new DefaultYoPID3DGains("positionControllerGains", GainCoupling.NONE, true, registry);
      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      updateGains();
      // Create sole waypoint trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.setIncludingFrame(taskSpaceEstimates.getSolePosition(quadrant));
         solePositionSetpoint.changeFrame(referenceFrames.getBodyFrame());
         quadrupedSoleWaypointLists.get(quadrant).get(0).set(solePositionSetpoint, zeroVelocity, 0.0);
         switch (fallBehaviorType.getEnumValue())
         {
         case GO_HOME_XYZ:
            solePositionSetpoint.set(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0),
                  quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
            solePositionSetpoint.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
            break;
         case GO_HOME_Z:
            solePositionSetpoint.setZ(-stanceHeightParameter.get());
            break;
         case DO_NOTHING:
            solePositionProportionalGainsParameter.set(0, 0.0);
            solePositionProportionalGainsParameter.set(1, 0.0);
            solePositionProportionalGainsParameter.set(2, 0.0);
            updateGains();
            break;
         }
         quadrupedSoleWaypointLists.get(quadrant).get(1).set(solePositionSetpoint, zeroVelocity, trajectoryTimeParameter.get());
      }
      quadrupedSoleWaypointController.initialize(quadrupedSoleWaypointLists, yoPositionControllerGains, taskSpaceEstimates, false);

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();

      // Initialize force feedback
      yoUseForceFeedbackControl.set(useForceFeedbackControlParameter.get());
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(useForceFeedbackControlParameter.get());
         }
      }
   }

   @Override
   public ControllerEvent process()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      boolean success = quadrupedSoleWaypointController.compute(taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
      return success ? null : ControllerEvent.DONE;
   }

   @Override
   public void onExit()
   {
      yoUseForceFeedbackControl.set(true);
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(yoUseForceFeedbackControl.getBooleanValue());
         }
      }
   }

   private void updateGains()
   {
      yoPositionControllerGains.setProportionalGains(solePositionProportionalGainsParameter.get());
      yoPositionControllerGains.setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
      yoPositionControllerGains.setDerivativeGains(solePositionDerivativeGainsParameter.get());
   }
}
