package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers.DiagnosticControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers.DiagnosticMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers.ManualDiagnosticController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is the top level for the diagnostic section of the high-level controller.
 * <p>
 * {@code DiagnosticHighLevelManagerState} only provides a state machine allowing to switch between
 * different types of diagnostics.
 * </p>
 */
public class DiagnosticHighLevelManagerState extends HighLevelControllerState
{
   private static final HighLevelControllerName stateEnum = HighLevelControllerName.DIAGNOSTICS;

   private final List<DiagnosticControllerState> allStates = new ArrayList<>();
   private final GenericStateMachine<DiagnosticMode, DiagnosticControllerState> stateMachine;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   public DiagnosticHighLevelManagerState(HighLevelControllerParameters parameters, HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super(stateEnum, parameters, controllerToolbox);

      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      YoDouble yoTime = controllerToolbox.getYoTime();
      double controlDT = controllerToolbox.getControlDT();
      stateMachine = createStateMachine(controlledJoints, fullRobotModel, parameters, yoTime, controlDT);

      OneDoFJoint[] controllableOneDoFJoints = fullRobotModel.getControllableOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllableOneDoFJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controllableOneDoFJoints);
   }

   /**
    * This is where new diagnostics should be registered to the state machine to enable them on the robot.
    */
   private GenericStateMachine<DiagnosticMode, DiagnosticControllerState> createStateMachine(OneDoFJoint[] controlledJoints,
                                                                                             FullHumanoidRobotModel fullRobotModel,
                                                                                             HighLevelControllerParameters parameters, YoDouble yoTime,
                                                                                             double controlDT)
   {
      GenericStateMachine<DiagnosticMode, DiagnosticControllerState> stateMachine = new GenericStateMachine<>("diagnosticMode", "diagnosticModeSwitchTime",
                                                                                                              DiagnosticMode.class, yoTime, registry);

      ManualDiagnosticController manualDiagnosticController = new ManualDiagnosticController(controlledJoints, fullRobotModel, yoTime, controlDT, parameters,
                                                                                             registry);
      stateMachine.addState(manualDiagnosticController);
      allStates.add(manualDiagnosticController);

      stateMachine.setCurrentState(DiagnosticMode.MANUAL);

      return stateMachine;
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public void initialize()
   {
      for (int i = 0; i < allStates.size(); i++)
         allStates.get(i).initializeControlGains(getStateSpecificJointSettings());

      hasBeenInitialized.set(true);
   }

   @Override
   public void doAction()
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      lowLevelOneDoFJointDesiredDataHolder.overwriteWith(stateMachine.getCurrentState().getOutputForLowLevelController());
      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void doTransitionIntoAction()
   {
      stateMachine.getCurrentState().doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      stateMachine.getCurrentState().doTransitionOutOfAction();
   }
}
