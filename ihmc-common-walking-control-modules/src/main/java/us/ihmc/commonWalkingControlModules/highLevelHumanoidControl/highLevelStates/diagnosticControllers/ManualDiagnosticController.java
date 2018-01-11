package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ManualDiagnosticController extends DiagnosticControllerState
{
   private static final DiagnosticMode stateEnum = DiagnosticMode.MANUAL;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoEnum<?> selectedJointEnum;

   private final YoBoolean hasSelectedJointChanged = new YoBoolean("hasSelectedJointChanged", registry);
   private final YoBoolean resetFunctionGenerators = new YoBoolean("resetFunctionGenerators", registry);

   private final YoDouble masterScale = new YoDouble("masterScale", registry);

   private final YoDouble kpSelected = new YoDouble("kpSelected", registry);
   private final YoDouble kdSelected = new YoDouble("kdSelected", registry);

   private final YoDouble qDesiredSelected = new YoDouble("qDesiredSelected", registry);
   private final YoDouble qdDesiredSelected = new YoDouble("qdDesiredSelected", registry);
   private final YoDouble qOffsetFunctionSelected = new YoDouble("qOffsetFunctionSelected", registry);
   private final YoDouble qdOffsetFunctionSelected = new YoDouble("qdOffsetFunctionSelected", registry);

   private final YoDouble qSelected = new YoDouble("qSelected", registry);
   private final YoDouble qdSelected = new YoDouble("qdSelected", registry);

   private final YoDouble tauSelected = new YoDouble("tauSelected", registry);
   private final YoDouble tauOffsetSelected = new YoDouble("tauOffsetSelected", registry);
   private final YoDouble tauPDSelected = new YoDouble("tauPDSelected", registry);
   private final YoDouble tauFunctionSelected = new YoDouble("tauFunctionSelected", registry);
   private final YoDouble tauDesiredSelected = new YoDouble("tauDesiredSelected", registry);

   private final YoEnum<YoFunctionGeneratorMode> positionFunctionGenModeSelected = new YoEnum<>("positionFunctionGenModeSelected", registry,
                                                                                                YoFunctionGeneratorMode.class);
   private final YoDouble positionFunctionGenAmplitudeSelected = new YoDouble("positionFunctionGenAmplitudeSelected", registry);
   private final YoDouble positionFunctionGenFrequencySelected = new YoDouble("positionFunctionGenFrequencySelected", registry);

   private final YoEnum<YoFunctionGeneratorMode> tauFunctionGenModeSelected = new YoEnum<>("tauFunctionGenModeSelected", registry,
                                                                                           YoFunctionGeneratorMode.class);
   private final YoDouble tauFunctionGenAmplitudeSelected = new YoDouble("tauFunctionGenAmplitudeSelected", registry);
   private final YoDouble tauFunctionGenFrequencySelected = new YoDouble("tauFunctionGenFrequencySelected", registry);

   private final List<JointDiagnosticDataHolder> jointDiagnosticDataHolders = new ArrayList<>();
   private final JointDesiredOutputList jointDesiredOutputList;

   private final WholeBodySetpointParameters standPrepParameters;

   public ManualDiagnosticController(OneDoFJoint[] controllableOneDoFJoints, FullHumanoidRobotModel fullRobotModel, YoDouble yoTime, double controlDT,
                                     HighLevelControllerParameters highLevelControllerParameters, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      String[] jointNames = Arrays.stream(controllableOneDoFJoints).map(OneDoFJoint::getName).toArray(String[]::new);

      /*
       * Creating YoEnum from String[] is far from ideal, but is probably the current best way to
       * get a generic YoEnum that adapts to the robot model and still be human-readable in
       * SCSVisualizer.
       */
      selectedJointEnum = new YoEnum<>("selectedJoint", "", registry, true, jointNames);
      selectedJointEnum.addVariableChangedListener(v -> hasSelectedJointChanged.set(true));
      selectedJointEnum.set(YoEnum.NULL_VALUE);

      standPrepParameters = highLevelControllerParameters.getStandPrepParameters();

      for (String jointName : jointNames)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
         jointDiagnosticDataHolders.add(new JointDiagnosticDataHolder(joint, yoTime, controlDT));
      }

      jointDesiredOutputList = new JointDesiredOutputList(controllableOneDoFJoints);

      positionFunctionGenModeSelected.set(YoFunctionGeneratorMode.SINE);
      tauFunctionGenModeSelected.set(YoFunctionGeneratorMode.SINE);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initializeControlGains(JointDesiredOutputListReadOnly jointsSettings)
   {
      for (int i = 0; i < jointDiagnosticDataHolders.size(); i++)
      {
         JointDiagnosticDataHolder jointDiagnosticDataHolder = jointDiagnosticDataHolders.get(i);
         OneDoFJoint joint = jointDiagnosticDataHolder.joint;
         JointDesiredOutputReadOnly jointSettings = jointsSettings.getJointDesiredOutput(joint);
         jointDiagnosticDataHolder.positionController.setProportionalGain(jointSettings.getStiffness());
         jointDiagnosticDataHolder.positionController.setDerivativeGain(jointSettings.getDamping());
      }
   }

   @Override
   public void doAction()
   {
      resetFunctionGenerators();
      updateSelectedJoint();

      masterScale.set(MathTools.clamp(masterScale.getDoubleValue(), 0.0, 1.0));

      if (selectedJointEnum.getOrdinal() != YoEnum.NULL_VALUE)
      {
         JointDiagnosticDataHolder selected = jointDiagnosticDataHolders.get(selectedJointEnum.getOrdinal());
         selected.qDesired.set(qDesiredSelected.getDoubleValue());
         selected.qdDesired.set(qdDesiredSelected.getDoubleValue());

         selected.positionController.setProportionalGain(kpSelected.getDoubleValue());
         selected.positionController.setDerivativeGain(kdSelected.getDoubleValue());

         selected.positionFunctionGenerator.setMode(positionFunctionGenModeSelected.getEnumValue());
         selected.positionFunctionGenerator.setAmplitude(positionFunctionGenAmplitudeSelected.getDoubleValue());
         selected.positionFunctionGenerator.setFrequency(positionFunctionGenFrequencySelected.getDoubleValue());

         selected.tauFunctionGenerator.setMode(tauFunctionGenModeSelected.getEnumValue());
         selected.tauFunctionGenerator.setAmplitude(tauFunctionGenAmplitudeSelected.getDoubleValue());
         selected.tauFunctionGenerator.setFrequency(tauFunctionGenFrequencySelected.getDoubleValue());
         selected.tauFunctionGenerator.setOffset(tauOffsetSelected.getDoubleValue());
      }

      for (int i = 0; i < jointDiagnosticDataHolders.size(); i++)
         jointDiagnosticDataHolders.get(i).computeJointDesiredOutput(jointDesiredOutputList.getJointDesiredOutput(i));

      if (selectedJointEnum.getOrdinal() != YoEnum.NULL_VALUE)
      {
         JointDiagnosticDataHolder selected = jointDiagnosticDataHolders.get(selectedJointEnum.getOrdinal());
         qOffsetFunctionSelected.set(selected.qOffsetFunction.getDoubleValue());
         qdOffsetFunctionSelected.set(selected.qdOffsetFunction.getDoubleValue());
         qSelected.set(selected.joint.getQ());
         qdSelected.set(selected.joint.getQd());
         tauSelected.set(selected.joint.getTauMeasured());
         tauPDSelected.set(selected.tauPD.getDoubleValue());
         tauFunctionSelected.set(selected.tauFunction.getDoubleValue());
         tauDesiredSelected.set(selected.tauDesired.getDoubleValue());
      }
   }

   private void updateSelectedJoint()
   {
      if (!hasSelectedJointChanged.getBooleanValue())
         return;

      if (selectedJointEnum.getOrdinal() == YoEnum.NULL_VALUE)
         return;

      JointDiagnosticDataHolder selected = jointDiagnosticDataHolders.get(selectedJointEnum.getOrdinal());
      qDesiredSelected.set(selected.qDesired.getDoubleValue());
      qdDesiredSelected.set(selected.qdDesired.getDoubleValue());

      kpSelected.set(selected.positionController.getProportionalGain());
      kdSelected.set(selected.positionController.getDerivativeGain());

      positionFunctionGenModeSelected.set(selected.positionFunctionGenerator.getMode());
      positionFunctionGenAmplitudeSelected.set(selected.positionFunctionGenerator.getAmplitude());
      positionFunctionGenFrequencySelected.set(selected.positionFunctionGenerator.getFrequency());

      tauFunctionGenModeSelected.set(selected.tauFunctionGenerator.getMode());
      tauFunctionGenAmplitudeSelected.set(selected.tauFunctionGenerator.getAmplitude());
      tauFunctionGenFrequencySelected.set(selected.tauFunctionGenerator.getFrequency());
      tauOffsetSelected.set(selected.tauFunctionGenerator.getOffset());

      hasSelectedJointChanged.set(false);
   }

   private void resetFunctionGenerators()
   {
      if (!resetFunctionGenerators.getBooleanValue())
         return;

      for (int i = 0; i < jointDiagnosticDataHolders.size(); i++)
         jointDiagnosticDataHolders.get(i).resetFunctionGenerators();

      positionFunctionGenModeSelected.set(YoFunctionGeneratorMode.SINE);
      positionFunctionGenAmplitudeSelected.set(0.0);
      positionFunctionGenFrequencySelected.set(0.0);

      tauFunctionGenModeSelected.set(YoFunctionGeneratorMode.SINE);
      tauFunctionGenAmplitudeSelected.set(0.0);
      tauFunctionGenFrequencySelected.set(0.0);

      resetFunctionGenerators.set(false);
   }

   @Override
   public void doTransitionIntoAction()
   {
      resetFunctionGenerators.set(true);

      tauOffsetSelected.set(0.0);

      for (int i = 0; i < jointDiagnosticDataHolders.size(); i++)
      {
         JointDiagnosticDataHolder jointDiagnosticDataHolder = jointDiagnosticDataHolders.get(i);
         OneDoFJoint joint = jointDiagnosticDataHolder.joint;
         jointDiagnosticDataHolder.qDesired.set(standPrepParameters.getSetpoint(joint.getName()));
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   private class JointDiagnosticDataHolder
   {
      private final OneDoFJoint joint;
      private final YoDouble qDesired;
      private final YoDouble qdDesired;
      private final YoDouble qOffsetFunction;
      private final YoDouble qdOffsetFunction;

      private final YoDouble tauFunction;
      private final YoDouble tauPD;
      private final YoDouble tauDesired;

      private final YoFunctionGenerator positionFunctionGenerator;
      private final YoFunctionGenerator tauFunctionGenerator;

      private final PDController positionController;

      public JointDiagnosticDataHolder(OneDoFJoint joint, YoDouble yoTime, double controlDT)
      {
         this.joint = joint;

         String jointName = joint.getName();
         qDesired = new YoDouble("q_d_man_diag_" + jointName, registry);
         qdDesired = new YoDouble("qd_d_man_diag_" + jointName, registry);

         qOffsetFunction = new YoDouble("q_off_fun_man_diag_" + jointName, registry);
         qdOffsetFunction = new YoDouble("qd_off_fun_man_diag_" + jointName, registry);

         tauFunction = new YoDouble("tau_fun_man_diag_" + jointName, registry);
         tauPD = new YoDouble("tau_pd_man_diag_" + jointName, registry);
         tauDesired = new YoDouble("tau_d_man_diag_" + jointName, registry);

         positionFunctionGenerator = new YoFunctionGenerator(jointName + "Position", yoTime, registry, true, controlDT);
         positionFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
         positionFunctionGenerator.setAmplitude(0.0);
         positionFunctionGenerator.setFrequency(0.0);
         positionFunctionGenerator.setOffset(0.0);

         tauFunctionGenerator = new YoFunctionGenerator(jointName + "Tau", yoTime, registry, false, controlDT);
         tauFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
         tauFunctionGenerator.setAmplitude(0.0);
         tauFunctionGenerator.setFrequency(0.0);
         tauFunctionGenerator.setOffset(0.0);

         positionController = new PDController(jointName, registry);
      }

      public void computeJointDesiredOutput(JointDesiredOutput outputToPack)
      {
         qOffsetFunction.set(positionFunctionGenerator.getValue());
         qdOffsetFunction.set(positionFunctionGenerator.getValueDot());

         double q = joint.getQ();
         double q_d = qDesired.getDoubleValue() + qOffsetFunction.getDoubleValue();
         q_d = MathTools.clamp(q_d, joint.getJointLimitLower(), joint.getJointLimitUpper());
         double qd = joint.getQd();
         double qd_d = qdDesired.getDoubleValue() + qdOffsetFunction.getDoubleValue();
         double pdOutput = masterScale.getDoubleValue() * positionController.compute(q, q_d, qd, qd_d);

         tauPD.set(pdOutput);
         tauFunction.set(tauFunctionGenerator.getValue());

         tauDesired.set(tauPD.getDoubleValue() + tauFunction.getDoubleValue());

         outputToPack.setDesiredPosition(q_d);
         outputToPack.setDesiredVelocity(qd_d);
         outputToPack.setDesiredTorque(tauDesired.getDoubleValue());
      }

      public void resetFunctionGenerators()
      {
         positionFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
         positionFunctionGenerator.setAmplitude(0.0);
         positionFunctionGenerator.setFrequency(0.0);
         positionFunctionGenerator.setOffset(0.0);

         tauFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
         tauFunctionGenerator.setAmplitude(0.0);
         tauFunctionGenerator.setFrequency(0.0);
         tauFunctionGenerator.setOffset(0.0);
      }
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return jointDesiredOutputList;
   }
}
