package us.ihmc.sensorProcessing.simulatedSensors;

import static us.ihmc.robotics.math.corruptors.GaussianCorruptorYoFrameVector3D.createGaussianCorruptorYoFrameVector3D;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.FORCE_SENSOR;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_POSITION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_TAU;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.TORQUE_SENSOR;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.corruptors.GaussianCorruptorYoFrameQuaternion;
import us.ihmc.robotics.math.corruptors.GaussianCorruptorYoFrameVector3D;
import us.ihmc.robotics.math.corruptors.GaussianCorruptorYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SensorNoiseSimulator
{
   private static final String PERFECT = "perfect";
   private static final String GAUSSIAN = "gauss";

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LinkedHashMap<OneDoFJoint, YoDouble> inputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> inputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> inputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> inputJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> inputOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputTorques = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> intermediateOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, YoDouble> outputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> outputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> outputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> outputJointTaus = new LinkedHashMap<>();

   private final ArrayList<IMUSensor> inputIMUs = new ArrayList<IMUSensor>();
   private final ArrayList<IMUSensor> outputIMUs = new ArrayList<IMUSensor>();

   private final ForceSensorDataHolder inputForceSensors;
   private final ForceSensorDataHolder outputForceSensors;

   private final List<OneDoFJoint> jointSensorDefinitions;
   private final List<IMUDefinition> imuSensorDefinitions;
   private final List<ForceSensorDefinition> forceSensorDefinitions;

   private final List<String> allJointSensorNames = new ArrayList<>();
   private final List<String> allIMUSensorNames = new ArrayList<>();
   private final List<String> allForceSensorNames = new ArrayList<>();

   private final RotationMatrix tempOrientation = new RotationMatrix();

   private final FrameVector3D tempForce = new FrameVector3D();
   private final FrameVector3D tempTorque = new FrameVector3D();
   private final Wrench tempWrench = new Wrench();

   private final Random random;

   public SensorNoiseSimulator(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, Random random, SensorNoiseParameters sensorNoiseParameters,
                               YoVariableRegistry parentRegistry)
   {
      this.random = random;

      jointSensorDefinitions = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      imuSensorDefinitions = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();
      forceSensorDefinitions = stateEstimatorSensorDefinitions.getForceSensorDefinitions();

      String prefix = null;
      String suffix = null;

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         allJointSensorNames.add(jointName);

         prefix = JOINT_POSITION.getProcessorNamePrefix(PERFECT);
         suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointPosition = new YoDouble(prefix + suffix, registry);
         inputJointPositions.put(oneDoFJoint, rawJointPosition);
         outputJointPositions.put(oneDoFJoint, rawJointPosition);
         processedJointPositions.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_VELOCITY.getProcessorNamePrefix(PERFECT);
         suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointVelocity = new YoDouble(prefix + suffix, registry);
         inputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         outputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         processedJointVelocities.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_ACCELERATION.getProcessorNamePrefix(PERFECT);
         suffix = JOINT_ACCELERATION.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointAcceleration = new YoDouble(prefix + suffix, registry);
         inputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         outputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         processedJointAccelerations.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_TAU.getProcessorNamePrefix(PERFECT);
         suffix = JOINT_TAU.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointTau = new YoDouble(prefix + suffix, registry);
         inputJointTaus.put(oneDoFJoint, rawJointTau);
         outputJointTaus.put(oneDoFJoint, rawJointTau);
         processedJointTaus.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());
      }

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         allIMUSensorNames.add(imuName);
         ReferenceFrame sensorFrame = imuDefinition.getIMUFrame();

         prefix = IMU_ORIENTATION.getProcessorNamePrefix(PERFECT);
         suffix = IMU_ORIENTATION.getProcessorNameSuffix(imuName, -1);
         YoFrameQuaternion rawOrientation = new YoFrameQuaternion(prefix, suffix, worldFrame, registry);
         inputOrientations.put(imuDefinition, rawOrientation);
         intermediateOrientations.put(imuDefinition, rawOrientation);
         processedOrientations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = IMU_ANGULAR_VELOCITY.getProcessorNamePrefix(PERFECT);
         suffix = IMU_ANGULAR_VELOCITY.getProcessorNameSuffix(imuName, -1);
         YoFrameVector rawAngularVelocity = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputAngularVelocities.put(imuDefinition, rawAngularVelocity);
         intermediateAngularVelocities.put(imuDefinition, rawAngularVelocity);
         processedAngularVelocities.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = IMU_LINEAR_ACCELERATION.getProcessorNamePrefix(PERFECT);
         suffix = IMU_LINEAR_ACCELERATION.getProcessorNameSuffix(imuName, -1);
         YoFrameVector rawLinearAcceleration = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         intermediateLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         processedLinearAccelerations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         inputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
         outputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();
         allForceSensorNames.add(sensorName);
         ReferenceFrame sensorFrame = forceSensorDefinition.getSensorFrame();

         prefix = FORCE_SENSOR.getProcessorNamePrefix(PERFECT);
         suffix = FORCE_SENSOR.getProcessorNameSuffix(sensorName, -1);
         YoFrameVector rawForce = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputForces.put(forceSensorDefinition, rawForce);
         intermediateForces.put(forceSensorDefinition, rawForce);
         processedForces.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = TORQUE_SENSOR.getProcessorNamePrefix(PERFECT);
         suffix = TORQUE_SENSOR.getProcessorNameSuffix(sensorName, -1);
         YoFrameVector rawTorque = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputTorques.put(forceSensorDefinition, rawTorque);
         intermediateTorques.put(forceSensorDefinition, rawTorque);
         processedTorques.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());
      }

      inputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);
      outputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);

      sensorNoiseParameters.configureSensorNoiseSimulator(this);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      startComputation();
   }

   public void startComputation()
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);

         updateProcessors(processedJointPositions.get(oneDoFJoint));
         updateProcessors(processedJointVelocities.get(oneDoFJoint));
         updateProcessors(processedJointAccelerations.get(oneDoFJoint));
         updateProcessors(processedJointTaus.get(oneDoFJoint));
      }

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);

         IMUSensor inputIMU = inputIMUs.get(i);
         tempOrientation.set(inputOrientations.get(imuDefinition));
         inputIMU.setOrientationMeasurement(tempOrientation);
         inputIMU.setAngularVelocityMeasurement(inputAngularVelocities.get(imuDefinition));
         inputIMU.setLinearAccelerationMeasurement(inputLinearAccelerations.get(imuDefinition));

         updateProcessors(processedOrientations.get(imuDefinition));
         updateProcessors(processedAngularVelocities.get(imuDefinition));
         updateProcessors(processedLinearAccelerations.get(imuDefinition));

         IMUSensor outputIMU = outputIMUs.get(i);
         tempOrientation.set(intermediateOrientations.get(imuDefinition));
         outputIMU.setOrientationMeasurement(tempOrientation);
         outputIMU.setAngularVelocityMeasurement(intermediateAngularVelocities.get(imuDefinition));
         outputIMU.setLinearAccelerationMeasurement(intermediateLinearAccelerations.get(imuDefinition));
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);

         inputForceSensors.getForceSensorValue(forceSensorDefinition, tempWrench);
         tempWrench.getLinearPartIncludingFrame(tempForce);
         tempWrench.getAngularPartIncludingFrame(tempTorque);
         inputForces.get(forceSensorDefinition).set(tempForce);
         inputTorques.get(forceSensorDefinition).set(tempTorque);

         updateProcessors(processedForces.get(forceSensorDefinition));
         updateProcessors(processedTorques.get(forceSensorDefinition));

         tempForce.setIncludingFrame(intermediateForces.get(forceSensorDefinition));
         tempTorque.setIncludingFrame(intermediateTorques.get(forceSensorDefinition));
         tempWrench.set(tempForce, tempTorque);
         outputForceSensors.setForceSensorValue(forceSensorDefinition, tempWrench);
      }
   }

   private void updateProcessors(List<ProcessingYoVariable> processors)
   {
      for (int j = 0; j < processors.size(); j++)
      {
         processors.get(j).update();
      }
   }

   /**
    * Adds Gaussian noise to the given sensor signal. This is cumulative, by calling this method
    * twice for instance, you will obtain a signal with additional noise.
    * 
    * @param standardDeviation the noise standard deviation.
    * @return a map from sensor name to the index of the noise simulator that was created.
    */
   public Map<String, Integer> addSensorGaussianNoise(DoubleProvider standardDeviation, SensorType sensorType)
   {
      return addSensorGaussianNoiseWithSensorsToIgnore(standardDeviation, sensorType);
   }

   /**
    * Adds Gaussian noise to the given sensor signal. This is cumulative, by calling this method
    * twice for instance, you will obtain a signal with additional noise.
    * 
    * @param sensorsToBeProcessed list of the names of the sensors that need to be processed.
    * @param standardDeviation the noise standard deviation.
    * @return a map from sensor name to the index of the noise simulator that was created.
    */
   public Map<String, Integer> addSensorGaussianNoiseOnlyForSpecifiedSensors(DoubleProvider standardDeviation, SensorType sensorType, String... sensorsToBeProcessed)
   {
      return addSensorGaussianNoiseWithSensorsToIgnore(standardDeviation, sensorType, invertSensorSelection(sensorType, sensorsToBeProcessed));
   }

   /**
    * Adds Gaussian noise to the given sensor signal. This is cumulative, by calling this method
    * twice for instance, you will obtain a signal with additional noise.
    * 
    * @param standardDeviation the noise standard deviation.
    * @param sensorsToIgnore list of the names of the sensors to ignore.
    * @return a map from sensor name to the index of the noise simulator that was created.
    */
   public Map<String, Integer> addSensorGaussianNoiseWithSensorsToIgnore(DoubleProvider standardDeviation, SensorType sensorType, String... sensorsToIgnore)
   {
      Map<String, Integer> processorIDMap;
      List<String> sensorToIgnoreList = new ArrayList<>();
      if (sensorsToIgnore != null && sensorsToIgnore.length > 0)
         sensorToIgnoreList.addAll(Arrays.asList(sensorsToIgnore));

      if (sensorType.isJointSensor())
         processorIDMap = addJointGaussianNoiseWithJointsToIgnore(standardDeviation, sensorType, sensorToIgnoreList);
      else if (sensorType.isWrenchSensor())
         processorIDMap = addForceSensorGaussianNoiseWithSensorsToIgnore(standardDeviation, sensorType, sensorToIgnoreList);
      else if (sensorType.isIMUSensor())
      {
         if (sensorType == SensorType.IMU_ORIENTATION)
            processorIDMap = addIMUOrientationGaussianNoiseWithSensorsToIgnore(standardDeviation, sensorToIgnoreList);
         else
            processorIDMap = addIMUVectorTypeDataGaussianNoise(standardDeviation, sensorType, sensorToIgnoreList);
      }
      else
         throw new RuntimeException("Unknown type of sensor.");

      return Collections.unmodifiableMap(processorIDMap);
   }

   private Map<String, Integer> addJointGaussianNoiseWithJointsToIgnore(DoubleProvider standardDeviation, SensorType sensorType, List<String> jointsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<OneDoFJoint, YoDouble> outputJointSignals = getOutputJointSignals(sensorType);
      LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointSignals = getProcessedJointSignals(sensorType);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointsToIgnore.contains(jointName))
            continue;

         YoDouble intermediateJointSignal = outputJointSignals.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointSignals.get(oneDoFJoint);
         String prefix = sensorType.getProcessorNamePrefix(GAUSSIAN);
         int newProcessorID = processors.size();
         processorsIDs.put(jointName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(jointName, newProcessorID);
         GaussianCorruptorYoVariable filter = new GaussianCorruptorYoVariable(prefix + suffix, registry, random, standardDeviation, intermediateJointSignal);
         processedJointSignals.get(oneDoFJoint).add(filter);

         outputJointSignals.put(oneDoFJoint, filter);
      }

      return processorsIDs;
   }

   private Map<String, Integer> addForceSensorGaussianNoiseWithSensorsToIgnore(DoubleProvider standardDeviation, SensorType sensorType,
                                                                               List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateForceSensorSignals = getIntermediateForceSensorSignals(sensorType);
      LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForceSensorSignals = getProcessedForceSensorSignals(sensorType);

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();

         if (sensorsToIgnore.contains(sensorName))
            continue;

         YoFrameVector intermediateSignal = intermediateForceSensorSignals.get(forceSensorDefinition);
         List<ProcessingYoVariable> processors = processedForceSensorSignals.get(forceSensorDefinition);
         String prefix = sensorType.getProcessorNamePrefix(GAUSSIAN);
         int newProcessorID = processors.size();
         processorsIDs.put(sensorName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(sensorName, newProcessorID);
         GaussianCorruptorYoFrameVector3D filter = createGaussianCorruptorYoFrameVector3D(prefix, suffix, registry, random, standardDeviation,
                                                                                          intermediateSignal);
         processors.add(filter);

         intermediateForceSensorSignals.put(forceSensorDefinition, filter);
      }

      return processorsIDs;
   }

   private Map<String, Integer> addIMUOrientationGaussianNoiseWithSensorsToIgnore(DoubleProvider standardDeviation, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorIDs = new HashMap<>();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorsToIgnore.contains(imuName))
            continue;

         FrameQuaternionReadOnly intermediateOrientation = intermediateOrientations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedOrientations.get(imuDefinition);
         String prefix = IMU_ORIENTATION.getProcessorNamePrefix(GAUSSIAN);
         int newProcessorID = processors.size();
         processorIDs.put(imuName, newProcessorID);
         String suffix = IMU_ORIENTATION.getProcessorNameSuffix(imuName, newProcessorID);
         GaussianCorruptorYoFrameQuaternion filteredOrientation = new GaussianCorruptorYoFrameQuaternion(prefix, suffix, registry, random, standardDeviation,
                                                                                                         intermediateOrientation);
         processors.add(filteredOrientation);

         intermediateOrientations.put(imuDefinition, filteredOrientation);
      }

      return processorIDs;
   }

   private Map<String, Integer> addIMUVectorTypeDataGaussianNoise(DoubleProvider standardDeviation, SensorType sensorType, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<IMUDefinition, YoFrameVector> intermediateIMUVectorTypeSignals = getIntermediateIMUVectorTypeSignals(sensorType);
      LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedIMUVectorTypeSignals = getProcessedIMUVectorTypeSignals(sensorType);

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorsToIgnore.contains(imuName))
            continue;

         YoFrameVector intermediateSignal = intermediateIMUVectorTypeSignals.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedIMUVectorTypeSignals.get(imuDefinition);
         String prefix = sensorType.getProcessorNamePrefix(GAUSSIAN);
         int newProcessorID = processors.size();
         processorsIDs.put(imuName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(imuName, newProcessorID);
         GaussianCorruptorYoFrameVector3D filter = createGaussianCorruptorYoFrameVector3D(prefix, suffix, registry, random, standardDeviation,
                                                                                          intermediateSignal);
         processors.add(filter);

         intermediateIMUVectorTypeSignals.put(imuDefinition, filter);
      }

      return processorsIDs;
   }

   private String[] invertSensorSelection(SensorType sensorType, String... subSelection)
   {
      if (sensorType.isJointSensor())
         return invertSensorSelection(allJointSensorNames, subSelection);
      else if (sensorType.isIMUSensor())
         return invertSensorSelection(allIMUSensorNames, subSelection);
      else if (sensorType.isWrenchSensor())
         return invertSensorSelection(allForceSensorNames, subSelection);
      else
         throw new RuntimeException("Invert selection is not implemented for this type of sensor: sensorType = " + sensorType);
   }

   private String[] invertSensorSelection(List<String> allSensorNames, String... subSelection)
   {
      List<String> invertSelection = new ArrayList<>();
      List<String> originalJointSensorSelectionList = new ArrayList<>();
      if (subSelection != null && subSelection.length > 0)
         originalJointSensorSelectionList.addAll(Arrays.asList(subSelection));

      for (int i = 0; i < allSensorNames.size(); i++)
      {
         String jointName = allSensorNames.get(i);
         if (!originalJointSensorSelectionList.contains(jointName))
            invertSelection.add(jointName);
      }
      return invertSelection.toArray(new String[0]);
   }

   private LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> getProcessedJointSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case JOINT_POSITION:
         return processedJointPositions;
      case JOINT_VELOCITY:
         return processedJointVelocities;
      case JOINT_ACCELERATION:
         return processedJointAccelerations;
      case JOINT_TAU:
         return processedJointTaus;
      default:
         throw new RuntimeException("Expected a joint sensor.");
      }
   }

   private LinkedHashMap<OneDoFJoint, YoDouble> getOutputJointSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case JOINT_POSITION:
         return outputJointPositions;
      case JOINT_VELOCITY:
         return outputJointVelocities;
      case JOINT_ACCELERATION:
         return outputJointAccelerations;
      case JOINT_TAU:
         return outputJointTaus;
      default:
         throw new RuntimeException("Expected a joint sensor.");
      }
   }

   private LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> getProcessedForceSensorSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case FORCE_SENSOR:
         return processedForces;
      case TORQUE_SENSOR:
         return processedTorques;
      default:
         throw new RuntimeException("Expected a forcce/torque sensor.");
      }
   }

   private LinkedHashMap<ForceSensorDefinition, YoFrameVector> getIntermediateForceSensorSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case FORCE_SENSOR:
         return intermediateForces;
      case TORQUE_SENSOR:
         return intermediateTorques;
      default:
         throw new RuntimeException("Expected a forcce/torque sensor.");
      }
   }

   private LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> getProcessedIMUVectorTypeSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case IMU_ANGULAR_VELOCITY:
         return processedAngularVelocities;
      case IMU_LINEAR_ACCELERATION:
         return processedLinearAccelerations;
      default:
         throw new RuntimeException("Expected either: " + SensorType.IMU_ANGULAR_VELOCITY + " or " + SensorType.IMU_LINEAR_ACCELERATION);
      }
   }

   private LinkedHashMap<IMUDefinition, YoFrameVector> getIntermediateIMUVectorTypeSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case IMU_ANGULAR_VELOCITY:
         return intermediateAngularVelocities;
      case IMU_LINEAR_ACCELERATION:
         return intermediateLinearAccelerations;
      default:
         throw new RuntimeException("Expected either: " + SensorType.IMU_ANGULAR_VELOCITY + " or " + SensorType.IMU_LINEAR_ACCELERATION);
      }
   }

   public void setJointPositionSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointPositions.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointVelocities.get(oneDoFJoint).set(value);
   }
   
   public void setJointAccelerationSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointAccelerations.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointTaus.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, QuaternionReadOnly value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, RotationMatrixReadOnly value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3DReadOnly value)
   {
      inputAngularVelocities.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3DReadOnly value)
   {
      inputLinearAccelerations.get(imuDefinition).set(value);
   }

   public void setForceSensorValue(ForceSensorDefinition forceSensorDefinition, DenseMatrix64F value)
   {
      if (value.getNumRows() != Wrench.SIZE || value.getNumCols() != 1)
         throw new RuntimeException("Unexpected size");

      inputForceSensors.setForceSensorValue(forceSensorDefinition, value);
   }

   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return outputIMUs;
   }

   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      return outputForceSensors;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }
}
