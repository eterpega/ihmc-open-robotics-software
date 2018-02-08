package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.tuple.Pair;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimulatedSensorHolderAndReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   protected final YoInteger step = new YoInteger("step", registry);

   protected final YoDouble yoTime;

   protected final List<Pair<OneDoFJoint, DoubleProvider>> jointPositionSensors = new ArrayList<>();
   protected final List<Pair<OneDoFJoint, DoubleProvider>> jointVelocitySensors = new ArrayList<>();
   protected final List<Pair<OneDoFJoint, DoubleProvider>> jointTorqueSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, QuaternionProvider>> orientationSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, Vector3DProvider>> angularVelocitySensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, Vector3DProvider>> linearAccelerationSensors = new ArrayList<>();
   protected final List<Pair<ForceSensorDefinition, WrenchCalculatorInterface>> forceTorqueSensors = new ArrayList<>();

   protected final Random random = new Random(343465);
   protected final SensorNoiseSimulator sensorNoiseSimulator;
   protected final SensorProcessing sensorProcessing;

   public SimulatedSensorHolderAndReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                                         SensorProcessingConfiguration sensorProcessingConfiguration, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {

      SensorNoiseParameters sensorNoiseParameters = sensorProcessingConfiguration.getSensorNoiseParameters();
      if (sensorNoiseParameters != null)
         this.sensorNoiseSimulator = new SensorNoiseSimulator(stateEstimatorSensorDefinitions, random, sensorNoiseParameters, registry);
      else
         this.sensorNoiseSimulator = null;
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      this.yoTime = yoTime;
      step.set(29831);

      parentRegistry.addChild(registry);
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, DoubleProvider jointPositionSensor)
   {
      jointPositionSensors.add(Pair.of(oneDoFJoint, jointPositionSensor));
   }

   public void addJointTorqueSensorPort(OneDoFJoint oneDoFJoint, DoubleProvider jointTorqueSensor)
   {
      jointTorqueSensors.add(Pair.of(oneDoFJoint, jointTorqueSensor));
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, DoubleProvider jointVelocitySensor)
   {
      jointVelocitySensors.add(Pair.of(oneDoFJoint, jointVelocitySensor));
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, QuaternionProvider orientationSensor)
   {
      orientationSensors.add(Pair.of(imuDefinition, orientationSensor));
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, Vector3DProvider angularVelocitySensor)
   {
      angularVelocitySensors.add(Pair.of(imuDefinition, angularVelocitySensor));
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, Vector3DProvider linearAccelerationSensor)
   {
      linearAccelerationSensors.add(Pair.of(imuDefinition, linearAccelerationSensor));
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.add(Pair.of(forceSensorDefinition, groundContactPointBasedWrenchCalculator));
   }

   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   public void read()
   {
      updateSensorNoiseSimulator();
      updateSensorProcessor();
      step.increment();
   }

   private void updateSensorNoiseSimulator()
   {
      if (sensorNoiseSimulator == null)
         return;

      for (int i = 0; i < jointPositionSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointPositionSensors.get(i).getLeft();
         double newValue = jointPositionSensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setJointPositionSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointTorqueSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointTorqueSensors.get(i).getLeft();
         double newValue = jointTorqueSensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setJointTauSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointVelocitySensors.size(); i++)
      {
         double newValue = jointVelocitySensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setJointVelocitySensorValue(jointVelocitySensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < orientationSensors.size(); i++)
      {
         QuaternionReadOnly newValue = orientationSensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setOrientationSensorValue(orientationSensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         Vector3DReadOnly newValue = angularVelocitySensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setAngularVelocitySensorValue(angularVelocitySensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {
         Vector3DReadOnly newValue = linearAccelerationSensors.get(i).getRight().getValue();
         sensorNoiseSimulator.setLinearAccelerationSensorValue(linearAccelerationSensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < forceTorqueSensors.size(); i++)
      {
         final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.get(i).getRight();
         forceTorqueSensor.calculate();
         sensorNoiseSimulator.setForceSensorValue(forceTorqueSensors.get(i).getLeft(), forceTorqueSensor.getWrench());
      }

      sensorNoiseSimulator.startComputation();
   }

   private final DenseMatrix64F newWrench = new DenseMatrix64F(6, 1);

   private void updateSensorProcessor()
   {
      for (int i = 0; i < jointPositionSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointPositionSensors.get(i).getLeft();
         double newValue = jointPositionSensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getJointPositionProcessedOutput(oneDoFJoint);
         sensorProcessing.setJointPositionSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointTorqueSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointTorqueSensors.get(i).getLeft();
         double newValue = jointTorqueSensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getJointTauProcessedOutput(oneDoFJoint);
         sensorProcessing.setJointTauSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointVelocitySensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointVelocitySensors.get(i).getLeft();
         double newValue = jointVelocitySensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getJointVelocityProcessedOutput(oneDoFJoint);
         sensorProcessing.setJointVelocitySensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < orientationSensors.size(); i++)
      {
         IMUDefinition imu = orientationSensors.get(i).getLeft();
         QuaternionReadOnly newValue = orientationSensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getIMUProcessedOutputs().get(i).getOrientationMeasurement();
         sensorProcessing.setOrientationSensorValue(imu, newValue);
      }

      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         Vector3DReadOnly newValue = angularVelocitySensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getIMUProcessedOutputs().get(i).getAngularVelocityMeasurement();
         sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {
         Vector3DReadOnly newValue = linearAccelerationSensors.get(i).getRight().getValue();
         if (sensorNoiseSimulator != null)
            newValue = sensorNoiseSimulator.getIMUProcessedOutputs().get(i).getLinearAccelerationMeasurement();
         sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < forceTorqueSensors.size(); i++)
      {
         ForceSensorDefinition forceSensor = forceTorqueSensors.get(i).getLeft();

         if (sensorNoiseSimulator == null)
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.get(i).getRight();
            forceTorqueSensor.calculate();
            newWrench.set(forceTorqueSensor.getWrench());
         }
         else
         {
            sensorNoiseSimulator.getForceSensorProcessedOutputs().getForceSensorValue(forceSensor, newWrench);
         }

         sensorProcessing.setForceSensorValue(forceSensor, newWrench);
      }

      long timestamp = Conversions.secondsToNanoseconds(yoTime.getDoubleValue());
      sensorProcessing.startComputation(timestamp, timestamp, -1);
   }

   @Override
   public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
