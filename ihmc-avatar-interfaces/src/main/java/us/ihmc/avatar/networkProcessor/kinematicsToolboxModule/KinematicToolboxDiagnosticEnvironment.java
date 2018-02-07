package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.ros2.RealtimeNode;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String threadName = "NonRealtimeScheduler";

   public KinematicToolboxDiagnosticEnvironment(DRCRobotModel drcRobotModel)
   {
      FullHumanoidRobotModel humanoidFullRobotModel = drcRobotModel.createFullRobotModel();
      DRCRobotJointMap jointMap = drcRobotModel.getJointMap();
      HumanoidFloatingRootJointRobot humanoidFloatingRobotModel = drcRobotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = drcRobotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      robotInitialSetup.initializeRobot(humanoidFloatingRobotModel, jointMap);
      SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(humanoidFloatingRobotModel, humanoidFullRobotModel,
                                                                                             null);
      sdfPerfectReader.read();

      ForceSensorDefinition[] forceSensorDefinitionArray = humanoidFullRobotModel.getForceSensorDefinitions();
      List<ForceSensorDefinition> forceSensorDefinitionList = Arrays.asList(forceSensorDefinitionArray);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitionList);

      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, netClassList);
      try
      {
         controllerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      SensorOutputMapReadOnly sensorOutputMapReadOnly = initializeSensorOutputMapReadOnly();
      SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly = initializeSensorRawOutputMapReadOnly();
      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      RealtimeNode realtimeNode;
      try
      {
         realtimeNode = new RealtimeNode(new PeriodicNonRealtimeThreadSchedulerFactory(), getClass().getSimpleName(), "/us_ihmc");
         new RobotConfigurationDataPublisher(humanoidFullRobotModel, forceSensorDataHolder, realtimeNode,
                                             sensorOutputMapReadOnly, sensorRawOutputMapReadOnly,
                                             robotMotionStatusFromController);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      DRCNetworkModuleParameters parameters = new DRCNetworkModuleParameters();
      parameters.enableNetworkProcessor(true);
      parameters.enableUiModule(true);
      parameters.enableKinematicsToolbox(true);
      parameters.enableKinematicsToolboxVisualizer(true);
      parameters.enableLocalControllerCommunicator(true);
      new DRCNetworkProcessor(drcRobotModel, parameters);
   }

   private SensorRawOutputMapReadOnly initializeSensorRawOutputMapReadOnly()
   {
      return new SensorRawOutputMapReadOnly()
      {

         @Override
         public long getVisionSensorTimestamp()
         {
            return 0;
         }

         @Override
         public long getTimestamp()
         {
            return 0;
         }

         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return 0;
         }

         @Override
         public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMURawOutputs()
         {
            return Collections.<IMUSensorReadOnly>emptyList();
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
         {
            return null;
         }
      };
   }

   private long timestamp = 0L;

   private SensorOutputMapReadOnly initializeSensorOutputMapReadOnly()
   {
      return new SensorOutputMapReadOnly()
      {

         @Override
         public long getVisionSensorTimestamp()
         {
            timestamp += Conversions.millisecondsToNanoseconds(1L);
            return timestamp;
         }

         @Override
         public long getTimestamp()
         {
            return timestamp;
         }

         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return timestamp;
         }

         @Override
         public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
         {
            return false;
         }

         @Override
         public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
         {
            return null;
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
         {
            return null;
         }
      };
   }

}
