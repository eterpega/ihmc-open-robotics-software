package us.ihmc.sensorProcessing.communication.producers;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.ros2.RealtimeNode;
import us.ihmc.ros2.RealtimePublisher;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Publishes robot configuration data using realtime ROS2 node
 */
public class RobotConfigurationDataPublisher implements RawOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SensorTimestampHolder sensorTimestampHolder;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();
   private final FloatingInverseDynamicsJoint rootJoint;
   private final ArrayList<ForceSensorDataReadOnly> forceSensorDataList = new ArrayList<>();

   private final Wrench packingWrench = new Wrench();

   // Convert from rotationMatrix to quaternion
   private final RotationMatrix[] imuOrientationsAsMatrix;

   private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
   private final RealtimePublisher<RobotConfigurationData> robotConfigurationDataPublisher;

   public RobotConfigurationDataPublisher(FullRobotModel estimatorModel, ForceSensorDataHolderReadOnly forceSensorDataHolder, RealtimeNode realtimeNode,
                                          SensorTimestampHolder sensorTimestampHolder, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly,
                                          RobotMotionStatusHolder robotMotionStatusFromController) throws IOException
   {
      this.sensorTimestampHolder = sensorTimestampHolder;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;

      rootJoint = estimatorModel.getRootJoint();

      FullRobotModelUtils.getAllJointsExcludingHands(joints, estimatorModel);

      for (ForceSensorDefinition definition : forceSensorDataHolder.getForceSensorDefinitions()
                                                                   .toArray(new ForceSensorDefinition[forceSensorDataHolder.getForceSensorDefinitions().size()]))
      {
         ForceSensorDataReadOnly forceSensorData = forceSensorDataHolder.get(definition);
         forceSensorDataList.add(forceSensorData);
      }

      imuOrientationsAsMatrix = new RotationMatrix[estimatorModel.getIMUDefinitions().length];
      for (int i = 0; i < imuOrientationsAsMatrix.length; i++)
      {
         imuOrientationsAsMatrix[i] = new RotationMatrix();
      }

      robotConfigurationDataPublisher = realtimeNode.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");
   }

   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {
      // These confirm the robot received your last command
      // robotConfigurationData.setLast_received_packet_robot_timestamp(robotConfigurationData.getTimestamp());
      // robotConfigurationData.setLast_received_packet_type_id();
      // robotConfigurationData.setLast_received_packet_unique_id();

      robotConfigurationData.getPelvis_angular_velocity().set(rootJoint.getAngularVelocityForReading());
      robotConfigurationData.getPelvis_linear_velocity().set(rootJoint.getLinearVelocityForReading());
      robotConfigurationData.getPelvis_linear_acceleration().set(rootJoint.getLinearAccelerationForReading());
      robotConfigurationData.getRoot_translation().set(rootJoint.getTranslationForReading());
      robotConfigurationData.getRoot_orientation().set(rootJoint.getRotationForReading());
      for (int i = 0; i < robotConfigurationData.getJoint_angles().size(); i++)
      {
         robotConfigurationData.getJoint_angles().set(i, (float) joints.get(i).getQ());
         robotConfigurationData.getJoint_velocities().set(i, (float) joints.get(i).getQd());
         robotConfigurationData.getJoint_torques().set(i, (float) joints.get(i).getTauMeasured());
      }
      robotConfigurationData.getHeader().getStamp().setNanosec(sensorTimestampHolder.getVisionSensorTimestamp());
      robotConfigurationData.setSensor_head_pps_timestamp(sensorTimestampHolder.getSensorHeadPPSTimestamp());
      robotConfigurationData.setRobot_motion_status(robotMotionStatusFromController.getCurrentRobotMotionStatus().getBehaviorId());

      robotConfigurationData.getForce_sensor_data().clear();
      for (int sensorNumber = 0; sensorNumber < forceSensorDataList.size(); sensorNumber++)
      {
         forceSensorDataList.get(sensorNumber).getWrench(packingWrench);
         controller_msgs.msg.dds.Wrench ddsWrench = robotConfigurationData.getForce_sensor_data().add();
         ddsWrench.getAngular_part().set(packingWrench.getAngularPart());
         ddsWrench.getLinear_part().set(packingWrench.getLinearPart());
      }

      if (sensorRawOutputMapReadOnly != null)
      {
         List<? extends IMUSensorReadOnly> imuRawOutputs = sensorRawOutputMapReadOnly.getIMURawOutputs();
         for (int sensorNumber = 0; sensorNumber < imuRawOutputs.size(); sensorNumber++)
         {
            IMUSensorReadOnly imuSensor = imuRawOutputs.get(sensorNumber);
            imuSensor.getLinearAccelerationMeasurement(robotConfigurationData.getImu_sensor_data().get(sensorNumber).getLinear_acceleration());
            imuSensor.getOrientationMeasurement(imuOrientationsAsMatrix[sensorNumber]);
            robotConfigurationData.getImu_sensor_data().get(sensorNumber).getOrientation().set(imuOrientationsAsMatrix[sensorNumber]);
            imuSensor.getAngularVelocityMeasurement(robotConfigurationData.getImu_sensor_data().get(sensorNumber).getAngular_velocity());
         }
      }

      robotConfigurationDataPublisher.publish(robotConfigurationData);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}