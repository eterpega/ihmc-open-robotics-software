package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;


/**
 * JointStateUpdater simply reads the joint position/velocity sensors and computes joint position/velocity estimates based on IMU data.
 * @author Patrick Hammer
 *
 */
public class JointStateUpdater
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private OneDoFJoint[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final ArrayList<IMUBasedJointVelocityEstimator> iMUBasedJointVelocityEstimators = new ArrayList<>();

   private final YoBoolean enableIMUBasedJointVelocityEstimator = new YoBoolean("enableIMUBasedJointVelocityEstimator", registry);

   public JointStateUpdater(OneDoFJoint[] oneDoFJoints, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      this.sensorMap = sensorOutputMapReadOnly;

      this.oneDoFJoints = oneDoFJoints;

      for (ImmutablePair<String, String> imuPair : stateEstimatorParameters.getIMUSensorsToUseInJointStateEstimator())
      {
         final IMUBasedJointVelocityEstimator jointEstimator = createIMUBasedJointVelocityEstimator(sensorOutputMapReadOnly, stateEstimatorParameters, imuPair,
                                                                                                    registry);
         if (jointEstimator != null)
         {
            iMUBasedJointVelocityEstimators.add(jointEstimator);
         }
      }

      parentRegistry.addChild(registry);
   }
   
   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry) {
      this.sensorMap = sensorOutputMapReadOnly;
      this.oneDoFJoints = null;
   }

   public void setJointsToUpdate(OneDoFJoint[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   private IMUBasedJointVelocityEstimator createIMUBasedJointVelocityEstimator(SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                                                               StateEstimatorParameters stateEstimatorParameters,
                                                                               ImmutablePair<String, String> imuPair, YoVariableRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null)
         return null;

      enableIMUBasedJointVelocityEstimator.set(stateEstimatorParameters.useIMUsForJointVelocityEstimation());

      final String parentImuName = imuPair.getLeft();
      final String childImuName = imuPair.getRight();

      IMUSensorReadOnly parentImu = sensorOutputMapReadOnly.getIMUProcessedOutputs().stream().filter(item -> parentImuName.equals(item.getSensorName()))
                                                           .findFirst().orElse(null);
      IMUSensorReadOnly childImu = sensorOutputMapReadOnly.getIMUProcessedOutputs().stream().filter(item -> childImuName.equals(item.getSensorName()))
                                                          .findFirst().orElse(null);

      // TODO create the module with the two IMUs to compute and smoothen the spine joint velocities here.
      if (parentImu != null && childImu != null)
      {
         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         double slopTime = stateEstimatorParameters.getIMUJointVelocityEstimationBacklashSlopTime();
         IMUBasedJointVelocityEstimator iMUBasedJointVelocityEstimator = new IMUBasedJointVelocityEstimator(parentImu, childImu, sensorOutputMapReadOnly,
                                                                                                            estimatorDT, slopTime, parentRegistry);
         iMUBasedJointVelocityEstimator.compute();
         double alphaIMUsForSpineJointVelocityEstimation = stateEstimatorParameters.getAlphaIMUsForSpineJointVelocityEstimation();
         double alphaIMUsForSpineJointPositionEstimation = stateEstimatorParameters.getAlphaIMUsForSpineJointPositionEstimation();
         iMUBasedJointVelocityEstimator.setAlphaFuse(alphaIMUsForSpineJointVelocityEstimation, alphaIMUsForSpineJointPositionEstimation);
         return iMUBasedJointVelocityEstimator;
      }
      else
      {
         PrintTools.warn("Could not find the configured IMU pair in the robot model: parentIMU = " + parentImuName + ", childIMU = " + childImuName);
         if (parentImu == null)
         {
            PrintTools.warn("Parent IMU is null.");
         }
         if (childImu == null)
         {
            PrintTools.warn("Child IMU is null.");
         }
         return null;
      }
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      for (IMUBasedJointVelocityEstimator jointVelocityEstimator : iMUBasedJointVelocityEstimators)
      {
         jointVelocityEstimator.compute();
      }

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         double positionSensorData = sensorMap.getJointPositionProcessedOutput(oneDoFJoint);
         double velocitySensorData = sensorMap.getJointVelocityProcessedOutput(oneDoFJoint);
         double torqueSensorData = sensorMap.getJointTauProcessedOutput(oneDoFJoint);
         boolean jointEnabledIndicator = sensorMap.isJointEnabled(oneDoFJoint);

         if (enableIMUBasedJointVelocityEstimator.getBooleanValue())
         {
            // TODO: seems like there should be a joint to estimator mapping
            for (IMUBasedJointVelocityEstimator jointVelocityEstimator : iMUBasedJointVelocityEstimators)
            {
               double estimatedJointVelocity = jointVelocityEstimator.getEstimatedJointVelocitiy(oneDoFJoint);
               if (!Double.isNaN(estimatedJointVelocity))
                  velocitySensorData = estimatedJointVelocity;

               double estimatedJointPosition = jointVelocityEstimator.getEstimatedJointPosition(oneDoFJoint);
               if (!Double.isNaN(estimatedJointPosition))
                  positionSensorData = estimatedJointPosition;
            }
         }

         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
         oneDoFJoint.setTauMeasured(torqueSensorData);
         oneDoFJoint.setEnabled(jointEnabledIndicator);
      }
   }
}
