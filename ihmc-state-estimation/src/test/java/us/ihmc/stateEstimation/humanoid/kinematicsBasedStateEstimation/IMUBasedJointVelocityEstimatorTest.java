package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class IMUBasedJointVelocityEstimatorTest
{
   private static RigidBody rootLink = new RigidBody("rootLink", ReferenceFrame.getWorldFrame());
   private static RevoluteJoint fixedJoint = new RevoluteJoint("fixedJoint", rootLink, new Vector3D(1, 0, 0));
   private static RigidBody parentLink = new RigidBody("parentLink", fixedJoint, new RotationMatrix(), 1.0, new RigidBodyTransform());
   private static RevoluteJoint testJoint = new RevoluteJoint("OneDofJoint", parentLink, new Vector3D(1, 0, 0));
   private static RigidBody childLink = new RigidBody("childLink", testJoint, new RotationMatrix(), 1.0, new RigidBodyTransform());

   private static String parentImuName = "parentImu";
   private static String childImuName = "childImu";
   private static IMUDefinition parentImuDef = new IMUDefinition(parentImuName, parentLink, new RigidBodyTransform());
   private static IMUDefinition childImuDef = new IMUDefinition(childImuName, childLink, new RigidBodyTransform());

   private static IMUSensor parentImu = new IMUSensor(parentImuDef, new SensorNoiseParameters());
   private static IMUSensor childImu = new IMUSensor(childImuDef, new SensorNoiseParameters());

   private YoVariableRegistry testVarRegistry = new YoVariableRegistry("testVars");

   private IMUBasedJointVelocityEstimator estimatorToTest = new IMUBasedJointVelocityEstimator(parentImu, childImu, new SensorOutputMapReadOnly() {
      @Override
      public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
      {
         return 0;
      }

      @Override
      public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
      {
         return 1;
      }

      @Override
      public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
      {
         return 0;
      }

      @Override
      public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
      {
         return 0;
      }

      @Override
      public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
      {
         return false;
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

      @Override
      public long getTimestamp()
      {
         return 0;
      }

      @Override
      public long getVisionSensorTimestamp()
      {
         return 0;
      }

      @Override
      public long getSensorHeadPPSTimestamp()
      {
         return 0;
      }
   }, 0.1, 0.1, testVarRegistry);


	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testZeroAngleEstimate()
   {
      parentImu.setAngularVelocityMeasurement(new Vector3D(1, 0, 0));
      childImu.setAngularVelocityMeasurement(new Vector3D(0, 0, 0));

      estimatorToTest.compute();

      testVarRegistry.getVariable(parentImuName + "JointVelocityEstimatorAlphaFuseVelocity").setValueFromDouble(1.0);
      testVarRegistry.getVariable(childImuName + "JointVelocityEstimatorAlphaFuseVelocity").setValueFromDouble(1.0);

      System.out.println(testVarRegistry.getAllVariables());

      final double pos = estimatorToTest.getEstimatedJointPosition(testJoint);
      final double vel = estimatorToTest.getEstimatedJointVelocitiy(testJoint);
      System.out.println(vel);
   }

}
