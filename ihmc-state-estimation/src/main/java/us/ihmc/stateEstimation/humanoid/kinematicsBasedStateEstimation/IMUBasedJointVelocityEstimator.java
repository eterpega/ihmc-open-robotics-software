/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Creates an alpha filter defined by:
 * 
 * qdFiltered = alpha*qd_from_IMU_estimate + (1-alpha)*qdFromEncoders
 * 
 * In which qdFiltered is defined as the filtered version of:
 * 
 * qdFromEncoders = {qd_WaistRotator, qd_WaistExtensor, qd_WaistLateralExtensor}
 *
 * call getEncoderVelocityEstimates for output of the filter.
 * 
 */
public class IMUBasedJointVelocityEstimator
{
   private final YoDouble alphaVelocity;
   private final YoDouble alphaPosition;
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final YoDouble slopTime;
   private final Map<OneDoFJoint, BacklashProcessingYoVariable> jointVelocities = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointVelocitiesFromIMUOnly = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositions = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositionsFromIMUOnly = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;
   private final FrameVector3D childAngularVelocity = new FrameVector3D();
   private final FrameVector3D parentAngularVelocity = new FrameVector3D();

   private final DenseMatrix64F jacobianAngularPart64F;
   private final DenseMatrix64F jacobianTransposed;
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated;
   private final DenseMatrix64F inverse;
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(1, 1);
   
   private final double estimatorDT;

   public IMUBasedJointVelocityEstimator(IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, SensorOutputMapReadOnly sensorMap, double estimatorDT,
                                         double slopTime, YoVariableRegistry registry)
   {
      this.sensorMap = sensorMap;
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;
      jacobian = new GeometricJacobian(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink(), childIMU.getMeasurementLink().getBodyFixedFrame());
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);

      String namePrefix = childIMU.getSensorName() + "JointVelocityEstimator";
      alphaVelocity = new YoDouble(namePrefix + "AlphaFuseVelocity", registry);
      alphaVelocity.set(0.0);
      alphaPosition = new YoDouble(namePrefix + "AlphaFusePosition", registry);
      alphaPosition.set(0.0);

      this.estimatorDT = estimatorDT;
      this.slopTime = new YoDouble(namePrefix + "SlopTime", registry);
      this.slopTime.set(slopTime);

      jacobianAngularPart64F = new DenseMatrix64F(3, joints.length);
      jacobianTransposed = new DenseMatrix64F(joints.length, 3);
      qd_estimated = new DenseMatrix64F(joints.length, 1);
      inverse = new DenseMatrix64F(joints.length, joints.length);

      for (OneDoFJoint joint : joints)
      {
         jointVelocitiesFromIMUOnly.put(joint, new YoDouble("qd_" + joint.getName() + "_IMUBased", registry));
         jointVelocities.put(joint, new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", estimatorDT, this.slopTime, registry));

         jointPositionsFromIMUOnly.put(joint, new YoDouble("q_" + joint.getName() + "_IMUBased", registry));
         jointPositions.put(joint, new YoDouble("q_" + joint.getName() + "_FusedWithIMU", registry));
      }
   }

   public void setAlphaFuse(double alphaVelocity, double alphaPosition)
   {
      this.alphaVelocity.set(alphaVelocity);
      this.alphaPosition.set(alphaPosition);
   }

   public void compute()
   {
      jacobian.compute();
      // jacobian is 6xn
      CommonOps.extract(jacobian.getJacobianMatrix(), 0, 3, 0, joints.length, jacobianAngularPart64F, 0, 0);

      CommonOps.transpose(jacobianAngularPart64F, jacobianTransposed);

      // set tempMatrix = J' * J
      //       nxn       nx3  3xn    where n = joints.length
      tempMatrix.reshape(jacobianTransposed.getNumRows(), jacobianTransposed.getNumRows());
      CommonOps.mult(jacobianTransposed, jacobianAngularPart64F, tempMatrix);

      if (Math.abs(CommonOps.det(tempMatrix)) < 1e-5)
      {
         return;
      }

      // set inverse = inv(J' * J)
      CommonOps.invert(tempMatrix, inverse);

      // set tempMatrix = inv(J' * J) * J'
      tempMatrix.reshape(jacobianTransposed.getNumRows(), jacobianTransposed.getNumCols());
      CommonOps.mult(inverse, jacobianTransposed, tempMatrix);

      childAngularVelocity.setToZero(childIMU.getMeasurementFrame());
      childIMU.getAngularVelocityMeasurement(childAngularVelocity);
      childAngularVelocity.changeFrame(jacobian.getJacobianFrame());

      parentAngularVelocity.setToZero(parentIMU.getMeasurementFrame());
      parentIMU.getAngularVelocityMeasurement(parentAngularVelocity);
      parentAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      childAngularVelocity.sub(parentAngularVelocity);

      childAngularVelocity.get(omega);
      CommonOps.mult(tempMatrix, omega, qd_estimated);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         double qd_sensorMap = joint.getQd();//sensorMap.getJointVelocityProcessedOutput(joint);
         double qd_IMU = qd_estimated.get(i, 0);
         double qd_fused = (1.0 - alphaVelocity.getDoubleValue()) * qd_sensorMap + alphaVelocity.getDoubleValue() * qd_IMU;

         jointVelocitiesFromIMUOnly.get(joint).set(qd_IMU);
         jointVelocities.get(joint).update(qd_fused);

         double q_sensorMap = joint.getQ();//sensorMap.getJointPositionProcessedOutput(joint);
         double q_IMU = jointPositions.get(joint).getDoubleValue() + estimatorDT * qd_IMU; // is qd_IMU or qd_fused better here?
         double q_fused = (1.0 - alphaPosition.getDoubleValue()) * q_sensorMap + alphaPosition.getDoubleValue() * q_IMU;

         jointPositionsFromIMUOnly.get(joint).set(q_IMU);
         jointPositions.get(joint).set(q_fused);
      }
   }

   public double getEstimatedJointVelocitiy(OneDoFJoint joint)
   {
      BacklashProcessingYoVariable estimatedJointVelocity = jointVelocities.get(joint);
      if (estimatedJointVelocity != null)
         return estimatedJointVelocity.getDoubleValue();
      else
         return Double.NaN;
   }

   public double getEstimatedJointPosition(OneDoFJoint joint)
   {
      YoDouble estimatedJointPosition = jointPositions.get(joint);
      if (estimatedJointPosition != null)
         return estimatedJointPosition.getDoubleValue();
      else
         return Double.NaN;
   }
}
