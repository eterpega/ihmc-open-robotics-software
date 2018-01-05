package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;


/**
 * JointStateUpdater simply reads the joint position/velocity sensors and updates the FullInverseDynamicsStructure.
 * (Based on {@link us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater}.)
 * @author Sylvain
 *
 */
public class JointStateUpdaterWithSpatialAccelerationCalc
{
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final JointStateUpdater jointStateUpdater;
   private final RigidBody rootBody;

   public JointStateUpdaterWithSpatialAccelerationCalc(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      this.rootBody = inverseDynamicsStructure.getElevator();
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);
      jointStateUpdater = new JointStateUpdater(oneDoFJoints, sensorOutputMapReadOnly, stateEstimatorParameters, parentRegistry);
   }

   public void setJointsToUpdate(OneDoFJoint[] oneDoFJoints)
   {
      this.jointStateUpdater.setJointsToUpdate(oneDoFJoints);
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      jointStateUpdater.updateJointState();
      this.rootBody.updateFramesRecursively();
      spatialAccelerationCalculator.compute();
   }
}
