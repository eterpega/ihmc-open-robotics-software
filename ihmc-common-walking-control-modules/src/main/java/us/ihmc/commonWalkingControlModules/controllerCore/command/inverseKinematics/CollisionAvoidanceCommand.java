package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class CollisionAvoidanceCommand implements InverseKinematicsCommand<CollisionAvoidanceCommand>
{
   private int numberOfJoints;
   private DenseMatrix64F jacobian = new DenseMatrix64F();
   private InverseDynamicsJoint[] joints;
   private int taskSize = 0;

   public CollisionAvoidanceCommand(InverseDynamicsJoint[] oneDoFJoints)
   {
      this.numberOfJoints = oneDoFJoints.length;
      this.joints = oneDoFJoints;
      this.jacobian.reshape(numberOfJoints, taskSize);
      reset();
   }

   public void reset()
   {
      taskSize = 0;
      jacobian.reshape(taskSize, numberOfJoints);
   }

   public InverseDynamicsJoint[] getJoints()
   {
      return joints;
   }

   public boolean getIsEqualityConstraint()
   {
      return false;
   }

   public int getNumberOfJoints()
   {
      return joints.length;
   }

   public InverseDynamicsJoint getJoint(int jointIndex)
   {
      return joints[jointIndex];
   }

   @Override
   public void set(CollisionAvoidanceCommand other)
   {
      this.numberOfJoints = other.getNumberOfJoints();
      this.taskSize = other.getTaskSize();
      this.joints = other.getJoints();
      this.jacobian.set(other.getTaskJacobian());
   }
   
   public DenseMatrix64F getTaskJacobian()
   {
      return jacobian;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COLLISION_AVOIDANCE;
   }

   public int getTaskSize()
   {
      return taskSize;
   }

   public double getTaskJacobianEntry(InverseDynamicsJoint joint, int taskIndex, int dofIndex)
   {
      return jacobian.get(taskIndex, getJointStartIndex(joint) + dofIndex);
   }

   private int getJointStartIndex(InverseDynamicsJoint joint)
   {
      int startIndex = 0;
      for(int i = 0; i < joints.length; i++)
      {
         if(joints[i] == joint)
            return startIndex;
         startIndex += joints[i].getDegreesOfFreedom();
      }
      return -1;
   }
}
