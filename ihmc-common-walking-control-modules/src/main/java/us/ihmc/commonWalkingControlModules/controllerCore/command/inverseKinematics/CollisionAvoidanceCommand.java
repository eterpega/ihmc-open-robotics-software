package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.THashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class CollisionAvoidanceCommand implements InverseKinematicsCommand<CollisionAvoidanceCommand>
{
   private static final int defaultTaskSize = 10;
   private int numberOfJoints;
   private int numberOfDoFs;
   private DenseMatrix64F jacobian;
   private DenseMatrix64F objective;
   private InverseDynamicsJoint[] joints;
   private int taskSize = 0;
   private THashMap<InverseDynamicsJoint, TIntArrayList> jointIndexMap = new THashMap<>();

   public CollisionAvoidanceCommand(InverseDynamicsJoint[] controlledJoints)
   {
      this.numberOfJoints = controlledJoints.length;
      this.joints = controlledJoints;
      this.numberOfDoFs = getNumberOfDoFs();
      this.jacobian = new DenseMatrix64F(defaultTaskSize, numberOfDoFs);
      this.objective = new DenseMatrix64F(defaultTaskSize, 1);
      createColumnIndexMap();
      reset();
   }

   private int getNumberOfDoFs()
   {
      int numberOfDoFs = 0;
      for(int i = 0; i < joints.length; i++)
         numberOfDoFs += joints[i].getDegreesOfFreedom();
      return numberOfDoFs;
   }

   public void reset()
   {
      taskSize = 0;
      jacobian.reshape(taskSize, numberOfDoFs);
      objective.reshape(taskSize, 1);
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
   
   public DenseMatrix64F getTaskObjective()
   {
      return objective;
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
      return -Integer.MAX_VALUE;
   }

   private void createColumnIndexMap()
   {
      int columnIndex = 0;
      for(int i = 0; i < joints.length; i++)
      {
         InverseDynamicsJoint joint = joints[i];
         TIntArrayList columnList = new TIntArrayList(joint.getDegreesOfFreedom());
         for(int j = 0; j < joint.getDegreesOfFreedom(); j++)
            columnList.add(columnIndex++);
         jointIndexMap.put(joint, columnList);
      }
   }
   
   public void addConstraint(List<InverseDynamicsJoint> kinematicChain, DenseMatrix64F taskJacobian, double taskObjective)
   {
      jacobian.reshape(taskSize + 1, numberOfDoFs, true);
      objective.reshape(taskSize + 1, 1);
      int taskJacobianColumnIndex = 0;
      for(int i = 0; i < kinematicChain.size(); i++)
      {
         InverseDynamicsJoint joint = kinematicChain.get(i);
         TIntArrayList columns = jointIndexMap.get(joint);
         if(columns == null)
         {
            taskJacobianColumnIndex += joint.getDegreesOfFreedom();
            throw new RuntimeException("Got collision avoidance jacobian for unregistered joint: " + joint.getName());
         }
         if(joint.getDegreesOfFreedom() != columns.size())
            throw new RuntimeException("Joint column index entries do not match joint degrees of freedom");
         for(int j = 0; j < columns.size(); j++)
         {
            //PrintTools.debug("Task: " + taskJacobianColumnIndex + " " + columns.get(j));
            jacobian.set(taskSize, columns.get(j), taskJacobian.get(0, taskJacobianColumnIndex++));
         }
      }
      //PrintTools.debug("Task Jacobian " + jacobian.toString() + " Task Objective: " + objective.toString());
      objective.set(taskSize, 0, taskObjective);
      taskSize++;
   }
}
