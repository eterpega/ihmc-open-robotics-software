package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.THashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.CollisionAvoidanceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CollisionAvoidanceCommandGenerator
{
   private final RigidBody rootBody;
   private final CollisionAvoidanceCommand command;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final FramePose collidingPointPose = new FramePose();
   private final PoseReferenceFrame collidingPointFrame = new PoseReferenceFrame("CollidingPointFrame", collidingPointPose);
   private final Quaternion defaultPointOrientation = new Quaternion();
   
   DenseMatrix64F tempJacobianMatrix, tempCollisionConstraint;
   DenseMatrix64F tempCollisionVector = new DenseMatrix64F(6, 1);
   /**
    * The secret sauce that ensures that the resultant joint velocity is in a direction that avoids collisions
    */
   private double collisionAvoidanceTaskObjective = 0.01;
   
   public CollisionAvoidanceCommandGenerator(RigidBody rootBody, CollisionAvoidanceCommand collisionAvoidanceCommand)
   {
      this.rootBody = rootBody;
      this.command = collisionAvoidanceCommand;
      this.tempJacobianMatrix = new DenseMatrix64F(6, 1);
      this.tempCollisionConstraint = new DenseMatrix64F(1, 1);
   }

   public void addCollisionConstraint(RigidBody rigidBody, Point3D rigidBodyPoint, Vector3D collsionVector)
   {
      jacobianCalculator.clear();
      //TODO check if the point orientation affects the results... 
      collidingPointPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), rigidBodyPoint, defaultPointOrientation);
      collidingPointFrame.setPoseAndUpdate(collidingPointPose);
      jacobianCalculator.setJacobianFrame(collidingPointFrame);
      jacobianCalculator.setKinematicChain(rootBody, rigidBody);
      List<InverseDynamicsJoint> kinematicChain = jacobianCalculator.getJointsFromBaseToEndEffector();
      jacobianCalculator.computeJacobianMatrix();
      jacobianCalculator.getJacobianMatrix(tempJacobianMatrix);
      setCollisionVectorMatrix(collsionVector);
      tempCollisionConstraint.reshape(tempCollisionVector.getNumCols(), tempJacobianMatrix.getNumCols());
      CommonOps.multTransA(tempCollisionVector, tempJacobianMatrix, tempCollisionConstraint);
      command.addConstraint(kinematicChain, tempCollisionConstraint, collisionAvoidanceTaskObjective);
   }
   
   private void setCollisionVectorMatrix(Vector3D collisionVector)
   {
      double norm = collisionVector.length();
      tempCollisionVector.reshape(6, 1);
      for(int i = 0; i < 3; i++)
      {
         // Setting the required rotation to zero
         tempCollisionVector.set(i, 0, 0.0);
         // Setting the required translation to the 
         tempCollisionVector.set(i + 3, 0, collisionVector.getElement(i) / norm);
      }
   }
}

