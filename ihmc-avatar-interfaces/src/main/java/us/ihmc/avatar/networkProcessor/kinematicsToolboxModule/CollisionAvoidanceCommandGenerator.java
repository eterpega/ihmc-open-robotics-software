package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.CollisionAvoidanceCommand;
import us.ihmc.commons.Epsilons;
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
   private final FrameConvexPolytopeVisualizer viz;
   
   DenseMatrix64F tempJacobianMatrix, tempCollisionConstraint;
   DenseMatrix64F tempCollisionVector = new DenseMatrix64F(6, 1);
   /**
    * The secret sauce that ensures that the resultant joint velocity is in a direction that avoids collisions. This should be as high as possible 
    * but not so high that the solution becomes in feasible
    */
   private double collisionAvoidanceTaskObjective = 750; //m/s 

   public CollisionAvoidanceCommandGenerator(RigidBody rootBody, CollisionAvoidanceCommand collisionAvoidanceCommand)
   {
      this(rootBody, collisionAvoidanceCommand, null);
   }
   
   public CollisionAvoidanceCommandGenerator(RigidBody rootBody, CollisionAvoidanceCommand collisionAvoidanceCommand, FrameConvexPolytopeVisualizer viz)
   {
      this.rootBody = rootBody;
      this.command = collisionAvoidanceCommand;
      this.tempJacobianMatrix = new DenseMatrix64F(6, 1);
      this.tempCollisionConstraint = new DenseMatrix64F(1, 1);
      this.viz = viz;
   }

   private final Point3D tempPoint = new Point3D();
   public void addCollisionConstraint(RigidBody rigidBody, Point3D rigidBodyPoint, Vector3D collsionVector)
   {
      if(viz!=null)
      {
         tempPoint.set(rigidBodyPoint);
         tempPoint.add(collsionVector);
         viz.showCollisionVector(rigidBodyPoint, tempPoint);
      }
      jacobianCalculator.clear();
      //TODO check if the point orientation affects the results... 
      collidingPointPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), rigidBodyPoint, defaultPointOrientation);
      collidingPointFrame.setPoseAndUpdate(collidingPointPose);
      if(viz != null)
         viz.showRigidBodyCollidingPoint(collidingPointFrame);
      jacobianCalculator.setKinematicChain(rootBody, rigidBody);
      jacobianCalculator.setJacobianFrame(collidingPointFrame);
      List<InverseDynamicsJoint> kinematicChain = jacobianCalculator.getJointsFromBaseToEndEffector();
      jacobianCalculator.computeJacobianMatrix();
      jacobianCalculator.getJacobianMatrix(tempJacobianMatrix);
      setCollisionVectorMatrix(collsionVector);
      tempCollisionConstraint.reshape(tempCollisionVector.getNumCols(), tempJacobianMatrix.getNumCols());
      CommonOps.multTransA(tempCollisionVector, tempJacobianMatrix, tempCollisionConstraint);
      CommonOps.scale(-1.0, tempCollisionConstraint);
      command.addConstraint(kinematicChain, tempCollisionConstraint, -collisionAvoidanceTaskObjective * collsionVector.length());
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

