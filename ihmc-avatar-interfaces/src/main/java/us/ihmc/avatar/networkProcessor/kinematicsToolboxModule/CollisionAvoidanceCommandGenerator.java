package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
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
   private final InverseDynamicsJoint[] controlledJoints;
   private final InverseKinematicsCommandList commandList;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final FramePose collidingPointPose = new FramePose();
   private final PoseReferenceFrame collidingPointFrame = new PoseReferenceFrame("CollidingPointFrame", collidingPointPose);
   private final Quaternion defaultPointOrientation = new Quaternion();
   
   public CollisionAvoidanceCommandGenerator(RigidBody rootBody, InverseDynamicsJoint[] controlledJoints, InverseKinematicsCommandList collisionAvoidanceCommandList)
   {
      this.rootBody = rootBody;
      this.controlledJoints = controlledJoints;
      this.commandList = collisionAvoidanceCommandList;
   }

   public void addCollisionConstraint(RigidBody rigidBody, Point3D rigidBodyPoint, Vector3D collsionVector)
   {
      //jacobianCalculator.clear();
      //collidingPointPose.setPoseIncludingFrame(rigidBody.getBodyFixedFrame(), rigidBodyPoint, defaultPointOrientation);
      //collidingPointFrame.setPoseAndUpdate(collidingPointPose);
      //jacobianCalculator.setJacobianFrame(collidingPointFrame);
      
      //TODO check if the point orientation affects the results... 
      
   }
}
