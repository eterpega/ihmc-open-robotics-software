package us.ihmc.avatar.networkProcessor.wbtToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

import java.util.Arrays;
import java.util.Collection;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.NormOps;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WholeBodyTrajectoryToolboxHelper
{
   public static double kinematicsChainLimitScore(RigidBody start, RigidBody end)
   {
      return jointsLimitScore(ScrewTools.createOneDoFJointPath(start, end));
   }

   public static double jointsLimitScore(Collection<OneDoFJoint> jointsToScore)
   {
      return jointsToScore.stream().mapToDouble(j -> jointLimitScore(j)).sum();
   }

   public static double jointsLimitScore(OneDoFJoint... jointsToScore)
   {
      return Arrays.stream(jointsToScore).mapToDouble(j -> jointLimitScore(j)).sum();
   }

   /**
    * This joint limit score is defined in the following paper (proposed by Yoshikawa).
    * <p>
    * Nelson, Khosla, "Strategies for increasing the tracking region of an eye-in-hand system by
    * singularity and joint limit avoidance.", The International journal of robotics research 14.3
    * (1995): 255-269.
    * <p>
    * See {@link
    * <p>
    * http://repository.cmu.edu/cgi/viewcontent.cgi?article=1581&context=isr}
    * <p>
    * See equation 24, 26. Joint limit score is stated on the head of natural exponential.
    * 
    */
   public static double jointLimitScore(OneDoFJoint jointToScore)
   {
      double q = jointToScore.getQ();
      double upperLimit = jointToScore.getJointLimitUpper();
      double lowerLimit = jointToScore.getJointLimitLower();

      double motionRange = upperLimit - lowerLimit;

      double diffUpper = upperLimit - q;
      double diffLower = q - lowerLimit;

      // Yoshikawa's definition.
      return diffUpper * diffLower / (motionRange * motionRange);
   }

   public static void setSelectionMatrix(SelectionMatrix6D selectionMatrix, ConfigurationSpaceName configurationSpaceName, boolean select)
   {
      switch (configurationSpaceName)
      {
      case X:
         selectionMatrix.selectLinearX(select);
         break;
      case Y:
         selectionMatrix.selectLinearY(select);
         break;
      case Z:
         selectionMatrix.selectLinearZ(select);
         break;
      case ROLL:
         selectionMatrix.selectAngularX(select);
         break;
      case PITCH:
         selectionMatrix.selectAngularY(select);
         break;
      case YAW:
         selectionMatrix.selectAngularZ(select);
         break;
      default:
         throw new RuntimeException("Unexpected enum value: " + configurationSpaceName);
      }
   }

   public static double computeTrajectoryPositionError(Pose3D solution, Pose3D expected, RigidBodyExplorationConfigurationMessage explorationMessage,
                                                       WaypointBasedTrajectoryMessage trajectory)
   {
      PoseReferenceFrame solutionRigidBodyFrame = new PoseReferenceFrame("solutionRigidBodyFrame", ReferenceFrame.getWorldFrame());
      solutionRigidBodyFrame.setPoseAndUpdate(new Point3D(solution.getPosition()), new Quaternion(solution.getOrientation()));

      FramePoint3D positionError = new FramePoint3D(ReferenceFrame.getWorldFrame(), expected.getPosition());
      positionError.changeFrame(solutionRigidBodyFrame);
      DenseMatrix64F positionErrorQ = new DenseMatrix64F(3, 1);
      positionError.get(positionErrorQ);

      FrameQuaternion orientationError = new FrameQuaternion(ReferenceFrame.getWorldFrame(), expected.getOrientation());
      orientationError.changeFrame(solutionRigidBodyFrame);
      Vector3D rotationError = new Vector3D();
      RotationVectorConversion.convertQuaternionToRotationVector(orientationError, rotationError);
      DenseMatrix64F rotationErrorQ = new DenseMatrix64F(3, 1);
      rotationError.get(rotationErrorQ);

      if (explorationMessage != null)
      {
         ConfigurationSpaceName[] degreesOfFreedomToExplore = explorationMessage.degreesOfFreedomToExplore;
         for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
         {
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.X || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Y
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Z)
               positionErrorQ.zero();
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.ROLL || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.PITCH
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.YAW)
               rotationErrorQ.zero();
         }
      }

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      trajectory.getSelectionMatrix(selectionMatrix);
      if (!selectionMatrix.isLinearXSelected() || !selectionMatrix.isLinearYSelected() || !selectionMatrix.isLinearZSelected())
         positionErrorQ.zero();
      if (!selectionMatrix.isAngularXSelected() || !selectionMatrix.isAngularYSelected() || !selectionMatrix.isAngularZSelected())
         rotationErrorQ.zero();

      return NormOps.normP2(positionErrorQ);
   }

   public static double computeTrajectoryOrientationError(Pose3D solution, Pose3D expected, RigidBodyExplorationConfigurationMessage explorationMessage,
                                                          WaypointBasedTrajectoryMessage trajectory)
   {
      PoseReferenceFrame solutionRigidBodyFrame = new PoseReferenceFrame("solutionRigidBodyFrame", ReferenceFrame.getWorldFrame());
      solutionRigidBodyFrame.setPoseAndUpdate(new Point3D(solution.getPosition()), new Quaternion(solution.getOrientation()));

      FramePoint3D positionError = new FramePoint3D(ReferenceFrame.getWorldFrame(), expected.getPosition());
      positionError.changeFrame(solutionRigidBodyFrame);
      DenseMatrix64F positionErrorQ = new DenseMatrix64F(3, 1);
      positionError.get(positionErrorQ);

      FrameQuaternion orientationError = new FrameQuaternion(ReferenceFrame.getWorldFrame(), expected.getOrientation());
      orientationError.changeFrame(solutionRigidBodyFrame);
      Vector3D rotationError = new Vector3D();
      RotationVectorConversion.convertQuaternionToRotationVector(orientationError, rotationError);
      DenseMatrix64F rotationErrorQ = new DenseMatrix64F(3, 1);
      rotationError.get(rotationErrorQ);

      if (explorationMessage != null)
      {
         ConfigurationSpaceName[] degreesOfFreedomToExplore = explorationMessage.degreesOfFreedomToExplore;
         for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
         {
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.X || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Y
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Z)
               positionErrorQ.zero();
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.ROLL || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.PITCH
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.YAW)
               rotationErrorQ.zero();
         }
      }

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      trajectory.getSelectionMatrix(selectionMatrix);
      if (!selectionMatrix.isLinearXSelected() || !selectionMatrix.isLinearYSelected() || !selectionMatrix.isLinearZSelected())
         positionErrorQ.zero();
      if (!selectionMatrix.isAngularXSelected() || !selectionMatrix.isAngularYSelected() || !selectionMatrix.isAngularZSelected())
         rotationErrorQ.zero();

      return NormOps.normP2(rotationErrorQ);
   }

   /**
    * Manifold message for Sphere.
    * 
    * @param rigidBody : which rigid body to reach on this sphere.
    * @param originPosition : origin position of this sphere.
    * @param radius : radius of this sphere.
    */
   public static ReachingManifoldMessage createSphereManifoldMessages(RigidBody rigidBody, Point3D originPosition, double radius)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI, -Math.PI * 0.5, 0.0};
      double[] upperLimits = new double[] {Math.PI, Math.PI * 0.5, radius};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }

   /**
    * Manifold message for Cylinder.
    * Cylinder will be created from the bottom.
    * 
    * @param rigidBody : which rigid body to reach on this cylinder.
    * @param originPosition : origin position of this cylinder.
    * @param originOrientation : origin orientation of this cylinder.
    * @param radius : radius of this cylinder.
    * @param height : height of this cylinder.
    * @return
    */
   public static ReachingManifoldMessage createCylinderManifoldMessages(RigidBody rigidBody, Point3D originPosition, Quaternion originOrientation,
                                                                        double radius, double height)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.X, ConfigurationSpaceName.Z};
      double[] lowerLimits = new double[] {-Math.PI, 0.0, 0.0};
      double[] upperLimits = new double[] {Math.PI, radius, height};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }

   /**
    * Manifold message for Box.
    * Box will be created from the center.
    * 
    * @param rigidBody : which rigid body to reach on this box.
    * @param originPosition : origin position of this box.
    * @param originOrientation : origin orientation of this box.
    * @param lengthX : length in X of this box.
    * @param lengthY : length in Y of this box.
    * @param lengthZ : length in Z of this box.
    * @return
    */
   public static ReachingManifoldMessage createBoxManifoldMessages(RigidBody rigidBody, Point3D originPosition, Quaternion originOrientation, double lengthX,
                                                                   double lengthY, double lengthZ)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-lengthX * 0.5, -lengthY * 0.5, -lengthZ * 0.5};
      double[] upperLimits = new double[] {lengthX * 0.5, lengthY * 0.5, lengthZ * 0.5};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }

}
