package us.ihmc.communication.packets;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link KinematicsToolboxRigidBodyMessage} is part of the API of the
 * {@code KinematicsToolboxController}.
 * <p>
 * It holds all the information needed for the {@code KinematicsToolboxController} to constrain the
 * given end-effector to certain number of task-space constraints, i.e. desired
 * position/orientation.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxRigidBodyMessage extends Packet<KinematicsToolboxRigidBodyMessage>
{
   /**
    * This is the unique hash code of the end-effector to be solved for. It used on the solver side
    * to retrieve the desired end-effector to be controlled.
    */
   public long endEffectorNameBasedHashCode;
   /**
    * This is the desired position of the control frame's origin. If the control frame has not been
    * defined, it represents the desired position of {@code endEffector.getBodyFixedFrame()}'s
    * origin. The data is assumed to be expressed in world frame.
    */
   public Point3D32 desiredPositionInWorld;
   /**
    * This is the desired orientation of the control frame. If the control frame has not been
    * defined, it represents the desired orientation of {@code endEffector.getBodyFixedFrame()}. The
    * data is assumed to be expressed in world frame.
    */
   public Quaternion32 desiredOrientationInWorld;
   /**
    * This is the position of the control frame's origin expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Point3D32 controlFramePositionInEndEffector;
   /**
    * This is the orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Quaternion32 controlFrameOrientationInEndEffector;

   // TODO Add doc
   public SelectionMatrix3DMessage angularSelectionMatrix;
   public SelectionMatrix3DMessage linearSelectionMatrix;

   /**
    * Weight Matrix used to define the priority of controlling the rotation around each axis on the
    * solver side:<br>
    */
   public WeightMatrix3DMessage angularWeightMatrix;

   /**
    * Weight Matrix used to define the priority of controlling the translation of each axis on the
    * solver side:<br>
    */
   public WeightMatrix3DMessage linearWeightMatrix;

   /**
    * Do not use this constructor, it is needed only for efficient serialization/deserialization.
    */
   public KinematicsToolboxRigidBodyMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(KinematicsToolboxRigidBodyMessage other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      if (other.desiredPositionInWorld != null)
         desiredPositionInWorld = new Point3D32(other.desiredPositionInWorld);
      if (other.desiredOrientationInWorld != null)
         desiredOrientationInWorld = new Quaternion32(other.desiredOrientationInWorld);
      if (other.controlFramePositionInEndEffector != null)
         controlFramePositionInEndEffector = new Point3D32(other.controlFramePositionInEndEffector);
      controlFrameOrientationInEndEffector = new Quaternion32(other.controlFrameOrientationInEndEffector);
      if (other.angularSelectionMatrix != null)
      {
         angularSelectionMatrix = new SelectionMatrix3DMessage();
         angularSelectionMatrix.set(other.angularSelectionMatrix);
      }
      if (other.linearSelectionMatrix != null)
      {
         linearSelectionMatrix = new SelectionMatrix3DMessage();
         linearSelectionMatrix.set(other.linearSelectionMatrix);
      }
      if (other.angularWeightMatrix != null)
      {
         angularWeightMatrix = new WeightMatrix3DMessage();
         angularWeightMatrix.set(other.angularWeightMatrix);
      }
      if (other.linearWeightMatrix != null)
      {
         linearWeightMatrix = new WeightMatrix3DMessage();
         linearWeightMatrix.set(other.linearWeightMatrix);
      }
      setPacketInformation(other);
   }

   /**
    * Sets the desired position that the control frame's origin should reach. By default the control
    * frame is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    */
   public void setDesiredPosition(Point3DReadOnly desiredPosition)
   {
      if (desiredPositionInWorld == null)
         desiredPositionInWorld = new Point3D32(desiredPosition);
      else
         desiredPositionInWorld.set(desiredPosition);
   }

   /**
    * Sets the desired position that the control frame's origin should reach. By default the control
    * frame is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredPosition(FramePoint3D desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPosition(desiredPosition);
   }

   /**
    * Sets the desired orientation that the control frame should reach. By default the control frame
    * is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed
    * in world frame.
    * 
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    */
   public void setDesiredOrientation(QuaternionReadOnly desiredOrientation)
   {
      if (desiredOrientationInWorld == null)
         desiredOrientationInWorld = new Quaternion32(desiredOrientation);
      else
         desiredOrientationInWorld.set(desiredOrientation);
   }

   /**
    * Sets the desired orientation that the control frame should reach. By default the control frame
    * is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed
    * in world frame.
    * 
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredOrientation(FrameQuaternion desiredOrientation)
   {
      desiredOrientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredOrientation(desiredOrientation);
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    */
   public void setDesiredPose(Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation)
   {
      setDesiredPosition(desiredPosition);
      setDesiredOrientation(desiredOrientation);
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPose the pose the control frame should reach. Not modified.
    */
   public void setDesiredPose(Pose3DReadOnly pose)
   {
      setDesiredPosition(pose.getPosition());
      setDesiredOrientation(pose.getOrientation());
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in world
    *            frame.
    */
   public void setDesiredPose(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation)
   {
      setDesiredPosition(desiredPosition);
      setDesiredOrientation(desiredOrientation);
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPose the pose the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredPose(FramePose3D desiredPose)
   {
      desiredPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPose((Pose3DReadOnly) desiredPose);
   }

   /** Ensures that the weight matrix's are initialized. */
   private void initializeWeight()
   {
      if (linearWeightMatrix == null)
      {
         linearWeightMatrix = new WeightMatrix3DMessage();
      }

      if (angularWeightMatrix == null)
      {
         angularWeightMatrix = new WeightMatrix3DMessage();
      }

   }

   /**
    * Sets the weight to use for this task.
    * <p>
    * The weight relates to the priority of a task relative to the other active tasks. A higher
    * weight refers to a higher priority.
    * </p>
    * 
    * @param weight the weight value for this task.
    */
   public void setWeight(double weight)
   {
      initializeWeight();
      angularWeightMatrix.xWeight = weight;
      angularWeightMatrix.yWeight = weight;
      angularWeightMatrix.zWeight = weight;
      linearWeightMatrix.xWeight = weight;
      linearWeightMatrix.yWeight = weight;
      linearWeightMatrix.zWeight = weight;
   }

   /**
    * Sets the weight to use for this task with the angular and linear parts set independently.
    * <p>
    * The weight relates to the priority of a task relative to the other active tasks. A higher
    * weight refers to a higher priority.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this task.
    * @param linear the weight to use for the linear part of this task.
    */
   public void setWeight(double angular, double linear)
   {
      initializeWeight();
      angularWeightMatrix.xWeight = angular;
      angularWeightMatrix.yWeight = angular;
      angularWeightMatrix.zWeight = angular;
      linearWeightMatrix.xWeight = linear;
      linearWeightMatrix.yWeight = linear;
      linearWeightMatrix.zWeight = linear;
   }

   /**
    * Sets the weight to use for this task with the angular and linear parts set independently.
    * <p>
    * The weight relates to the priority of a task relative to the other active tasks. A higher
    * weight refers to a higher priority.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this task.
    * @param linear the weight to use for the linear part of this task.
    */
   public void setWeight(WeightMatrix6D weightMatrix)
   {
      initializeWeight();
      WeightMatrix3D weightMatrix1 = weightMatrix.getLinearPart();
      linearWeightMatrix.weightFrameId = MessageTools.toFrameId(weightMatrix1.getWeightFrame());
      linearWeightMatrix.xWeight = weightMatrix1.getXAxisWeight();
      linearWeightMatrix.yWeight = weightMatrix1.getYAxisWeight();
      linearWeightMatrix.zWeight = weightMatrix1.getZAxisWeight();
      WeightMatrix3D weightMatrix2 = weightMatrix.getAngularPart();
      angularWeightMatrix.weightFrameId = MessageTools.toFrameId(weightMatrix2.getWeightFrame());
      angularWeightMatrix.xWeight = weightMatrix2.getXAxisWeight();
      angularWeightMatrix.yWeight = weightMatrix2.getYAxisWeight();
      angularWeightMatrix.zWeight = weightMatrix2.getZAxisWeight();
   }

   /**
    * Enables the control of all the degrees of freedom of the end-effector.
    */
   public void setSelectionMatrixToIdentity()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix = new SelectionMatrix3DMessage();
   }

   /**
    * Enables the control for the translational degrees of freedom of the end-effector and disable
    * the rotational part.
    */
   public void setSelectionMatrixForLinearControl()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      angularSelectionMatrix.xSelected = false;
      angularSelectionMatrix.ySelected = false;
      angularSelectionMatrix.zSelected = false;
      linearSelectionMatrix = new SelectionMatrix3DMessage();
   }

   /**
    * Enables the control for the rotational degrees of freedom of the end-effector and disable the
    * translational part.
    */
   public void setSelectionMatrixForAngularControl()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.xSelected = false;
      linearSelectionMatrix.ySelected = false;
      linearSelectionMatrix.zSelected = false;
   }

   /**
    * Sets the selection matrix to use for executing this message.
    * <p>
    * The selection matrix is used to determinate which degree of freedom of the end-effector should
    * be controlled. When it is NOT provided, the controller will assume that all the degrees of
    * freedom of the end-effector should be controlled.
    * </p>
    * <p>
    * The selection frames coming along with the given selection matrix are used to determine to
    * what reference frame the selected axes are referring to. For instance, if only the hand height
    * in world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should be world frame. When no reference frame is provided
    * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
    * frame if not defined otherwise.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to use when executing this trajectory message. Not
    *           modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix6D)
   {
      if (angularSelectionMatrix == null)
         angularSelectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart());
      else
      {
         angularSelectionMatrix.selectionFrameId = MessageTools.toFrameId(selectionMatrix6D.getAngularSelectionFrame());
         angularSelectionMatrix.xSelected = selectionMatrix6D.isAngularXSelected();
         angularSelectionMatrix.ySelected = selectionMatrix6D.isAngularYSelected();
         angularSelectionMatrix.zSelected = selectionMatrix6D.isAngularZSelected();
      }

      if (linearSelectionMatrix == null)
         linearSelectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart());
      else
      {
         linearSelectionMatrix.selectionFrameId = MessageTools.toFrameId(selectionMatrix6D.getLinearSelectionFrame());
         linearSelectionMatrix.xSelected = selectionMatrix6D.isLinearXSelected();
         linearSelectionMatrix.ySelected = selectionMatrix6D.isLinearYSelected();
         linearSelectionMatrix.zSelected = selectionMatrix6D.isLinearZSelected();
      }
   }

   /**
    * Resets the control frame so it is coincident with {@code endEffector.getBodyFixedFrame()}.
    */
   public void resetControlFrame()
   {
      if (controlFramePositionInEndEffector != null)
         controlFramePositionInEndEffector.setToZero();
      if (controlFrameOrientationInEndEffector != null)
         controlFrameOrientationInEndEffector.setToZero();
   }

   /**
    * Specifies the position of the control frame's origin. The given point is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePosition(Tuple3DReadOnly controlFramePosition)
   {
      if (controlFramePositionInEndEffector == null)
         controlFramePositionInEndEffector = new Point3D32(controlFramePosition);
      else
         controlFramePositionInEndEffector.set(controlFramePosition);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(QuaternionReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(RotationMatrixReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, QuaternionReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, RotationMatrixReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given pose is assumed to be expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePose the pose of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Pose3D controlFramePose)
   {
      setControlFramePosition(controlFramePose.getPosition());
      setControlFrameOrientation(controlFramePose.getOrientation());
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public void getDesiredPose(FramePose3D desiredPoseToPack)
   {
      desiredPoseToPack.setToZero(ReferenceFrame.getWorldFrame());

      if (desiredPositionInWorld != null)
         desiredPoseToPack.setPosition(desiredPositionInWorld);
      if (desiredOrientationInWorld != null)
         desiredPoseToPack.setOrientation(desiredOrientationInWorld);
   }

   public void getControlFramePose(RigidBody endEffector, FramePose3D controlFramePoseToPack)
   {
      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePoseToPack.setToZero(referenceFrame);

      if (controlFramePositionInEndEffector != null)
         controlFramePoseToPack.setPosition(controlFramePositionInEndEffector);
      if (controlFrameOrientationInEndEffector != null)
         controlFramePoseToPack.setOrientation(controlFrameOrientationInEndEffector);
   }

   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.resetSelection();
      if (angularSelectionMatrix != null)
      {
         SelectionMatrix3D selectionMatrix3D = selectionMatrixToPack.getAngularPart();
         selectionMatrix3D.clearSelection();
         selectionMatrix3D.selectXAxis(angularSelectionMatrix.xSelected);
         selectionMatrix3D.selectYAxis(angularSelectionMatrix.ySelected);
         selectionMatrix3D.selectZAxis(angularSelectionMatrix.zSelected);
      }
      if (linearSelectionMatrix != null)
      {
         SelectionMatrix3D selectionMatrix3D1 = selectionMatrixToPack.getLinearPart();
         selectionMatrix3D1.clearSelection();
         selectionMatrix3D1.selectXAxis(linearSelectionMatrix.xSelected);
         selectionMatrix3D1.selectYAxis(linearSelectionMatrix.ySelected);
         selectionMatrix3D1.selectZAxis(linearSelectionMatrix.zSelected);
      }
   }

   public void getWeightMatrix(WeightMatrix6D weightMatrixToPack)
   {
      weightMatrixToPack.clear();
      if (angularWeightMatrix != null)
      {
         WeightMatrix3D weightMatrix3D = weightMatrixToPack.getAngularPart();
         weightMatrix3D.clearWeightFrame();
         weightMatrix3D.setWeights(angularWeightMatrix.xWeight, angularWeightMatrix.yWeight, angularWeightMatrix.zWeight);
      }
      if (linearWeightMatrix != null)
      {
         WeightMatrix3D weightMatrix3D1 = weightMatrixToPack.getLinearPart();
         weightMatrix3D1.clearWeightFrame();
         weightMatrix3D1.setWeights(linearWeightMatrix.xWeight, linearWeightMatrix.yWeight, linearWeightMatrix.zWeight);
      }
   }

   /**
    * Returns the unique ID referring to the selection frame to use with the angular part of the
    * selection matrix of this message.
    * <p>
    * If this message does not have a angular selection matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the selection frame ID for the angular part of the selection matrix.
    */
   public long getAngularSelectionFrameId()
   {
      if (angularSelectionMatrix != null)
         return angularSelectionMatrix.getSelectionFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   /**
    * Returns the unique ID referring to the selection frame to use with the linear part of the
    * selection matrix of this message.
    * <p>
    * If this message does not have a linear selection matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the selection frame ID for the linear part of the selection matrix.
    */
   public long getLinearSelectionFrameId()
   {
      if (linearSelectionMatrix != null)
         return linearSelectionMatrix.getSelectionFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   /**
    * Returns the unique ID referring to the frame to use with the linear part of the weight matrix
    * of this message.
    * <p>
    * If this message does not have a linear weight matrix or the frame has not been set, this
    * method returns {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the frame ID for the linear part of the weight matrix.
    */
   public long getLinearWeightFrameId()
   {
      if (linearWeightMatrix != null)
         return linearWeightMatrix.getWeightFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   /**
    * Returns the unique ID referring to the frame to use with the angular part of the weight matrix
    * of this message.
    * <p>
    * If this message does not have a angular weight matrix or the frame has not been set, this
    * method returns {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the frame ID for the linear part of the weight matrix.
    */
   public long getAngularWeightFrameId()
   {
      if (angularWeightMatrix != null)
         return angularWeightMatrix.getWeightFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   /**
    * Compares each field of this message against the other message and returns {@code true} if they
    * are equal to an {@code epsilon}.
    * <p>
    * Note that this method considers two fields to be equal if they are both {@code null}, and
    * considers two fields to be different if only one is equal to {@code null}.
    * </p>
    * 
    * @return {@code true} if the two messages are equal to an {@code epsilon}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(KinematicsToolboxRigidBodyMessage other, double epsilon)
   {
      if (endEffectorNameBasedHashCode != other.endEffectorNameBasedHashCode)
         return false;
      if (!nullEqualsAndEpsilonEquals(desiredPositionInWorld, other.desiredPositionInWorld, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(desiredOrientationInWorld, other.desiredOrientationInWorld, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(controlFramePositionInEndEffector, other.controlFramePositionInEndEffector, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(controlFrameOrientationInEndEffector, other.controlFrameOrientationInEndEffector, epsilon))
         return false;

      // TODO Add the selection matrix back in here
      if (linearWeightMatrix == null ^ other.linearWeightMatrix == null)//bit wise or
      {
         return false;
      }
      if (angularWeightMatrix == null ^ other.angularWeightMatrix == null)//bit wise or
      {
         return false;
      }

      if (linearWeightMatrix != null && !linearWeightMatrix.epsilonEquals(linearWeightMatrix, epsilon))
      {
         return false;
      }

      if (angularWeightMatrix != null && !angularWeightMatrix.epsilonEquals(angularWeightMatrix, epsilon))
      {
         return false;
      }

      return true;
   }

   /**
    * Convenience method that first performs {@code null} tests before returning the result from
    * {@link EpsilonComparable#epsilonEquals(Object, double)}.
    * 
    * @param a the first object to compare. Not modified.
    * @param b the second object to compare. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the object are either {@code null} or not {@code null} and
    *         epsilon-equal, {@code false} if only one object is {@code null} or if both objects are
    *         not {@code null} but are not epsilon-equal.
    */
   static <T extends EpsilonComparable<T>> boolean nullEqualsAndEpsilonEquals(T a, T b, double epsilon)
   {
      if (a == null && b == null)
         return true;
      if (a == null && b != null)
         return false;
      if (a != null && b == null)
         return false;
      return a.epsilonEquals(b, epsilon);
   }
}
