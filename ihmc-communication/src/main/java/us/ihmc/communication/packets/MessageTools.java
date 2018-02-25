package us.ihmc.communication.packets;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.idl.TempPreallocatedList;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class MessageTools
{
   public static final boolean DEBUG = false;

   public static TextToSpeechPacket createTextToSpeechPacket(String textToSpeak)
   {
      if (DEBUG)
         System.out.println("created new TextToSpeechPacket " + textToSpeak);
      TextToSpeechPacket message = new TextToSpeechPacket();
      message.setTextToSpeak(textToSpeak);
      return message;
   }

   public static SimulatedLidarScanPacket createSimulatedLidarScanPacket(int sensorId, LidarScanParameters params, float[] ranges)
   {
      SimulatedLidarScanPacket message = new SimulatedLidarScanPacket();
      message.ranges.add(ranges);
      message.sensorId = sensorId;
      message.lidarScanParameters = new LidarScanParametersMessage();
      message.lidarScanParameters.timestamp = params.timestamp;
      message.lidarScanParameters.sweepYawMax = params.sweepYawMax;
      message.lidarScanParameters.sweepYawMin = params.sweepYawMin;
      message.lidarScanParameters.heightPitchMax = params.heightPitchMax;
      message.lidarScanParameters.heightPitchMin = params.heightPitchMin;
      message.lidarScanParameters.timeIncrement = params.timeIncrement;
      message.lidarScanParameters.scanTime = params.scanTime;
      message.lidarScanParameters.minRange = params.minRange;
      message.lidarScanParameters.maxRange = params.maxRange;
      message.lidarScanParameters.pointsPerSweep = params.pointsPerSweep;
      message.lidarScanParameters.scanHeight = params.scanHeight;
      return message;
   }

   public static InvalidPacketNotificationPacket createInvalidPacketNotificationPacket(Class<? extends Packet<?>> packetClass, String errorMessage)
   {
      InvalidPacketNotificationPacket message = new InvalidPacketNotificationPacket();
      message.setPacketClassSimpleName(packetClass.getSimpleName());
      message.setErrorMessage(errorMessage);
      return message;
   }

   public static LidarScanMessage createLidarScanMessage(long timestamp, Point3D32 lidarPosition, Quaternion32 lidarOrientation, float[] scan)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.robotTimestamp = timestamp;
      message.lidarPosition = lidarPosition;
      message.lidarOrientation = lidarOrientation;
      message.scan.add(scan);
      return message;
   }

   public static ObjectDetectorResultPacket createObjectDetectorResultPacket(HeatMapPacket heatMap, BoundingBoxesPacket boundingBoxes)
   {
      ObjectDetectorResultPacket message = new ObjectDetectorResultPacket();
      message.heatMap = heatMap;
      message.boundingBoxes = boundingBoxes;
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.position = new Point3D(position);
      message.orientation = null;
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position, Quaternion orientation)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.position = new Point3D(position);
      message.orientation = orientation;
      return message;
   }

   public static SetBooleanParameterPacket createSetBooleanParameterPacket(String parameterName, boolean parameterValue)
   {
      SetBooleanParameterPacket message = new SetBooleanParameterPacket();
      message.parameterName.append(parameterName);
      message.parameterValue = parameterValue;
      return message;
   }

   /**
    * Creates a new center of mass message.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param desiredPosition the position that center of mass should reach. The data is assumed to
    *           be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxCenterOfMassMessage createKinematicsToolboxCenterOfMassMessage(Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.desiredPositionInWorld.set(desiredPosition);
      return message;
   }

   public static SetDoubleArrayParameterPacket createSetDoubleArrayParameterPacket(String parameterName, double[] parameterValue)
   {
      SetDoubleArrayParameterPacket message = new SetDoubleArrayParameterPacket();
      message.parameterName.append(parameterName);
      message.parameterValue.add(parameterValue);
      return message;
   }

   public static DetectedFacesPacket createDetectedFacesPacket(String[] ids, Point3D[] positions)
   {
      DetectedFacesPacket message = new DetectedFacesPacket();
      copyData(ids, message.ids);
      copyData(positions, message.positions);
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * Before the message can be sent to the solver, you will need to provide at least a desired
    * orientation and/or desired position.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for linear control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPositionInWorld(desiredPosition);
      message.angularSelectionMatrix.xSelected = false;
      message.angularSelectionMatrix.ySelected = false;
      message.angularSelectionMatrix.zSelected = false;
      message.linearSelectionMatrix.xSelected = true;
      message.linearSelectionMatrix.ySelected = true;
      message.linearSelectionMatrix.zSelected = true;
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for angular control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredOrientationInWorld(desiredOrientation);
      message.angularSelectionMatrix.xSelected = true;
      message.angularSelectionMatrix.ySelected = true;
      message.angularSelectionMatrix.zSelected = true;
      message.linearSelectionMatrix.xSelected = false;
      message.linearSelectionMatrix.ySelected = false;
      message.linearSelectionMatrix.zSelected = false;
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPositionInWorld(desiredPosition);
      message.setDesiredOrientationInWorld(desiredOrientation);
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, ReferenceFrame controlFrame,
                                                                                           Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPositionInWorld(desiredPosition);
      message.setDesiredOrientationInWorld(desiredOrientation);
      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      message.setControlFramePositionInEndEffector(transformToBodyFixedFrame.getTranslationVector());
      message.controlFrameOrientationInEndEffector.set(transformToBodyFixedFrame.getRotationMatrix());
      return message;
   }

   public static SetDoubleParameterPacket createSetDoubleParameterPacket(String parameterName, double parameterValue)
   {
      SetDoubleParameterPacket message = new SetDoubleParameterPacket();
      message.parameterName.append(parameterName);
      message.parameterValue = parameterValue;
      return message;
   }

   /**
    * Copy constructor.
    * 
    * @param selectionMatrix3D the original selection matrix to copy. Not modified.
    */
   public static SelectionMatrix3DMessage createSelectionMatrix3DMessage(SelectionMatrix3D selectionMatrix3D)
   {
      SelectionMatrix3DMessage message = new SelectionMatrix3DMessage();
      message.selectionFrameId = MessageTools.toFrameId(selectionMatrix3D.getSelectionFrame());
      message.xSelected = selectionMatrix3D.isXSelected();
      message.ySelected = selectionMatrix3D.isYSelected();
      message.zSelected = selectionMatrix3D.isZSelected();
      return message;
   }

   public static WeightMatrix3DMessage createWeightMatrix3DMessage(WeightMatrix3D weightMatrix)
   {
      WeightMatrix3DMessage message = new WeightMatrix3DMessage();
      message.weightFrameId = MessageTools.toFrameId(weightMatrix.getWeightFrame());
      message.xWeight = weightMatrix.getXAxisWeight();
      message.yWeight = weightMatrix.getYAxisWeight();
      message.zWeight = weightMatrix.getZAxisWeight();
      return message;
   }

   public static WeightMatrix3DMessage createWeightMatrix3DMessage(double weight)
   {
      WeightMatrix3DMessage message = new WeightMatrix3DMessage();
      message.weightFrameId = MessageTools.toFrameId(null);
      message.xWeight = weight;
      message.yWeight = weight;
      message.zWeight = weight;
      return message;
   }

   public static ParameterListPacket createParameterListPacket(List<Parameter> parameters)
   {
      ParameterListPacket message = new ParameterListPacket();
      message.parameters = parameters;
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(OneDoFJoint[] joints)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(joints);
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData,
                                                                                   boolean useQDesired)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);
      MessageTools.packDesiredJointState(message, rootJoint, newJointData, useQDesired);
      return message;
   }

   public static SetStringParameterPacket createSetStringParameterPacket(String parameterName, String parameterValue)
   {
      SetStringParameterPacket message = new SetStringParameterPacket();
      message.parameterName.append(parameterName);
      message.parameterValue.append(parameterValue);
      return message;
   }

   public static BoundingBoxesPacket createBoundingBoxesPacket(int[] packedBoxes, String[] labels)
   {
      BoundingBoxesPacket message = new BoundingBoxesPacket();
      MessageTools.copyData(labels, message.labels);
      int n = packedBoxes.length / 4;

      for (int i = 0; i < n; i++)
      {
         message.boundingBoxXCoordinates.add(packedBoxes[i * 4]);
         message.boundingBoxYCoordinates.add(packedBoxes[i * 4 + 1]);
         message.boundingBoxWidths.add(packedBoxes[i * 4 + 2]);
         message.boundingBoxHeights.add(packedBoxes[i * 4 + 3]);
      }
      return message;
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(ControllerCrashLocation location, String stackTrace)
   {
      ControllerCrashNotificationPacket message = new ControllerCrashNotificationPacket();
      message.controllerCrashLocation = location.toByte();
      message.stacktrace.append(stackTrace);
      return message;
   }

   public static StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(long timestamp, float[] pointCloud, int[] colors)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.robotTimestamp = timestamp;
      message.pointCloud.add(pointCloud);
      message.colors.add(colors);
      return message;
   }

   public static ToolboxStateMessage createToolboxStateMessage(ToolboxState requestedState)
   {
      ToolboxStateMessage message = new ToolboxStateMessage();
      message.requestedToolboxState = requestedState.toByte();
      return message;
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType,
                                                                                       BoundingBox3D boundingBoxInWorldForRequest)
   {
      return createRequestPlanarRegionsListMessage(requestType, boundingBoxInWorldForRequest, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType, PacketDestination destination)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, destination);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType,
                                                                                       BoundingBox3D boundingBoxInWorldForRequest,
                                                                                       PacketDestination destination)
   {
      RequestPlanarRegionsListMessage message = new RequestPlanarRegionsListMessage();
      message.planarRegionsRequestType = requestType.toByte();
      message.boundingBoxInWorldForRequest = new BoundingBox3DMessage();
      if (boundingBoxInWorldForRequest != null)
      {
         message.boundingBoxInWorldForRequest.minPoint.set(boundingBoxInWorldForRequest.getMinPoint());
         message.boundingBoxInWorldForRequest.maxPoint.set(boundingBoxInWorldForRequest.getMaxPoint());
      }
      if (destination != null)
         message.setDestination(destination);
      return message;
   }

   public static PlanarRegionsListMessage createPlanarRegionsListMessage(List<PlanarRegionMessage> planarRegions)
   {
      PlanarRegionsListMessage message = new PlanarRegionsListMessage();
      copyData(planarRegions, message.planarRegions);
      return message;
   }

   public static <T extends Enum<T>> T fromByteToEnum(byte value, Class<T> enumType)
   {
      return enumType.getEnumConstants()[(int) value];
   }

   public static LidarScanParameters toLidarScanParameters(LidarScanParametersMessage message)
   {
      return new LidarScanParameters(message.pointsPerSweep, message.scanHeight, message.sweepYawMin, message.sweepYawMax, message.heightPitchMin,
                                     message.heightPitchMax, message.timeIncrement, message.minRange, message.maxRange, message.scanTime, message.timestamp);
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TByteArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TByteArrayList source, TByteArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TDoubleArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TDoubleArrayList source, TDoubleArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TFloatArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TFloatArrayList source, TFloatArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TIntArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TIntArrayList source, TIntArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TLongArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TLongArrayList source, TLongArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Performs a deep copy of the data from {@code source} to {@code destination} after calling
    * {@link TempPreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    * @param <T> Should be either {@code Enum}, {@code StringBuilder}, or {@code Settable<T>}.
    * @throws IllegalArgumentException if the type {@code T} is none of the following: {@code Enum},
    *            {@code StringBuilder}, {@code Settable<T>}.
    */
   @SuppressWarnings("unchecked")
   public static <T> void copyData(TempPreallocatedList<T> source, TempPreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null || source.isEmpty())
         return;

      if (source.isEnum())
      {
         for (int i = 0; i < source.size(); i++)
         {
            destination.add(source.get(i));
         }
      }
      else
      {
         T firstElement = destination.add();

         if (firstElement instanceof Settable)
         {
            destination.resetQuick();

            for (int i = 0; i < source.size(); i++)
            {
               ((Settable<T>) destination.add()).set(source.get(i));
            }
         }
         else if (firstElement instanceof StringBuilder)
         {
            destination.resetQuick();

            for (int i = 0; i < source.size(); i++)
            {
               StringBuilder destinationElement = (StringBuilder) destination.add();
               destinationElement.setLength(0);
               destinationElement.append((StringBuilder) source.get(i));
            }
         }
         else
         {
            throw new IllegalArgumentException(MessageTools.class.getSimpleName() + ".copyData(...) can only be used with "
                  + TempPreallocatedList.class.getSimpleName() + "s declared with either of the following types: Enum, StringBuilder, and"
                  + Settable.class.getSimpleName());
         }
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TempPreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends Settable<T>> void copyData(List<T> source, TempPreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      try
      {
         for (int i = 0; i < source.size(); i++)
         {
            destination.add().set(source.get(i));
         }
      }
      catch (ArrayIndexOutOfBoundsException e)
      {
         PrintTools.error("Caught exception while copying data from list of size: " + source.size());
         throw e;
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TempPreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends Settable<T>> void copyData(T[] source, TempPreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      try
      {
         for (int i = 0; i < source.length; i++)
         {
            destination.add().set(source[i]);
         }
      }
      catch (ArrayIndexOutOfBoundsException e)
      {
         PrintTools.error("Caught exception while copying data from array of length: " + source.length);
         throw e;
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TempPreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(String[] source, TempPreallocatedList<StringBuilder> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.length; i++)
      {
         StringBuilder destinationElement = destination.add();
         destinationElement.setLength(0);
         destinationElement.append(source[i]);
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TempPreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(StringBuilder[] source, TempPreallocatedList<StringBuilder> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.length; i++)
      {
         StringBuilder destinationElement = destination.add();
         destinationElement.setLength(0);
         destinationElement.append(source[i]);
      }
   }

   public static <T> List<T> toList(TempPreallocatedList<T> original)
   {
      List<T> list = new ArrayList<>();
      for (int i = 0; i < original.size(); i++)
         list.add(original.get(i));
      return list;
   }

   public static <T extends EpsilonComparable<T>> boolean epsilonEquals(TempPreallocatedList<T> listOne, TempPreallocatedList<T> listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!listOne.get(i).epsilonEquals(listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static boolean epsilonEquals(TDoubleArrayList listOne, TDoubleArrayList listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!MathTools.epsilonEquals(listOne.get(i), listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static boolean epsilonEquals(TFloatArrayList listOne, TFloatArrayList listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!MathTools.epsilonEquals(listOne.get(i), listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static long toFrameId(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
         return NameBasedHashCodeTools.NULL_HASHCODE;
      else
         return referenceFrame.getNameBasedHashCode();
   }

   public static Point3D32[] unpackPointCloud32(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      Point3D32[] points = new Point3D32[stereoVisionPointCloudMessage.pointCloud.size() / 3];
      for (int index = 0; index < stereoVisionPointCloudMessage.pointCloud.size() / 3; index++)
      {
         Point3D32 scanPoint = new Point3D32();
         scanPoint.setX(stereoVisionPointCloudMessage.pointCloud.get(3 * index + 0));
         scanPoint.setY(stereoVisionPointCloudMessage.pointCloud.get(3 * index + 1));
         scanPoint.setZ(stereoVisionPointCloudMessage.pointCloud.get(3 * index + 2));
         points[index] = scanPoint;
      }
      return points;
   }

   public static Color[] unpackPointCloudColors(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      Color[] colors = new Color[stereoVisionPointCloudMessage.colors.size()];

      for (int i = 0; i < stereoVisionPointCloudMessage.colors.size(); i++)
      {
         colors[i] = new Color(stereoVisionPointCloudMessage.colors.get(i));
      }

      return colors;
   }

   public static void unpackDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FloatingInverseDynamicsJoint rootJointToUpdate,
                                              OneDoFJoint[] jointsToUpdate)
   {
      int jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(jointsToUpdate);

      if (jointNameHash != kinematicsToolboxOutputStatus.jointNameHash)
         throw new RuntimeException("The robots are different.");

      for (int i = 0; i < kinematicsToolboxOutputStatus.desiredJointAngles.size(); i++)
         jointsToUpdate[i].setQ(kinematicsToolboxOutputStatus.desiredJointAngles.get(i));

      rootJointToUpdate.setPosition(kinematicsToolboxOutputStatus.desiredRootTranslation);
      rootJointToUpdate.setRotation(kinematicsToolboxOutputStatus.desiredRootOrientation);
   }

   public static void packDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FloatingInverseDynamicsJoint rootJoint,
                                            OneDoFJoint[] newJointData, boolean useQDesired)
   {
      int jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);
      
      if (jointNameHash != kinematicsToolboxOutputStatus.jointNameHash)
         throw new RuntimeException("The robots are different.");
      
      kinematicsToolboxOutputStatus.desiredJointAngles.reset();
      
      if (useQDesired)
      {
         for (int i = 0; i < newJointData.length; i++)
         {
            kinematicsToolboxOutputStatus.desiredJointAngles.add((float) newJointData[i].getqDesired());
         }
      }
      else
      {
         for (int i = 0; i < newJointData.length; i++)
         {
            kinematicsToolboxOutputStatus.desiredJointAngles.add((float) newJointData[i].getQ());
         }
      }
      
      if (rootJoint != null)
      {
         rootJoint.getTranslation(kinematicsToolboxOutputStatus.desiredRootTranslation);
         rootJoint.getRotation(kinematicsToolboxOutputStatus.desiredRootOrientation);
      }
   }

   public static KinematicsToolboxOutputStatus interpolateMessages(KinematicsToolboxOutputStatus outputStatusOne, KinematicsToolboxOutputStatus outputStatusTwo,
                                                                   double alpha)
   {
      if (outputStatusOne.jointNameHash != outputStatusTwo.jointNameHash)
         throw new RuntimeException("Output status are not compatible.");
      
      KinematicsToolboxOutputStatus interplateOutputStatus = new KinematicsToolboxOutputStatus();
      
      TFloatArrayList jointAngles1 = outputStatusOne.getDesiredJointAngles();
      TFloatArrayList jointAngles2 = outputStatusTwo.getDesiredJointAngles();
      
      for (int i = 0; i < jointAngles1.size(); i++)
      {
         interplateOutputStatus.desiredJointAngles.add((float) EuclidCoreTools.interpolate(jointAngles1.get(i), jointAngles2.get(i), alpha));
      }
      
      Vector3D32 rootTranslation1 = outputStatusOne.getDesiredRootTranslation();
      Vector3D32 rootTranslation2 = outputStatusTwo.getDesiredRootTranslation();
      Quaternion32 rootOrientation1 = outputStatusOne.getDesiredRootOrientation();
      Quaternion32 rootOrientation2 = outputStatusTwo.getDesiredRootOrientation();
      
      interplateOutputStatus.desiredRootTranslation.interpolate(rootTranslation1, rootTranslation2, alpha);
      interplateOutputStatus.desiredRootOrientation.interpolate(rootOrientation1, rootOrientation2, alpha);
      
      interplateOutputStatus.jointNameHash = outputStatusOne.jointNameHash;
      
      return interplateOutputStatus;
   }

   /**
    * Provides a privileged configuration that the {@code KinematicsToolboxController} will use as a
    * reference and attempt to find the solution that is the closest.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param rootJointPosition the privileged root joint position. Not modified.
    * @param rootJointOrientation the privileged root joint orientation. Not modified.
    * @param jointNameBasedHashCodes allows to safely identify to which joint each angle in
    *           {@link #privilegedJointAngles} belongs to. The name-based hash code can be obtained
    *           from {@link OneDoFJoint#getNameBasedHashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and
    *            {@code jointNameBasedHashCodes} are different.
    */
   public static void packPrivilegedRobotConfiguration(KinematicsToolboxConfigurationMessage kinematicsToolboxConfigurationMessage,
                                                       Tuple3DReadOnly rootJointPosition, QuaternionReadOnly rootJointOrientation,
                                                       long[] jointNameBasedHashCodes, float[] jointAngles)
   {
      kinematicsToolboxConfigurationMessage.setPrivilegedRootJointPosition(rootJointPosition);
      kinematicsToolboxConfigurationMessage.setPrivilegedRootJointOrientation(rootJointOrientation);
      MessageTools.packPrivilegedJointAngles(kinematicsToolboxConfigurationMessage, jointNameBasedHashCodes, jointAngles);
   }

   /**
    * When provided, the {@code KinematicsToolboxController} will attempt to find the closest
    * solution to the privileged configuration.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param jointNameBasedHashCodes allows to safely identify to which joint each angle in
    *           {@link #privilegedJointAngles} belongs to. The name-based hash code can be obtained
    *           from {@link OneDoFJoint#getNameBasedHashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and
    *            {@code jointNameBasedHashCodes} are different.
    */
   public static void packPrivilegedJointAngles(KinematicsToolboxConfigurationMessage kinematicsToolboxConfigurationMessage, long[] jointNameBasedHashCodes,
                                                float[] jointAngles)
   {
      if (jointNameBasedHashCodes.length != jointAngles.length)
         throw new IllegalArgumentException("The two arrays jointAngles and jointNameBasedHashCodes have to be of same length.");
      
      kinematicsToolboxConfigurationMessage.privilegedJointNameBasedHashCodes.reset();
      kinematicsToolboxConfigurationMessage.privilegedJointNameBasedHashCodes.add(jointNameBasedHashCodes);
      kinematicsToolboxConfigurationMessage.privilegedJointAngles.reset();
      kinematicsToolboxConfigurationMessage.privilegedJointAngles.add(jointAngles);
   }

   public static void packScan(LidarScanMessage lidarScanMessage, Point3DReadOnly[] scan)
   {
      lidarScanMessage.scan.reset();
      
      for (Point3DReadOnly scanPoint : scan)
      {
         lidarScanMessage.scan.add((float) scanPoint.getX());
         lidarScanMessage.scan.add((float) scanPoint.getY());
         lidarScanMessage.scan.add((float) scanPoint.getZ());
      }
   }

   public static void unpackScanPoint(LidarScanMessage lidarScanMessage, int index, Point3DBasics scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(lidarScanMessage.scan.get(index++));
      scanPointToPack.setY(lidarScanMessage.scan.get(index++));
      scanPointToPack.setZ(lidarScanMessage.scan.get(index++));
   }

   public static Point3D[] unpackScanPoint3ds(LidarScanMessage lidarScanMessage)
   {
      int numberOfScanPoints = lidarScanMessage.scan.size() / 3;
      Point3D[] scanPoints = new Point3D[numberOfScanPoints];
      for (int index = 0; index < numberOfScanPoints; index++)
      {
         Point3D scanPoint1 = new Point3D();
         MessageTools.unpackScanPoint(lidarScanMessage, index, scanPoint1);
         Point3D scanPoint = scanPoint1;
         scanPoints[index] = scanPoint;
      }
      return scanPoints;
   }
}
