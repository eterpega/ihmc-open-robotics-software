package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.idl.TempPreallocatedList;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "This message commands the controller to execute a list of footsteps. See FootstepDataMessage for information about defining a footstep."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/footstep_list")
public class FootstepDataListMessage extends Packet<FootstepDataListMessage>
{
   public static final byte EXECUTION_TIMING_CONTROL_DURATIONS = 0;
   public static final byte EXECUTION_TIMING_CONTROL_ABSOLUTE_TIMINGS = 1;

   @RosExportedField(documentation = "Defines the list of footstep to perform.")
   public TempPreallocatedList<FootstepDataMessage> footstepDataList = new TempPreallocatedList<>(FootstepDataMessage.class, FootstepDataMessage::new, 100);

   @RosExportedField(documentation = "When CONTROL_DURATIONS is chosen:"
         + "\n The controller will try to achieve the swingDuration and the transferDuration specified in the message. If a"
         + "\n footstep touches down early, the next step will not be affected by this and the whole trajectory might finish" + "\n earlier then expected."
         + "\nWhen CONTROL_ABSOLUTE_TIMINGS is chosen:"
         + "\n The controller will compute the expected times for swing start and touchdown and attempt to start a footstep"
         + "\n at that time. If a footstep touches down early, the following transfer will be extended to make up for this"
         + "\n time difference and the footstep plan will finish at the expected time.")
   public byte executionTiming = ExecutionTiming.CONTROL_DURATIONS.toByte();

   @RosExportedField(documentation = "The swingDuration is the time a foot is not in ground contact during a step."
         + "\nEach step in a list of footsteps might have a different swing duration. The value specified here is a default"
         + "\nvalue, used if a footstep in this list was created without a swingDuration.")
   public double defaultSwingDuration = 0.0;
   @RosExportedField(documentation = "The transferDuration is the time spent with the feet in ground contact before a step."
         + "\nEach step in a list of footsteps might have a different transfer duration. The value specified here is a default"
         + "\nvalue, used if a footstep in this list was created without a transferDuration.")
   public double defaultTransferDuration = -1.0;

   @RosExportedField(documentation = "Specifies the time used to return to a stable standing stance after the execution of the"
         + "\nfootstep list is finished. If the value is negative the defaultTransferDuration will be used.")
   public double finalTransferDuration = -1.0;

   @RosExportedField(documentation = "If false the controller adjust each footstep height to be at the support sole height.")
   public boolean trustHeightOfFootsteps = true;
   @RosExportedField(documentation = "Contains information on whether the robot can automatically adjust its footsteps to retain balance.")
   public boolean areFootstepsAdjustable = true;

   @RosExportedField(documentation = "If true the controller will adjust upcoming footsteps with the location error of previous steps.")
   public boolean offsetFootstepsWithExecutionError = false;

   @RosExportedField(documentation = "Properties for queueing footstep lists.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootstepDataListMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public FootstepDataListMessage(FootstepDataListMessage other)
   {
      MessageTools.copyData(other.footstepDataList, footstepDataList);
      executionTiming = other.executionTiming;
      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      trustHeightOfFootsteps = other.trustHeightOfFootsteps;
      areFootstepsAdjustable = other.areFootstepsAdjustable;
      offsetFootstepsWithExecutionError = other.offsetFootstepsWithExecutionError;
      if (other.queueingProperties != null)
         queueingProperties.set(other.queueingProperties);
   }

   @Override
   public void set(FootstepDataListMessage other)
   {
      MessageTools.copyData(other.footstepDataList, footstepDataList);
      executionTiming = other.executionTiming;
      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      trustHeightOfFootsteps = other.trustHeightOfFootsteps;
      areFootstepsAdjustable = other.areFootstepsAdjustable;
      offsetFootstepsWithExecutionError = other.offsetFootstepsWithExecutionError;
      if (other.queueingProperties != null)
         queueingProperties.set(other.queueingProperties);

      setPacketInformation(other);
   }

   public TempPreallocatedList<FootstepDataMessage> getFootstepDataList()
   {
      return footstepDataList;
   }

   @Override
   public boolean epsilonEquals(FootstepDataListMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(footstepDataList, other.footstepDataList, epsilon))
         return false;

      if (!MathTools.epsilonCompare(this.defaultSwingDuration, other.defaultSwingDuration, epsilon))
      {
         return false;
      }

      if (!MathTools.epsilonCompare(this.defaultTransferDuration, other.defaultTransferDuration, epsilon))
      {
         return false;
      }

      if (this.executionTiming != other.executionTiming)
      {
         return false;
      }

      if (!MathTools.epsilonCompare(this.finalTransferDuration, other.finalTransferDuration, epsilon))
      {
         return false;
      }

      if (this.offsetFootstepsWithExecutionError != other.offsetFootstepsWithExecutionError)
      {
         return false;
      }

      return true;
   }

   @Override
   public String toString()
   {
      String startingFootstep = "";
      int numberOfSteps = this.footstepDataList.size();
      if (numberOfSteps > 0)
      {
         startingFootstep = this.footstepDataList.get(0).getLocation().toString();
         Quaternion quat4d = this.footstepDataList.get(0).getOrientation();

         FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), quat4d);
         startingFootstep = startingFootstep + ", yaw= " + frameOrientation.getYaw() + ", pitch= " + frameOrientation.getPitch() + ", roll= "
               + frameOrientation.getRoll();
      }

      if (this.footstepDataList.size() == 1)
      {
         RobotSide footSide = RobotSide.fromByte(footstepDataList.get(0).getRobotSide());
         String side = "Right Step";
         if (footSide.getSideNameFirstLetter().equals("L"))
            side = "Left Step";

         return (side);
      }
      else
      {
         return ("Starting Footstep: " + startingFootstep + "\n" + "\tExecution Mode: "
               + ExecutionMode.fromByte(queueingProperties.getExecutionMode()).toString() + "\n" + "\tExecution Timing: " + this.executionTiming + "\n"
               + "\tTransfer Duration: " + this.defaultTransferDuration + "\n" + "\tSwing Duration: " + this.defaultSwingDuration + "\n" + "\tSize: "
               + this.footstepDataList.size() + " Footsteps");
      }
   }

   public void setDefaultSwingDuration(double defaultSwingDuration)
   {
      this.defaultSwingDuration = defaultSwingDuration;
   }

   public void setDefaultTransferDuration(double defaultTransferDuration)
   {
      this.defaultTransferDuration = defaultTransferDuration;
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }

   public void setExecutionTiming(byte executionTiming)
   {
      this.executionTiming = executionTiming;
   }

   public byte getExecutionTiming()
   {
      return executionTiming;
   }

   public void setTrustHeightOfFootsteps(boolean trustHeight)
   {
      trustHeightOfFootsteps = trustHeight;
   }

   public void setAreFootstepsAdjustable(boolean areFootstepsAdjustable)
   {
      this.areFootstepsAdjustable = areFootstepsAdjustable;
   }

   public void setOffsetFootstepsWithExecutionError(boolean offsetFootstepsWithExecutionError)
   {
      this.offsetFootstepsWithExecutionError = offsetFootstepsWithExecutionError;
   }

   public boolean getOffsetFootstepsWithExecutionError()
   {
      return offsetFootstepsWithExecutionError;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataListMessage(this);
   }
}
