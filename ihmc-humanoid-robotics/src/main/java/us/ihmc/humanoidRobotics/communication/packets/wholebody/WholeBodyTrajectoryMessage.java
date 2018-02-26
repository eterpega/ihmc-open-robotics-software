package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

@RosMessagePacket(documentation = "Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n"
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message."
      + " If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/whole_body_trajectory")
public class WholeBodyTrajectoryMessage extends Packet<WholeBodyTrajectoryMessage>
{
   @RosExportedField(documentation = "Trajectory for the left hand")
   public HandTrajectoryMessage leftHandTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right hand")
   public HandTrajectoryMessage rightHandTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the left arm joints")
   public ArmTrajectoryMessage leftArmTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right arm joints")
   public ArmTrajectoryMessage rightArmTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the chest")
   public ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the pelvis")
   public PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the left foot")
   public FootTrajectoryMessage leftFootTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right foot")
   public FootTrajectoryMessage rightFootTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the head")
   public HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage();

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public WholeBodyTrajectoryMessage()
   {
   }

   public WholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage other)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      leftHandTrajectoryMessage = new HandTrajectoryMessage(other.leftHandTrajectoryMessage);
      rightHandTrajectoryMessage = new HandTrajectoryMessage(other.rightHandTrajectoryMessage);
      leftArmTrajectoryMessage = new ArmTrajectoryMessage(other.leftArmTrajectoryMessage);
      rightArmTrajectoryMessage = new ArmTrajectoryMessage(other.rightArmTrajectoryMessage);
      chestTrajectoryMessage = new ChestTrajectoryMessage(other.chestTrajectoryMessage);
      pelvisTrajectoryMessage = new PelvisTrajectoryMessage(other.pelvisTrajectoryMessage);
      leftFootTrajectoryMessage = new FootTrajectoryMessage(other.leftFootTrajectoryMessage);
      rightFootTrajectoryMessage = new FootTrajectoryMessage(other.rightFootTrajectoryMessage);
      headTrajectoryMessage = new HeadTrajectoryMessage(other.headTrajectoryMessage);
   }

   @Override
   public void set(WholeBodyTrajectoryMessage other)
   {
      leftHandTrajectoryMessage.set(other.leftHandTrajectoryMessage);
      rightHandTrajectoryMessage.set(other.rightHandTrajectoryMessage);
      leftArmTrajectoryMessage.set(other.leftArmTrajectoryMessage);
      rightArmTrajectoryMessage.set(other.rightArmTrajectoryMessage);
      chestTrajectoryMessage.set(other.chestTrajectoryMessage);
      pelvisTrajectoryMessage.set(other.pelvisTrajectoryMessage);
      leftFootTrajectoryMessage.set(other.leftFootTrajectoryMessage);
      rightFootTrajectoryMessage.set(other.rightFootTrajectoryMessage);
      headTrajectoryMessage.set(other.headTrajectoryMessage);
      setPacketInformation(other);
   }

   public ChestTrajectoryMessage getChestTrajectoryMessage()
   {
      return chestTrajectoryMessage;
   }

   public PelvisTrajectoryMessage getPelvisTrajectoryMessage()
   {
      return pelvisTrajectoryMessage;
   }

   public HeadTrajectoryMessage getHeadTrajectoryMessage()
   {
      return headTrajectoryMessage;
   }

   public void setLeftHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      leftHandTrajectoryMessage.set(handTrajectoryMessage);
   }

   public void setRightHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      rightHandTrajectoryMessage.set(handTrajectoryMessage);
   }

   public void setLeftArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      leftArmTrajectoryMessage.set(armTrajectoryMessage);
   }

   public void setRightArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      rightArmTrajectoryMessage.set(armTrajectoryMessage);
   }

   public void setChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      this.chestTrajectoryMessage.set(chestTrajectoryMessage);
   }

   public void setPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      this.pelvisTrajectoryMessage.set(pelvisTrajectoryMessage);
   }

   public void setLeftFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      leftFootTrajectoryMessage.set(footTrajectoryMessage);
   }

   public void setRightFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      rightFootTrajectoryMessage.set(footTrajectoryMessage);
   }

   public void setHeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      this.headTrajectoryMessage.set(headTrajectoryMessage);
   }

   public HandTrajectoryMessage getLeftHandTrajectoryMessage()
   {
      return leftHandTrajectoryMessage;
   }

   public HandTrajectoryMessage getRightHandTrajectoryMessage()
   {
      return rightHandTrajectoryMessage;
   }

   public ArmTrajectoryMessage getLeftArmTrajectoryMessage()
   {
      return leftArmTrajectoryMessage;
   }

   public ArmTrajectoryMessage getRightArmTrajectoryMessage()
   {
      return rightArmTrajectoryMessage;
   }

   public FootTrajectoryMessage getLeftFootTrajectoryMessage()
   {
      return leftFootTrajectoryMessage;
   }

   public FootTrajectoryMessage getRightFootTrajectoryMessage()
   {
      return rightFootTrajectoryMessage;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      executionDelayTime = delayTime;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryMessage other, double epsilon)
   {
      if (!leftHandTrajectoryMessage.epsilonEquals(other.leftHandTrajectoryMessage, epsilon))
         return false;
      if (!rightHandTrajectoryMessage.epsilonEquals(other.rightHandTrajectoryMessage, epsilon))
         return false;
      if (!leftArmTrajectoryMessage.epsilonEquals(other.leftArmTrajectoryMessage, epsilon))
         return false;
      if (!rightArmTrajectoryMessage.epsilonEquals(other.rightArmTrajectoryMessage, epsilon))
         return false;
      if (!chestTrajectoryMessage.epsilonEquals(other.chestTrajectoryMessage, epsilon))
         return false;
      if (!pelvisTrajectoryMessage.epsilonEquals(other.pelvisTrajectoryMessage, epsilon))
         return false;
      if (!headTrajectoryMessage.epsilonEquals(other.headTrajectoryMessage, epsilon))
         return false;
      if (!leftFootTrajectoryMessage.epsilonEquals(other.leftFootTrajectoryMessage, epsilon))
         return false;
      if (!rightFootTrajectoryMessage.epsilonEquals(other.rightFootTrajectoryMessage, epsilon))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ":";
      string += "\n" + leftHandTrajectoryMessage.toString();
      string += "\n" + rightHandTrajectoryMessage.toString();
      string += "\n" + leftArmTrajectoryMessage.toString();
      string += "\n" + rightArmTrajectoryMessage.toString();
      string += "\n" + pelvisTrajectoryMessage.toString();
      string += "\n" + headTrajectoryMessage.toString();
      string += "\n" + leftFootTrajectoryMessage.toString();
      string += "\n" + rightFootTrajectoryMessage.toString();
      return string;
   }
}
