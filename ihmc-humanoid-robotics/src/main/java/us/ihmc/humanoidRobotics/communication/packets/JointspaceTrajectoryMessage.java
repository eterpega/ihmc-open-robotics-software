package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.idl.TempPreallocatedList;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/jointspace_trajectory")
public final class JointspaceTrajectoryMessage extends Packet<JointspaceTrajectoryMessage>
{
   @RosExportedField(documentation = "List of points in the trajectory.")
   public TempPreallocatedList<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = new TempPreallocatedList<>(OneDoFJointTrajectoryMessage.class,
                                                                                                                  OneDoFJointTrajectoryMessage::new, 10);
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = MessageTools.createQueueableMessage();

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public JointspaceTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * 
    * @param other message to clone.
    */
   public JointspaceTrajectoryMessage(JointspaceTrajectoryMessage other)
   {
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
      queueingProperties.set(other.queueingProperties);
      MessageTools.copyData(other.jointTrajectoryMessages, jointTrajectoryMessages);
   }

   @Override
   public void set(JointspaceTrajectoryMessage other)
   {
      queueingProperties.set(other.queueingProperties);
      MessageTools.copyData(other.jointTrajectoryMessages, jointTrajectoryMessages);
      setPacketInformation(other);
   }

   public TempPreallocatedList<OneDoFJointTrajectoryMessage> getJointTrajectoryMessages()
   {
      return jointTrajectoryMessages;
   }

   public void setQueueingProperties(QueueableMessage queueingProperties)
   {
      this.queueingProperties = queueingProperties;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateJointspaceTrajectoryMessage(this);
   }

   @Override
   public boolean epsilonEquals(JointspaceTrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
         return false;

      if (jointTrajectoryMessages.size() != other.jointTrajectoryMessages.size())
      {
         return false;
      }

      for (int i = 0; i < jointTrajectoryMessages.size(); i++)
      {
         if (!jointTrajectoryMessages.get(i).epsilonEquals(other.jointTrajectoryMessages.get(i), epsilon))
         {
            return false;
         }
      }

      return true;
   }
}
