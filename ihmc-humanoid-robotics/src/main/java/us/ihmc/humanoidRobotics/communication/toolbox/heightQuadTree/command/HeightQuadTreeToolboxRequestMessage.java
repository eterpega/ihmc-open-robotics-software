package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class HeightQuadTreeToolboxRequestMessage extends Packet<HeightQuadTreeToolboxRequestMessage>
{
   public boolean requestClearQuadTree;
   public boolean requestQuadTreeUpdate;

   public HeightQuadTreeToolboxRequestMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public static HeightQuadTreeToolboxRequestMessage clearRequest(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage clearMessage = new HeightQuadTreeToolboxRequestMessage();
      clearMessage.setDestination(destination);
      clearMessage.requestClearQuadTree = true;
      clearMessage.requestQuadTreeUpdate = false;
      return clearMessage;
   }

   public static HeightQuadTreeToolboxRequestMessage requestQuadTreeUpdate(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage requestMessage = new HeightQuadTreeToolboxRequestMessage();
      requestMessage.setDestination(destination);
      requestMessage.requestClearQuadTree = false;
      requestMessage.requestQuadTreeUpdate = true;
      return requestMessage;
   }

   @Override
   public void set(HeightQuadTreeToolboxRequestMessage other)
   {
      requestClearQuadTree = other.requestClearQuadTree;
      requestQuadTreeUpdate = other.requestQuadTreeUpdate;
      setPacketInformation(other);
   }

   public boolean isClearQuadTreeRequested()
   {
      return requestClearQuadTree;
   }

   public boolean isQuadTreeUpdateRequested()
   {
      return requestQuadTreeUpdate;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeToolboxRequestMessage other, double epsilon)
   {
      if (requestClearQuadTree != other.requestClearQuadTree)
         return false;
      if (requestQuadTreeUpdate != other.requestQuadTreeUpdate)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": clear request = " + requestClearQuadTree + ", quadTree request = " + requestQuadTreeUpdate;
   }
}
