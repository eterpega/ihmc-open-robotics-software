package us.ihmc.communication.packets;

public class ToolboxStateMessage extends Packet<ToolboxStateMessage>
{
   public static final byte WAKE_UP = 0;
   public static final byte REINITIALIZE = 1;
   public static final byte SLEEP = 2;

   public byte requestedToolboxState;

   public ToolboxStateMessage()
   {
   }

   @Override
   public void set(ToolboxStateMessage other)
   {
      requestedToolboxState = other.requestedToolboxState;
      set(other);
   }

   public void setRequestedToolboxState(byte requestedState)
   {
      this.requestedToolboxState = requestedState;
   }

   public byte getRequestedToolboxState()
   {
      return requestedToolboxState;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      return requestedToolboxState == other.requestedToolboxState;
   }
}
