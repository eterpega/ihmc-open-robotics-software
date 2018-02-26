package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.communication.packets.Packet;

public class QuadrupedForceControllerStatePacket extends Packet<QuadrupedForceControllerStatePacket>
{
   private QuadrupedForceControllerEnum state;

   public QuadrupedForceControllerStatePacket()
   {
      this.state = null;
   }

   public QuadrupedForceControllerStatePacket(QuadrupedForceControllerEnum state)
   {
      this.state = state;
   }

   public QuadrupedForceControllerEnum get()
   {
      return state;
   }

   public void set(QuadrupedForceControllerEnum state){ this.state = state;}
   @Override public boolean epsilonEquals(QuadrupedForceControllerStatePacket other, double epsilon)
   {
      return (this.state == other.state);
   }
}
