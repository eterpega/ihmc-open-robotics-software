package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class DrillDetectionPacket extends Packet<DrillDetectionPacket>
{
   public boolean isDrillOn;

   public DrillDetectionPacket()
   {
   }

   @Override
   public void set(DrillDetectionPacket other)
   {
      isDrillOn = other.isDrillOn;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(DrillDetectionPacket other, double epsilon)
   {
      return this.isDrillOn == other.isDrillOn;
   }
}
