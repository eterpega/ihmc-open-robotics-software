package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

@HighBandwidthPacket
public class FisheyePacket extends Packet<FisheyePacket>
{
   public VideoPacket videoPacket = HumanoidMessageTools.createVideoPacket();

   public FisheyePacket()
   {
   }

   @Override
   public void set(FisheyePacket other)
   {
      videoPacket.set(other.videoPacket);
   }

   @Override
   public boolean epsilonEquals(FisheyePacket other, double epsilon)
   {
      return videoPacket.epsilonEquals(other.videoPacket, epsilon);
   }
}
