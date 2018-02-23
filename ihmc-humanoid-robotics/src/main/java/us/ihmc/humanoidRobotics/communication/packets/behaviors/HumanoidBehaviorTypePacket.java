package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket>
{
   public HumanoidBehaviorType behaviorType;

   // empty constructor for deserialization
   public HumanoidBehaviorTypePacket()
   {
   }

   public HumanoidBehaviorTypePacket(HumanoidBehaviorType behaviorType)
   {
      this.behaviorType = behaviorType;
   }

   public HumanoidBehaviorType getBehaviorType()
   {
      return behaviorType;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      return this.getBehaviorType().equals(other.getBehaviorType());
   }
}
