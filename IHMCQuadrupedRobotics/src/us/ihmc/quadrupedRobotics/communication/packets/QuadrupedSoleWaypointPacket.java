package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypoints;

public class QuadrupedSoleWaypointPacket extends Packet<QuadrupedSoleWaypointPacket>
{

   private QuadrupedSoleWaypoints quadrupedSoleWaypoints;

   public QuadrupedSoleWaypointPacket()
   {
      this.quadrupedSoleWaypoints = new QuadrupedSoleWaypoints();
   }

   public QuadrupedSoleWaypointPacket(QuadrupedSoleWaypoints quadrupedSoleWaypoints)
   {
      this.quadrupedSoleWaypoints = new QuadrupedSoleWaypoints(quadrupedSoleWaypoints);
   }

   @Override
   public boolean epsilonEquals(QuadrupedSoleWaypointPacket other, double epsilon)
   {
      return quadrupedSoleWaypoints.epsilonEquals(other.quadrupedSoleWaypoints, epsilon);
   }

   public QuadrupedSoleWaypoints get()
   {
      return quadrupedSoleWaypoints;
   }

   public void set(QuadrupedSoleWaypoints quadrupedSoleWaypoints)
   {
      this.quadrupedSoleWaypoints = quadrupedSoleWaypoints;
   }
}
