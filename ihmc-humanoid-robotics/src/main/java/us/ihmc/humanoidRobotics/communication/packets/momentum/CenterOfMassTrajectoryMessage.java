package us.ihmc.humanoidRobotics.communication.packets.momentum;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;

public class CenterOfMassTrajectoryMessage extends Packet<CenterOfMassTrajectoryMessage>
{
   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and
    * velocity at a given time.
    */
   public EuclideanTrajectoryMessage euclideanTrajectory = new EuclideanTrajectoryMessage();

   public CenterOfMassTrajectoryMessage()
   {
   }

   @Override
   public void set(CenterOfMassTrajectoryMessage other)
   {
      euclideanTrajectory.set(other.euclideanTrajectory);
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (euclideanTrajectory != null)
         euclideanTrajectory.setUniqueId(uniqueId);
   }

   public EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   @Override
   public boolean epsilonEquals(CenterOfMassTrajectoryMessage other, double epsilon)
   {
      if (!euclideanTrajectory.epsilonEquals(other.euclideanTrajectory, epsilon))
         return false;

      return true;
   }
}
