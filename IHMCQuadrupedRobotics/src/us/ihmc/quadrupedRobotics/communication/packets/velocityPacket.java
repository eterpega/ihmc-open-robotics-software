package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Vector3d;

public class velocityPacket extends Packet<velocityPacket>
{
   private final Vector3d velocity;

   public velocityPacket()
   {
      this.velocity = new Vector3d();
   }

   public velocityPacket(Vector3d velocity)
   {
      this.velocity = new Vector3d(velocity);
   }

   public velocityPacket(double vx, double vy, double vz)
   {
      this.velocity = new Vector3d(vx, vy, vz);
   }

   public void get(Vector3d velocity)
   {
      velocity.set(this.velocity);
   }

   public double getX()
   {
      return velocity.getX();
   }

   public double getY()
   {
      return velocity.getY();
   }

   public double getZ()
   {
      return velocity.getZ();
   }

   @Override public boolean epsilonEquals(velocityPacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.velocity, epsilon);
   }
}
