package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Point3d;

public class positionPacket extends Packet<positionPacket>
{
   private Point3d position;

   public positionPacket()
   {
      this.position = new Point3d();
   }

   public positionPacket(Point3d position)
   {
      this.position = new Point3d(position);
   }

   public positionPacket(double x, double y, double z)
   {
      this.position = new Point3d(x, y, z);
   }

   public void get(Point3d position)
   {
      position.set(this.position);
   }

   public double getX()
   {
      return this.position.getX();
   }

   public double getY()
   {
      return this.position.getY();
   }

   public double getZ()
   {
      return this.position.getZ();
   }

   @Override public boolean epsilonEquals(positionPacket other, double epsilon)
   {
      return this.position.epsilonEquals(other.position, epsilon);
   }
}
