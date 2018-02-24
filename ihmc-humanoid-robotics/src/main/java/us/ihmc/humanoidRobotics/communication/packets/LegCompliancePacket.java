package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Arrays;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This packet controls the stiffness of LegJoints. i.e., the maximum force a joint puts out when it
 * is (pushed) away from a desired position. This is useful to prevent robot from falling when leg
 * hit things unexpectedly. However, low stiffness (high compliance) can lead to poor joint tracking
 * in the presence of natural joint stiction. Finding a good balance is application specific. In our
 * hybrid velocity+force controller, most force comes from velocity control, therefore, only
 * parameter related to velocity control is exposed.
 */
public class LegCompliancePacket extends Packet<LegCompliancePacket>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   /**
    * maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the
    * maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control
    * making the joint very compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3
    * gives decent tracking for slow motion and yet still compliant. The numbers in the array
    * correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.
    */
   public float[] maxVelocityDeltas; //values in the order of AtlasJointId.getLegJoints()

   public byte robotSide;

   public LegCompliancePacket()
   {
   }

   @Override
   public void set(LegCompliancePacket other)
   {
      maxVelocityDeltas = Arrays.copyOf(other.maxVelocityDeltas, other.maxVelocityDeltas.length);
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(LegCompliancePacket other, double epsilon)
   {
      if (maxVelocityDeltas.length != other.maxVelocityDeltas.length)
         return false;
      for (int i = 0; i < maxVelocityDeltas.length; i++)
         if (!MathTools.epsilonEquals(maxVelocityDeltas[i], other.maxVelocityDeltas[i], epsilon))
            return false;
      return true;
   }

   @Override
   public String toString()
   {
      StringBuilder s = new StringBuilder();
      s.append("LegCompliancePacket: side " + RobotSide.fromByte(robotSide).name());

      s.append(" maxVelocityDeltas ");
      for (int i = 0; i < maxVelocityDeltas.length; i++)
      {
         s.append(" " + maxVelocityDeltas[i]);
      }
      return s.toString();
   }
}
