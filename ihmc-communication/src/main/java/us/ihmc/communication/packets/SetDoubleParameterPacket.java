package us.ihmc.communication.packets;

import com.google.common.math.DoubleMath;

public class SetDoubleParameterPacket extends Packet<SetDoubleParameterPacket>
{
   public StringBuilder parameterName = new StringBuilder();
   public double parameterValue;

   // Empty constructor for serialization
   public SetDoubleParameterPacket()
   {
   }

   @Override
   public void set(SetDoubleParameterPacket other)
   {
      parameterName.setLength(0);
      parameterName.append(other.parameterName);
      parameterValue = other.parameterValue;
      setPacketInformation(other);
   }

   public String getParameterNameAsString()
   {
      return parameterName.toString();
   }

   public double getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetDoubleParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && DoubleMath.fuzzyEquals(parameterValue, other.parameterValue, 1e-12);
   }
}
