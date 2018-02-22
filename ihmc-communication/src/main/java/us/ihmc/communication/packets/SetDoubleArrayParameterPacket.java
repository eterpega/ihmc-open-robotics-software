package us.ihmc.communication.packets;

import java.util.Arrays;

import com.google.common.math.DoubleMath;

public class SetDoubleArrayParameterPacket extends Packet<SetDoubleArrayParameterPacket>
{

   public StringBuilder parameterName = new StringBuilder();
   public double[] parameterValue;

   // Empty constructor for serialization
   public SetDoubleArrayParameterPacket()
   {
   }

   @Override
   public void set(SetDoubleArrayParameterPacket other)
   {
      parameterName.setLength(0);
      parameterName.append(other.parameterName);
      parameterValue = Arrays.copyOf(other.parameterValue, other.parameterValue.length);
      setPacketInformation(other);
   }

   public String getParameterNameAsString()
   {
      return parameterName.toString();
   }

   public double[] getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetDoubleArrayParameterPacket other, double epsilon)
   {
      if (!parameterName.equals(other.parameterName) || parameterValue.length != other.parameterValue.length)
      {
         return false;
      }

      for (int i = 0; i < parameterValue.length; i++)
      {
         if (!DoubleMath.fuzzyEquals(parameterValue[i], other.parameterValue[i], 1e-12))
         {
            return false;
         }
      }

      return true;
   }
}
