package us.ihmc.communication.packets;

import com.google.common.math.DoubleMath;

import gnu.trove.list.array.TDoubleArrayList;

public class SetDoubleArrayParameterPacket extends Packet<SetDoubleArrayParameterPacket>
{

   public StringBuilder parameterName = new StringBuilder();
   public TDoubleArrayList parameterValue = new TDoubleArrayList();

   // Empty constructor for serialization
   public SetDoubleArrayParameterPacket()
   {
   }

   @Override
   public void set(SetDoubleArrayParameterPacket other)
   {
      parameterName.setLength(0);
      parameterName.append(other.parameterName);
      MessageTools.copyData(other.parameterValue, parameterValue);
      setPacketInformation(other);
   }

   public String getParameterNameAsString()
   {
      return parameterName.toString();
   }

   public TDoubleArrayList getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetDoubleArrayParameterPacket other, double epsilon)
   {
      if (!parameterName.equals(other.parameterName) || parameterValue.size() != other.parameterValue.size())
      {
         return false;
      }

      for (int i = 0; i < parameterValue.size(); i++)
      {
         if (!DoubleMath.fuzzyEquals(parameterValue.get(i), other.parameterValue.get(i), 1e-12))
         {
            return false;
         }
      }

      return true;
   }
}
