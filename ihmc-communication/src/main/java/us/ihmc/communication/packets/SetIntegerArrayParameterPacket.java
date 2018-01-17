package us.ihmc.communication.packets;

public class SetIntegerArrayParameterPacket extends Packet<SetIntegerArrayParameterPacket>
{
   private final String parameterName;
   private final int[] parameterValue;

   // Empty constructor for serialization
   public SetIntegerArrayParameterPacket()
   {
      this(null, new int[] {});
   }

   public SetIntegerArrayParameterPacket(String parameterName, int[] parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public int[] getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetIntegerArrayParameterPacket other, double epsilon)
   {
      if (!parameterName.equals(other.parameterName) || parameterValue.length != other.parameterValue.length)
      {
         return false;
      }

      for (int i = 0; i < parameterValue.length; i++)
      {
         if (parameterValue[i] != other.parameterValue[i])
         {
            return false;
         }
      }

      return true;
   }
}