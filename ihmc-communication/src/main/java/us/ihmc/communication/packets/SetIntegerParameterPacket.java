package us.ihmc.communication.packets;

public class SetIntegerParameterPacket extends Packet<SetIntegerParameterPacket>
{
   private final String parameterName;
   private final int parameterValue;

   // Empty constructor for serialization
   public SetIntegerParameterPacket()
   {
      this(null, 0);
   }

   public SetIntegerParameterPacket(String parameterName, int parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public int getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetIntegerParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && parameterValue == other.parameterValue;
   }
}
