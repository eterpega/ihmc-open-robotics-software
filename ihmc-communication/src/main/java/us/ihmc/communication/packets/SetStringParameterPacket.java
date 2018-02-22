package us.ihmc.communication.packets;

public class SetStringParameterPacket extends Packet<SetStringParameterPacket>
{
   public StringBuilder parameterName = new StringBuilder();
   public StringBuilder parameterValue = new StringBuilder();

   // Empty constructor for serialization
   public SetStringParameterPacket()
   {
   }

   @Override
   public void set(SetStringParameterPacket other)
   {
      parameterName.setLength(0);
      parameterName.append(other.parameterName);
      parameterValue.setLength(0);
      parameterValue.append(other.parameterValue);
      setPacketInformation(other);
   }

   public String getParameterNameAsString()
   {
      return parameterName.toString();
   }

   public String getParameterValueAsString()
   {
      return parameterValue.toString();
   }

   @Override
   public boolean epsilonEquals(SetStringParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && parameterValue.equals(other.parameterValue);
   }
}
