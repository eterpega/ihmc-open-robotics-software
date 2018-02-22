package us.ihmc.communication.packets;

public class SetBooleanParameterPacket extends Packet<SetBooleanParameterPacket>
{
   public StringBuilder parameterName = new StringBuilder();
   public boolean parameterValue;

   // Empty constructor for serialization
   public SetBooleanParameterPacket()
   {
   }

   @Override
   public void set(SetBooleanParameterPacket other)
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

   public boolean getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetBooleanParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && parameterValue == other.parameterValue;
   }
}
