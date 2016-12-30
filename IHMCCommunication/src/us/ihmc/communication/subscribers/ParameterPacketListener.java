package us.ihmc.communication.subscribers;

import us.ihmc.communication.packets.ParameterListPacket;
import us.ihmc.communication.packets.RequestParameterListPacket;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.packets.SetDoubleArrayParameterPacket;
import us.ihmc.communication.packets.SetDoubleParameterPacket;
import us.ihmc.communication.packets.SetIntegerArrayParameterPacket;
import us.ihmc.communication.packets.SetIntegerParameterPacket;
import us.ihmc.communication.packets.SetStringParameterPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.dataStructures.parameter.StringParameter;

public class ParameterPacketListener
{
   // TODO: Make this a packet communicator, not a global data producer?
   public ParameterPacketListener(final GlobalDataProducer communicator)
   {
      communicator.attachListener(RequestParameterListPacket.class, packet ->
      {
         ParameterListPacket response = new ParameterListPacket(ParameterRegistry.getInstance().getParameters());
         communicator.queueDataToSend(response);
      });

      communicator.attachListener(SetBooleanParameterPacket.class, this::onSetBooleanPacket);
      communicator.attachListener(SetDoubleArrayParameterPacket.class, this::onSetDoubleArrayPacket);
      communicator.attachListener(SetDoubleParameterPacket.class, this::onSetDoublePacket);
      communicator.attachListener(SetIntegerArrayParameterPacket.class, this::onSetIntegerArrayPacket);
      communicator.attachListener(SetIntegerParameterPacket.class, this::onSetIntegerPacket);
      communicator.attachListener(SetStringParameterPacket.class, this::onStringPacket);
   }

   public void onSetBooleanPacket(SetBooleanParameterPacket packet)
   {
      BooleanParameter parameter = lookup(packet.getParameterName(), BooleanParameter.class);
      parameter.set(packet.getParameterValue());
   }

   public void onSetDoubleArrayPacket(SetDoubleArrayParameterPacket packet)
   {
      DoubleArrayParameter parameter = lookup(packet.getParameterName(), DoubleArrayParameter.class);
      parameter.set(packet.getParameterValue());
   }

   public void onSetDoublePacket(SetDoubleParameterPacket packet)
   {
      DoubleParameter parameter = lookup(packet.getParameterName(), DoubleParameter.class);
      parameter.set(packet.getParameterValue());
   }

   public void onSetIntegerArrayPacket(SetIntegerArrayParameterPacket packet)
   {
      IntegerArrayParameter parameter = lookup(packet.getParameterName(), IntegerArrayParameter.class);
      parameter.set(packet.getParameterValue());
   }

   public void onSetIntegerPacket(SetIntegerParameterPacket packet)
   {
      IntegerParameter parameter = lookup(packet.getParameterName(), IntegerParameter.class);
      parameter.set(packet.getParameterValue());
   }

   public void onStringPacket(SetStringParameterPacket packet)
   {
      StringParameter parameter = lookup(packet.getParameterName(), StringParameter.class);
      parameter.set(packet.getParameterValue());
   }

   @SuppressWarnings("unchecked")
   private <T extends Parameter> T lookup(String name, Class<T> type)
   {
      Parameter parameter = ParameterRegistry.getInstance().getParameter(name);

      if (parameter == null)
      {
         System.err.println("Packet tried to set nonexistent parameter: " + name);
      }
      else if (!(type.isInstance(parameter)))
      {
         System.err.println(
               "Packet tried setting parameter of wrong type (expected: " + type.getSimpleName() + ", got: " + parameter.getClass().getSimpleName() + "): "
                     + name);
      }

      return (T) parameter;
   }
}
