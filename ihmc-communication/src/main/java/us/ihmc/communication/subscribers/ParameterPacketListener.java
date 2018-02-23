package us.ihmc.communication.subscribers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ParameterListPacket;
import us.ihmc.communication.packets.RequestParameterListPacket;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.packets.SetDoubleArrayParameterPacket;
import us.ihmc.communication.packets.SetDoubleParameterPacket;
import us.ihmc.communication.packets.SetStringParameterPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.dataStructures.parameter.StringParameter;

public class ParameterPacketListener
{
   // TODO: Make this a packet communicator, not a global data producer?
   public ParameterPacketListener(final GlobalDataProducer communicator)
   {
      communicator.attachListener(RequestParameterListPacket.class, new PacketConsumer<RequestParameterListPacket>()
      {
         @Override
         public void receivedPacket(RequestParameterListPacket packet)
         {
            ParameterListPacket response = MessageTools.createParameterListPacket(ParameterRegistry.getInstance().getParameters());
            communicator.queueDataToSend(response);
         }
      });

      communicator.attachListener(SetBooleanParameterPacket.class, new PacketConsumer<SetBooleanParameterPacket>()
      {
         @Override
         public void receivedPacket(SetBooleanParameterPacket packet)
         {
            onSetBooleanPacket(packet);
         }
      });
      communicator.attachListener(SetDoubleArrayParameterPacket.class, new PacketConsumer<SetDoubleArrayParameterPacket>()
      {
         @Override
         public void receivedPacket(SetDoubleArrayParameterPacket packet)
         {
            onSetDoubleArrayPacket(packet);
         }
      });
      communicator.attachListener(SetDoubleParameterPacket.class, new PacketConsumer<SetDoubleParameterPacket>()
      {
         @Override
         public void receivedPacket(SetDoubleParameterPacket packet)
         {
            onSetDoublePacket(packet);
         }
      });
      communicator.attachListener(SetStringParameterPacket.class, new PacketConsumer<SetStringParameterPacket>()
      {
         @Override
         public void receivedPacket(SetStringParameterPacket packet)
         {
            onStringPacket(packet);
         }
      });
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
