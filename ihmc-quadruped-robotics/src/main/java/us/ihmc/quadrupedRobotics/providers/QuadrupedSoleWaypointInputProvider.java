package us.ihmc.quadrupedRobotics.providers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedSoleWaypointPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedSoleWaypointInputProvider
{
   private final AtomicReference<QuadrupedSoleWaypointPacket> quadrupedSoleWaypointPacket;

   public QuadrupedSoleWaypointInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      quadrupedSoleWaypointPacket = new AtomicReference<>(new QuadrupedSoleWaypointPacket());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedSoleWaypointPacket.class, new PacketConsumer<QuadrupedSoleWaypointPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedSoleWaypointPacket packet)
            {
               quadrupedSoleWaypointPacket.set(packet);
            }
         });
      }
   }

   public QuadrantDependentList<QuadrupedSoleWaypointList> get()
   {
      return quadrupedSoleWaypointPacket.get().get();
   }
}
