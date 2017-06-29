package us.ihmc.simulationToolkit;

import us.ihmc.humanoidRobotics.communication.packets.SCSListenerPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.simulationconstructionset.PlaybackListener;


public class SCSPlaybackListener implements PlaybackListener
{
   public HumanoidGlobalDataProducer networkServer;

   public SCSPlaybackListener(HumanoidGlobalDataProducer dataProducer)
   {
      this.networkServer = dataProducer;
   }

   @Override public void play(double realTimeRate)
   {
   }

   @Override public void stop()
   {
      if (networkServer != null)
         networkServer.queueDataToSend(new SCSListenerPacket());
   }

   @Override public void notifyOfIndexChange(int newIndex, double newTime)
   {
   }

   @Override public void notifyOfManualEndChange(int inPoint, int outPoint)
   {

   }
}
