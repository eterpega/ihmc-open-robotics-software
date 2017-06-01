package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HatchLocationPacket;

public class SearchForHatchBehavior extends AbstractBehavior
{
   private RigidBodyTransform hatchTransformToWorld;
   private double hatchStepHeight;
   private double hatchOpeningHeight;
   private double hatchWidth;
   private double hatchThickness;
   
   private boolean receivedNewHatchLocation = false;
   private boolean resendHatchLocation = false;

   protected final ConcurrentListeningQueue<HatchLocationPacket> hatchLocationQueue = new ConcurrentListeningQueue<HatchLocationPacket>(10);

   public SearchForHatchBehavior(CommunicationBridge behaviorCommunicationBridge, boolean resendHatchLocation)
   {
      super("SearchForHatch", behaviorCommunicationBridge);
      attachNetworkListeningQueue(hatchLocationQueue, HatchLocationPacket.class);
      this.resendHatchLocation = resendHatchLocation;
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Searching for the Hatch");
      sendPacket(p1);
   }

   @Override
   public void doControl()
   {
      if (hatchLocationQueue.isNewPacketAvailable())
      {
         if(resendHatchLocation)
         {
            receivedHatchLocation(hatchLocationQueue.poll());
         }
         else
         {
            receivedHatchLocation(hatchLocationQueue.getLatestPacket());
         }
         PrintTools.debug(String.valueOf(hatchLocationQueue.isNewPacketAvailable()));
      }
   }

   @Override
   public boolean isDone()
   {
      return receivedNewHatchLocation;
   }

   @Override
   public void onBehaviorExited()
   {
      receivedNewHatchLocation = false;
   }

   public RigidBodyTransform getLocation()
   {
      return hatchTransformToWorld;
   }
   
   public double getStepHeight()
   {
      return hatchStepHeight;
   }
   
   public double getOpeningHeight()
   {
      return hatchOpeningHeight;
   }
   
   public double getWidth()
   {
      return hatchWidth;
   }
   
   public double getThickness()
   {
      return hatchThickness;
   }

   private void receivedHatchLocation(HatchLocationPacket hatchLocationPacket)
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Received Hatch Location From UI");
      sendPacket(p1);
      
      hatchTransformToWorld = hatchLocationPacket.getHatchTransformToWorld();
      hatchStepHeight = hatchLocationPacket.getHatchStepHeight();
      hatchOpeningHeight = hatchLocationPacket.getHatchOpeningHeight();
      hatchWidth =hatchLocationPacket.getHatchWidth();
      hatchThickness = hatchLocationPacket.getHatchThickness();

      receivedNewHatchLocation = true;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}
