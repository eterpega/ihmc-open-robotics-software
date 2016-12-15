package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */

public class RemoteBehaviorService extends BehaviorService
{
   public RemoteBehaviorService(String name, CommunicationBridgeInterface communicationBridge)
   {
      super(name, communicationBridge);
   }

   @Override public void run()
   {

   }

   @Override public void pause()
   {

   }

   @Override public void destroy()
   {

   }
}
