package us.ihmc.communication.packets;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Method;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.DisableOnDebug;
import org.junit.rules.Timeout;
import org.reflections.Reflections;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class MessageToolsTest
{
   @Rule
   public DisableOnDebug disableOnDebug = new DisableOnDebug(new Timeout(30, TimeUnit.SECONDS));

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testAllPacketHasAFactoryMethodWithNoArg()
   { // This test won't fail on Arrays or Lists
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc.communication");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithoutFactory = new TreeSet<>((o1, o2) -> o1.getSimpleName().compareTo(o2.getSimpleName()));

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         String expectedMethodName = "create" + packetType.getSimpleName();

         try
         {
            Method factoryMethod = MessageTools.class.getMethod(expectedMethodName);
            if (factoryMethod.getReturnType() != packetType)
               packetTypesWithoutFactory.add(packetType);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
            packetTypesWithoutFactory.add(packetType);
         }
      }

      if (verbose)
      {
         if (!packetTypesWithoutFactory.isEmpty())
         {
            PrintTools.error("List of packet without factory method:");
            packetTypesWithoutFactory.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Found packets without factory.", packetTypesWithoutFactory.isEmpty());
   }
}
