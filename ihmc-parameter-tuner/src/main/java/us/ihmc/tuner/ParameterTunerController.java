package us.ihmc.tuner;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;

import javafx.application.Platform;
import javafx.fxml.FXML;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.*;
import us.ihmc.communication.remote.IntegerPacket;
import us.ihmc.robotics.dataStructures.parameter.*;
import us.ihmc.tuner.tree.ParameterTree;

public class ParameterTunerController
{
   @FXML private ParameterTree paramTree;

   /**
    * Unlatches once the GUI elements have been initialized.
    */
   private final CountDownLatch initializedLatch = new CountDownLatch(1);

   public void initialize()
   {
      initializedLatch.countDown();
   }

   public void attachParameterCommunicator(final PacketCommunicator parameterSourceCommunicator)
   {
      try
      {
         // Wait for the GUI elements to be initialized.
         initializedLatch.await();

         // Listen for parameter lists, updating the internal registry when one arrives.
         parameterSourceCommunicator.attachListener(ParameterListPacket.class, packet ->
         {
            for (Parameter parameter : packet.getParameters())
            {
               if (ParameterRegistry.getInstance().getParameter(parameter.getPath()) != null)
               {
                  System.err.println("Registering packet that already exists in registry: " + parameter.getPath());

                  // TODO: Verify that this works and is the correct way to handle this.
                  ParameterRegistry.getInstance().getParameter(parameter.getPath()).tryLoad(parameter.dump());
                  continue;
               }

               // Register the packet in the internal registry.
               ParameterRegistry.getInstance().register(parameter);

               // When a parameter value changes, send a packet back to the parameter source reflecting the change.
               parameter.addChangeListener(modified ->
               {
                  final AtomicReference<Packet<?>> paramUpdatePacket = new AtomicReference<>();
                  modified.accept(new ParameterVisitor()
                  {
                     @Override
                     public void visitBoolean(BooleanParameter parameter)
                     {
                        paramUpdatePacket.set(new SetBooleanParameterPacket(modified.getPath(), parameter.get()));
                     }

                     @Override
                     public void visitDoubleArray(DoubleArrayParameter parameter)
                     {
                        paramUpdatePacket.set(new SetDoubleArrayParameterPacket(modified.getPath(), parameter.get()));
                     }

                     @Override
                     public void visitDouble(DoubleParameter parameter)
                     {
                        paramUpdatePacket.set(new SetDoubleParameterPacket(modified.getPath(), parameter.get()));
                     }

                     @Override
                     public void visitIntegerArray(IntegerArrayParameter parameter)
                     {
                        paramUpdatePacket.set(new SetIntegerArrayParameterPacket(modified.getPath(), parameter.get()));
                     }

                     @Override
                     public void visitInteger(IntegerParameter parameter)
                     {
                        paramUpdatePacket.set(new SetIntegerParameterPacket(modified.getPath(), parameter.get()));
                     }

                     @Override
                     public void visitString(StringParameter parameter)
                     {
                        // TODO: Support string parameters.
                     }
                  });

                  if (paramUpdatePacket.get() == null)
                     throw new IllegalArgumentException("Parameter of unknown type modified: " + modified);

                  parameterSourceCommunicator.send(paramUpdatePacket.get());
               });

               // Run on JavaFX thread
               Platform.runLater(() -> paramTree.setParameters(packet.getParameters()));
            }
         });

         // Now that we're listening for a parameter list, request one.
         parameterSourceCommunicator.send(new RequestParameterListPacket());
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException("Error waiting for GUI to be initialized: " + e);
      }
   }
}
