package us.ihmc.robotbuilder.util;

import javafx.application.Platform;
import javafx.embed.swing.JFXPanel;
import javaslang.concurrent.Future;
import javaslang.control.Option;
import org.junit.Test;

import javax.swing.*;

import static org.junit.Assert.*;

public class UtilTest
{
   @Test
   public void testRunLaterInUIExecutesInJavaFXThread() throws Exception
   {
      SwingUtilities.invokeAndWait(() -> {
         new JFXPanel(); // init JavaFX

         Future<Boolean> future = Util.runLaterInUI(Platform::isFxApplicationThread);
         long startTime = System.currentTimeMillis();
         while (System.currentTimeMillis() - startTime <= 100)
         {
            if (future.isCompleted())
            {
               assertTrue(future.get());
               return;
            }
         }
         fail("Failed waiting for UI thread");
      });
   }

   @Test
   public void testRunLaterInUIHandlesExceptionsProperly() throws Exception
   {
      SwingUtilities.invokeAndWait(() -> {
         new JFXPanel(); // init JavaFX

         Future<Boolean> future = Util.runLaterInUI(() -> {throw new Exception("Should catch this"); });
         long startTime = System.currentTimeMillis();
         while (System.currentTimeMillis() - startTime <= 100)
         {
            if (future.isCompleted())
            {
               assertTrue(future.isFailure());
               assertEquals(future.getCause().map(Throwable::getMessage), Option.of("Should catch this"));
               return;
            }
         }
         fail("Failed waiting for UI thread");
      });
   }
}
