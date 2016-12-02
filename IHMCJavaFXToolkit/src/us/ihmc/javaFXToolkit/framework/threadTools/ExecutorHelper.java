package us.ihmc.javaFXToolkit.framework.threadTools;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class ExecutorHelper
{
   public static final DaemonThreadFactory daemonThreadFactory = new DaemonThreadFactory();

   public static ScheduledExecutorService scheduleAtFixedRateWithDaemonThreadFactory(Runnable command, long initialDelay, long period, TimeUnit unit)
   {
      ScheduledExecutorService scheduledExecutor = Executors.newSingleThreadScheduledExecutor(daemonThreadFactory);
      scheduledExecutor.scheduleAtFixedRate (command, initialDelay, period, unit);
      return scheduledExecutor;
   }
}
