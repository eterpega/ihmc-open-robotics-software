package us.ihmc.javaFXToolkit.uiSplitModel;

import us.ihmc.javaFXToolkit.framework.data.BooleanUIVariable;
import us.ihmc.javaFXToolkit.framework.threadTools.DaemonThreadFactory;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Created by amoucheboeuf on 1/11/16.
 */
public class UITriggerTimer implements Runnable
{
   private static final DaemonThreadFactory daemonThreadFactory = new DaemonThreadFactory();
   private ExecutorService executorService;

   public final int triggerPeriod;

   public final TimeUnit timeUnit;
   public final BooleanUIVariable trigger;

   private boolean timerThreadInitialized = false;
   private boolean timerPaused = true;

   public UITriggerTimer(String triggerName, int triggerPeriod, TimeUnit timeUnit)
   {
      this.triggerPeriod = triggerPeriod;
      this.timeUnit = timeUnit;
      this.trigger = new BooleanUIVariable(triggerName, false);
   }

   public UITriggerTimer(BooleanUIVariable customTrigger, int triggerPeriod, TimeUnit timeUnit)
   {
      if (customTrigger == null)
         throw new NullPointerException();

      this.triggerPeriod = triggerPeriod;

      this.trigger = customTrigger;

      this.timeUnit = timeUnit;
   }

   public synchronized void startTimer()
   {
      if (timerThreadInitialized == false)
      {
         // Only entered once
         executorService = Executors.newSingleThreadExecutor(daemonThreadFactory);
         executorService.submit(this);

         timerThreadInitialized = true;
         timerPaused = false;
         return;
      }

      synchronized (this)
      {
         this.notify();
      }
   }

   public synchronized void stopTimer()
   {
      timerPaused = true;
   }

   @Override public void run()
   {
      long beforeTime, timeDifference, sleep = triggerPeriod;

      while (true)
      {

         if (!timerPaused)
         {
//            beforeTime = System.nanoTime();

            trigger.set(!trigger.get());

//            timeDifference = System.nanoTime() - beforeTime;
//
//            sleep = triggerPeriod - timeDifference;

//            if (sleep < 0)
//               continue;

            try
            {
               Thread.sleep(sleep);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

         }
         else
         {
            synchronized (this)
            {
               try
               {
                  this.wait();
                  timerPaused = false;
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }
         }
      }
   }

}
