package us.ihmc.javaFXToolkit.examples;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.uiSplitModel.UITriggerTimer;

import java.util.concurrent.TimeUnit;

/**
 * Created by amoucheboeuf on 6/24/16.
 */
public class UITriggerTimerExample extends Application
{

   public static final int TIMER_PERIOD = 100;
   public static final TimeUnit timeUnitMs = TimeUnit.MILLISECONDS;

   private boolean isPaused = true;

   private long currentTimeMs = 0, lastTimeMs = 0;

   @Override public void start(Stage primaryStage) throws Exception
   {

      Button button = new Button("Start Timer");
      primaryStage.setScene(new Scene(button));

      UITriggerTimer uiTriggerTimer = new UITriggerTimer("ExampleTimer", TIMER_PERIOD, timeUnitMs);


      uiTriggerTimer.trigger.addListener(
            observable -> {
               currentTimeMs = System.currentTimeMillis();
               System.out.println("Trigger variable changed state after "+ (currentTimeMs - lastTimeMs) + " ms" );
               lastTimeMs = currentTimeMs;
            }
      );


      button.setOnAction(event -> {
         if(isPaused) // start timer
         {
            currentTimeMs = 0;
            lastTimeMs = System.currentTimeMillis();
            button.setText("Stop Timer");

            uiTriggerTimer.startTimer();

            System.out.println("button start");

         }
         else // stop timer
         {
            System.out.println("Button Stop");
            button.setText("Start Timer");
            uiTriggerTimer.stopTimer();
         }
         isPaused = !isPaused;
      });


      primaryStage.show();
   }


}
