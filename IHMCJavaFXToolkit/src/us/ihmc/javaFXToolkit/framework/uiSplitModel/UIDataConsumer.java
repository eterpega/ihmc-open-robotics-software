package us.ihmc.javaFXToolkit.framework.uiSplitModel;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import us.ihmc.javaFXToolkit.framework.data.UIVariable;
import us.ihmc.javaFXToolkit.framework.data.dataProviders.BufferedUIDataProvider;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Created by amoucheboeuf on 6/23/16.
 */
public class UIDataConsumer
{
   private final BufferedUIDataProvider bufferedUIDataProvider;
   private final ConcurrentLinkedQueue<double[]> queue;
   private final String[] sharedUIVariableNames; // TODO See if I an replace that with an arraylist or map at one point so as to add remove UIVariables
   private final UIVariable triggerVariable;
   /**
    *
    * @param dataProvider
    * @param sharedUIVariableNames
    * @param queue
    * @param triggerVariable Change of state in trigger variable activate consumption of data in queue
    */
   public UIDataConsumer(BufferedUIDataProvider dataProvider, String[] sharedUIVariableNames, ConcurrentLinkedQueue<double[]> queue, UIVariable triggerVariable)
   {
      this.bufferedUIDataProvider = dataProvider;
      this.sharedUIVariableNames = sharedUIVariableNames;
      this.queue = queue;
      this.triggerVariable = triggerVariable;
      this.triggerVariable.addListener(triggerListener);
   }

   private ChangeListener triggerListener = (ObservableValue observable, Object oldValue, Object newValue) -> commitCurrentVariableValuesForRecording();

   private void commitCurrentVariableValuesForRecording() // Happens when consumer thread polls data
   {
      double[] variablesState;

      while((variablesState = queue.poll()) != null)  // Now polling one value at a time and adding to the buffers, may be a bottleneck
      {
         bufferedUIDataProvider.recordNewEntries(variablesState);
      }
   }

}
