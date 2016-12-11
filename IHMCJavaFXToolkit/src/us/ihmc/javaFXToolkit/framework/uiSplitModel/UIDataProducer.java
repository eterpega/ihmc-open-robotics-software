package us.ihmc.javaFXToolkit.framework.uiSplitModel;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import us.ihmc.javaFXToolkit.framework.data.UIVariable;
import us.ihmc.javaFXToolkit.framework.data.dataProviders.UIDataProvider;


import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Created by amoucheboeuf on 6/23/16.
 */
public class UIDataProducer
{

   private final UIDataProvider dataProvider;
   private final ConcurrentLinkedQueue<double[]> queue;
   private final String[] sharedUIVariableNames;

   /**
    * Production of data is bound to the change of state to
    *
    * @param dataProvider
    * @param sharedUIVariableNames
    * @param queue
    */
   public UIDataProducer(UIDataProvider dataProvider, String[] sharedUIVariableNames, ConcurrentLinkedQueue<double[]> queue)
   {
      this.dataProvider = dataProvider;
      this.sharedUIVariableNames = sharedUIVariableNames;
      this.queue = queue;

      this.dataProvider.getIterationIndexVariable().addListener(triggerListener);     // this variable acts as a trigger to record the state TODO Enforce that
   }

   private ChangeListener triggerListener = (ObservableValue observable, Object oldValue, Object newValue) -> produceNewDataSet();


   private void produceNewDataSet() // Happens at each dataProvider iteration
   {

      double[] variableStateAtIteration = new double[sharedUIVariableNames.length];

      int i = 0;
      for(String name : sharedUIVariableNames)
      {
         UIVariable uiVariable = dataProvider.getVariable(name);
         variableStateAtIteration[i++] = uiVariable.getValueAsDouble();
      }

      this.queue.offer(variableStateAtIteration);
   }
}
