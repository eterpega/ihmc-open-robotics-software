package us.ihmc.javaFXToolkit.framework.data.dataProviders;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import us.ihmc.javaFXToolkit.framework.data.*;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentBufferedUIVariable;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;

/**
 *Provides currentState of
 *
 * Buffered UIDataProvider needs to implement the split data model but one should still be able to modify UIVariables that wrap YoVariables so as to propagate changes made from the GUI to the sim.
 * Buffered data (Data history is however been changed on a different thread - following the split-model design)
 *
 * Version 1 will use split Model design on all UIVariables, second will only use it for a registered list of UIVariables that need access to the buffer. Variables will register at runtime.
 *
 *
 * Created by amoucheboeuf on 12/16/15.
 */
public class BufferedUIDataProvider implements UIDataProvider
{

   private final UIDataProvider dataProvider;
   private final String[] sharedUIVariableNames;

   private final ConcurrentHashMap<String, ConcurrentBufferedUIVariable> bufferedVariablesMap;

   private final int capacity;

   /**
    * @param capacity maximum capacity of the data buffer before data is overwritten
    */
   public BufferedUIDataProvider(UIDataProvider dataProvider, String[] sharedUIVariableNames, int capacity)
   {
      this.sharedUIVariableNames = sharedUIVariableNames;
      this.bufferedVariablesMap = new ConcurrentHashMap<>(sharedUIVariableNames.length);
      this.dataProvider = dataProvider;
      this.capacity = capacity;

      for (String name : sharedUIVariableNames)
      {
         UIVariable uiVariable = dataProvider.getVariable(name);

         // Here make a simple copy of the name, current value and type of UIVariable so as to create independent BufferedUIVariables

         UIVariable mappedVariable = null;
         switch (uiVariable.getType())
         {
         case BOOLEAN:
            mappedVariable = new BooleanUIVariable(name, ((BooleanUIVariable) uiVariable).getValue());
            break;
         case INTEGER:
            mappedVariable = new IntegerUIVariable(name, ((IntegerUIVariable) uiVariable).getValue());
            break;
         case LONG:
            mappedVariable = new LongUIVariable(name, ((LongUIVariable) uiVariable).getValue());
            break;
         case DOUBLE:
            mappedVariable = new DoubleUIVariable(name, ((DoubleUIVariable) uiVariable).getValue());
            break;
         case ENUM:
            mappedVariable = new EnumUIVariable(name, ((EnumUIVariable) uiVariable).getValue());
            break;
         }

         bufferedVariablesMap.put(name, new ConcurrentBufferedUIVariable(mappedVariable, capacity));
      }

      this.dataProvider.getDataProviderStateVariable().addListener(simulationStateChangeListener);
   }

   /**
    *
    */
   private ChangeListener simulationStateChangeListener = (ObservableValue observable, Object oldValue, Object newValue) -> {
      if (oldValue == UIDataProviderState.STOPPED && newValue == UIDataProviderState.RUNNING)
      {
         clearAllBuffers();
      }
   };

   @Override public void initDataProvider()
   {
      // TODO needs to be removed
   }

   /**
    *
    */

   public void recordNewEntries(double[] newData)
   {
      int i = 0;
      for (String name : sharedUIVariableNames)
      {
         bufferedVariablesMap.get(name).storeValueAsDoubleInBuffer(newData[i++]); // Make sure this changes the UIVariable currentValue
      }

   }

   public void recordNewEntriesBlocks(double[][] newData)
   {
      int i = 0;
      for (String name : sharedUIVariableNames)
      {
         bufferedVariablesMap.get(name).storeValuesAsDoubleInBuffer(newData[i++]); // Make sure this changes the UIVariable currentValue
      }

   }


   private void clearAllBuffers()
   {
      synchronized (bufferedVariablesMap)
      {
         for (ConcurrentBufferedUIVariable bufferedUIVariable : bufferedVariablesMap.values())
         {
            bufferedUIVariable.clearBuffer();
         }
      }
   }

   public int getSize()
   {
      return capacity;
   }

   @Override public String getDataProviderNiceName()
   {
      return dataProvider.getDataProviderNiceName();
   }

   @Override public UIVariable getVariable(String name)
   {
      return bufferedVariablesMap.get(name);
   }

   @Override public ArrayList<UIVariable> getAllVariables()
   {
      return new ArrayList<UIVariable>(bufferedVariablesMap.values());
   }

   @Override public EnumUIVariable getDataProviderStateVariable()
   {
      return dataProvider.getDataProviderStateVariable();
   }

   @Override public LongUIVariable getIterationIndexVariable()
   {
      return dataProvider.getIterationIndexVariable();
   }

   @Override public UIVariable<Double> getTimeStampVariable()
   {
      return getVariable(dataProvider.getTimeStampVariable().getName());
   }

   @Override public UIVariable<Double> getDeltaTimeVariable()
   {
      return dataProvider.getDeltaTimeVariable();
   }

   @Override public void start()
   {
      dataProvider.start();
   }

   @Override public void stop()
   {
      dataProvider.stop();
   }

   @Override public void pause()
   {
      dataProvider.pause();
   }

   public static int getBufferIndexForIterationIndex()
   {

      return 0;
   }
}
