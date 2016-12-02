package us.ihmc.javaFXToolkit.framework.data.dataProviders;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import us.ihmc.javaFXToolkit.framework.data.DoubleUIVariable;
import us.ihmc.javaFXToolkit.framework.data.EnumUIVariable;
import us.ihmc.javaFXToolkit.framework.data.LongUIVariable;
import us.ihmc.javaFXToolkit.framework.data.UIVariable;

import java.util.ArrayList;

/**
 * Created by amoucheboeuf on 1/25/16.
 */
public class SampledUIDataProvider implements UIDataProvider
{

   private UIDataProvider uiDataProvider;
   private double sampleRateSeconds;
   private double lastTimestamp;
   private LongUIVariable sampledIterationIndex;
   private DoubleUIVariable deltaTime;

   public SampledUIDataProvider(UIDataProvider uiDataProvider, int sampleRateMs)
   {
      this.uiDataProvider = uiDataProvider;
      this.sampleRateSeconds = sampleRateMs / 1000.0;
      this.sampledIterationIndex = new LongUIVariable("SampledIterationIndex", uiDataProvider.getIterationIndexVariable().get());
      this.lastTimestamp = 0.0;
      deltaTime = new DoubleUIVariable("SampledUIDataProviderDT", 0.0);
      deltaTime.set(this.sampleRateSeconds);
   }

   @Override public String getDataProviderNiceName()
   {
      return uiDataProvider.getDataProviderNiceName();
   }

   @Override public void initDataProvider()
   {
      this.uiDataProvider.getTimeStampVariable().addListener(timeStampListener);
   }

   private ChangeListener timeStampListener = (ObservableValue observable, Object oldValue, Object newValue) -> {
      double currentTimestamp = (double) newValue;
      if (currentTimestamp < lastTimestamp)
      {
         lastTimestamp = currentTimestamp;
         sampledIterationIndex.set(0L);
      }
      else if (currentTimestamp - lastTimestamp >= sampleRateSeconds)
      {
         lastTimestamp = currentTimestamp;
         sampledIterationIndex.set(sampledIterationIndex.get() + 1);
      }
   };

   @Override public UIVariable getVariable(String name)
   {
      return uiDataProvider.getVariable(name);
   }

   @Override public ArrayList<UIVariable> getAllVariables()
   {
      return uiDataProvider.getAllVariables();
   }

   @Override public EnumUIVariable getDataProviderStateVariable()
   {
      return uiDataProvider.getDataProviderStateVariable();
   }

   @Override public LongUIVariable getIterationIndexVariable()
   {
      return sampledIterationIndex;
   }

   @Override public UIVariable<Double> getTimeStampVariable()
   {
      return uiDataProvider.getTimeStampVariable();
   }

   @Override public UIVariable<Double> getDeltaTimeVariable()
   {
      return deltaTime;
   }

   @Override public void start()
   {
      uiDataProvider.start();
   }

   @Override public void stop()
   {
      uiDataProvider.stop();
   }

   @Override public void pause()
   {
      uiDataProvider.pause();
   }
}
