package us.ihmc.javaFXToolkit.framework.data.dataProviders;

import us.ihmc.javaFXToolkit.framework.data.EnumUIVariable;
import us.ihmc.javaFXToolkit.framework.data.LongUIVariable;
import us.ihmc.javaFXToolkit.framework.data.UIVariable;

import java.util.ArrayList;

/**
 * Created by amoucheboeuf on 11/16/15.
 */
public interface UIDataProvider
{

   public void initDataProvider(); // TODO Probably get rid of this method and initialize in constructor

   public String getDataProviderNiceName();

   public UIVariable getVariable(String name) throws IllegalArgumentException; // TODO force throw exception is name not correct

   public ArrayList<UIVariable> getAllVariables();

   /**
    * This will return the current state of the DataProvider examples of 
    * which are: Stopped, Running, and Paused
    *
    * @return
    */
   public EnumUIVariable getDataProviderStateVariable();

   /**
    * This will return a LongUIVariable which will have it's value incremented
    * every time a set of data is changed.  For instance if the data provider
    * has a simulation loop, then this value should be incremented at the end
    * of the loop after all the data has changed. Its value is reset after each time the simulation is stopped and started again
    *
    * @return
    */
   public LongUIVariable getIterationIndexVariable();

   public UIVariable<Double> getTimeStampVariable();

   public UIVariable<Double> getDeltaTimeVariable();

   /**
    * If this data provider has the ability to start and stop the flow of data
    * then this method will start the flow.
    */
   public void start();

   /**
    * If this data provider has the ability to start and stop the flow of data
    * then this method will stop the flow.
    */
   public void stop();

   /**
    * If this data provider has the ability to pause the flow of data
    * then this method will pause the flow.
    */
   public void pause();

}
