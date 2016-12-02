package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

/**
 * Created by amoucheboeuf on 6/16/16.
 */
public interface DataSeriesManagerChangeListener
{


   void xMinValueChanged(double oldValue, double newValue);
   void xMaxValueChanged(double oldValue, double newValue);
   void yMinValueChanged(double oldValue, double newValue);
   void yMaxValueChanged(double oldValue, double newValue);




}
