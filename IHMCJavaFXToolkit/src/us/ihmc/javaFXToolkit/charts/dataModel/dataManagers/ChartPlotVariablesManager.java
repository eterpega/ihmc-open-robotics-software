package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import us.ihmc.javaFXToolkit.framework.data.dataStructures.DoubleBufferReader;

/**
 * Created by amoucheboeuf on 6/23/16.
 */
public interface ChartPlotVariablesManager
{

//   void setPlotVariableToAxisX(String name, DoubleBufferReader bufferedData);

   void addPlotVariableToAxisY(String name, DoubleBufferReader bufferedData);

   void removeVariableFromAxisY(String name);

   void removeVariableFromAxisY(DoubleBufferReader bufferedData);


   // Clear Data
}
