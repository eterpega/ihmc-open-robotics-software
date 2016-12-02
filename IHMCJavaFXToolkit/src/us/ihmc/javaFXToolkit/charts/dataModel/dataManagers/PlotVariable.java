package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import us.ihmc.javaFXToolkit.framework.data.dataStructures.DoubleBufferReader;
import us.ihmc.robotics.MathTools;

/**
 * Created by amoucheboeuf on 6/28/16.
 */
class PlotVariable
{
   protected final String name;
   protected final DoubleBufferReader dataHolder;

   // Indices Holders
   private int leftPlotIndex;
   private int rightPlotIndex;

   protected PlotVariable(String name, DoubleBufferReader dataHolder)
   {
      this.name = name;
      this.dataHolder = dataHolder;
   }

   public void setlastPlotDataSet(int leftIndex, int rightIndex)
   {
      MathTools.checkIfInRange(leftIndex, 0, dataHolder.getCapacity());
      MathTools.checkIfInRange(rightIndex, 0, dataHolder.getCapacity());
      MathTools.checkIfLessOrEqual(leftIndex, rightIndex);

      this.leftPlotIndex = leftIndex;
      this.rightPlotIndex = rightIndex;
   }


   public int getLeftPlotIndex()
   {
      return leftPlotIndex;
   }

   public int getRightPlotIndex()
   {
      return rightPlotIndex;
   }


}
