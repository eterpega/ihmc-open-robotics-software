package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import javafx.scene.chart.XYChart;
import us.ihmc.robotics.MathTools;

import java.util.Arrays;

/**
 * Created by amoucheboeuf on 6/28/16.
 */
public class DataForDisplay
{

   public final XYChart.Series targetSeries;
   public final double[] xValues;
   public final double[] yValues;

   public final double xMin, xMax, yMin, yMax;


   public DataForDisplay(XYChart.Series targetSeries, double[] xValues, double[] yValues)
   {
      if(xValues.length != yValues.length )
      {
         throw new IllegalArgumentException(" xValues[] and yValues[] number of entries do not match");
      }
      else if(xValues.length == 0)
      {
         throw new IllegalArgumentException("You are trying to add an empty array of values");
      }

      this.targetSeries = targetSeries;
      this.xValues = xValues;
      this.yValues = yValues;

      // Determine local min and max

      double xMin = Double.MAX_VALUE;
      double xMax = Double.MIN_VALUE;
      double yMin = Double.MAX_VALUE;
      double yMax = Double.MIN_VALUE;

      for (int i=0; i< xValues.length; i++)
      {
         xMin = Math.min(xMin, xValues[i]);
         xMax = Math.max(xMax, xValues[i]);
         yMin = Math.min(yMin, yValues[i]);
         yMax = Math.max(yMax, yValues[i]);
      }

      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
   }

   public String toString()
   {
      StringBuffer stringBuffer = new StringBuffer();
      stringBuffer.append("\n");
      stringBuffer.append("Target Series Name = ");
      stringBuffer.append(targetSeries.getName());
      stringBuffer.append("\n");

      stringBuffer.append("Target Series Data = ");
      stringBuffer.append(targetSeries.getData());
      stringBuffer.append("\n");



      stringBuffer.append("X Data = ");
      stringBuffer.append(Arrays.toString(xValues));
      stringBuffer.append("\n");

      stringBuffer.append("Y Data = ");
      stringBuffer.append(Arrays.toString(yValues));
      stringBuffer.append("\n");


      return stringBuffer.toString();
   }





}
