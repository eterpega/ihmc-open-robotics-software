package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.chart.LineChart;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.DataProgressPanManager;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.DoubleBufferReader;

import java.util.concurrent.ConcurrentHashMap;

/**
 * Assumes that the ReadableBufferedData is contained in a circularBuffer
 *    Iteration number serves as a timestamp for all plot variables on Y and sets xAxis values to a look up table between iteration number and X plot variable value at this index
 *    Use plot variable buffer fill count,xAxisTimeStamp buffer fill count and iteration number to synchronize individual Y variable buffer values to X variable values for the same iteration count position
 *
 *
 *    Add/Remove data series from ReadableBufferedData variables
 *
 *    Only assumption made for time series: data are sorted in ascending order
 *
 *    Created by amoucheboeuf on 6/14/16.
 */
public class DataSeriesManager implements ChartPlotVariablesManager
{

   private DataSeriesManagerChangeListener dataSeriesManagerChangeListener;

   /**
    * Readable properties that the TimeSeries Exposes for other components to keep track of the current state of the chart
    */
   private final SimpleIntegerProperty lastIterationIndex = new SimpleIntegerProperty();

   private final DataProgressPanManager dataProgressPanManager;
   private final DynamicChart dynamicChart;

   // Variable  on x axis
   private TimeDataPlotVariable xAxisTimeStamp;

   private final ConcurrentHashMap<String, PlotVariable> plotVariableConcurrentHashMap;

   // Variables on y axis, may want to set a limit to the number of displayed variables per chart
   //   private ObservableMap<PlotVariable, XYChart.Series> slidingDataSeries;
   // TODO make it a concurrent map so when plot variables are added on the fly , the list is still able to be rendered

   //if no iteration variable is provided, one will rely on the oldest timestamp found as first iteration
   public DataSeriesManager(LineChart lineChart, String xAxisName, DoubleBufferReader xAxisDataBuffer)
   {
      this.dynamicChart = new DynamicChart(lineChart, xAxisDataBuffer.getCapacity());
      this.dataProgressPanManager = new DataProgressPanManager(dynamicChart);
      this.setPlotVariableToAxisX(xAxisName, xAxisDataBuffer);
      this.plotVariableConcurrentHashMap = new ConcurrentHashMap<>();
   }

   /**
    *  ADD / REMOVE PLOT VARIABLES
    *
    *  Set plot variable on X
    *  Add plot variables on Y
    *
    *  remove plot variables on Y
    *  Clear Data on Y
    */

   private void setPlotVariableToAxisX(String name, DoubleBufferReader bufferedData)
   {
      if (bufferedData.getCapacity() == 0)
      {
         throw new IllegalArgumentException("Try to set a ReadableBufferedData to chart with an empty capacity");
      }

      this.xAxisTimeStamp = new TimeDataPlotVariable(name, bufferedData);
   }

   @Override public void addPlotVariableToAxisY(String name, DoubleBufferReader bufferedData)
   {
      if (bufferedData.getCapacity() != dynamicChart.getChartBufferCapacity())
      {
         throw new IllegalArgumentException(
               "ReadableBufferedData buffer should have the same capacity as this chart buffer: " + dynamicChart.getChartBufferCapacity());
      }

      PlotVariable plotVariable = new PlotVariable(name, bufferedData);
      if (!plotVariableConcurrentHashMap.contains(name))
      {
         plotVariableConcurrentHashMap.put(name, plotVariable);
         dynamicChart.addDataSeries(name);
      }
   }

   @Override public void removeVariableFromAxisY(String name)
   {
      plotVariableConcurrentHashMap.remove(name);
      dynamicChart.removeDataSeries(name);
   }

   @Override public void removeVariableFromAxisY(DoubleBufferReader bufferedData)
   {
      for (PlotVariable plotVariable : plotVariableConcurrentHashMap.values())
      {
         if (bufferedData == plotVariable.dataHolder)
         {
            dynamicChart.removeDataSeries(plotVariable.name);
            plotVariableConcurrentHashMap.remove(plotVariable.name);
            break;
         }
      }
   }

   /**
    *    DATA POINTS HANDLING
    */

   public void clearPlotVariableData(String name)
   {
      dynamicChart.clearDataSeries(name);
   }

   /**
    * Equivalent to a reset:
    * - remove or hide all points plotted on the graph
    * - reset lower and upper bound positions
    *
    */
   public void clearDataInAllSeries()
   {
      dynamicChart.clearDataInAllSeries();
      dataProgressPanManager.reset();
      lastIterationIndex.set(0);
   }

   /**
    * DataSeriesChangeListener expect events related to change in upper and lower bounds
    * @param dataSeriesManagerChangeListener
    */

   public void setDataSeriesManagerChangeListener(DataSeriesManagerChangeListener dataSeriesManagerChangeListener)
   {
      this.dataSeriesManagerChangeListener = dataSeriesManagerChangeListener;
   }

   /**
    * Animation loop makes an external call to this method to render new data
    * Go fetch data from the buffered variables added as plot variables
    *  Refreshes graph by reading data in the buffer contained between the current iteration number and the last iteration displayed
    * Directly linked to the circular buffer data structure
    */
   public void updateDataDisplayedOnGraph()
   {
      int currentIterationNumber = getCurrentIterationNumber(xAxisTimeStamp.dataHolder);
      int currentIterationIndex = currentIterationNumber - 1;

      // Case at the beginning of the simulation or if no new data and animation loop ticks
      if (currentIterationIndex < 0 || currentIterationIndex == lastIterationIndex.get())
      {
         return;
      }

      // Case if the simulation ran once
      if (currentIterationIndex < lastIterationIndex.get())
      {
         lastIterationIndex.set(0);
      }

      ImmutablePair indices = getBufferIndicesForIterationIndicesWithinRange(xAxisTimeStamp.dataHolder, lastIterationIndex.get(), currentIterationIndex);

      commitDataForDisplayFromPlotVariablesList((int) indices.left, (int) indices.right);

      this.lastIterationIndex.set(currentIterationIndex);

   } // TODO Index problem corrupts graph: missing data

   /**
    * Maps iteration indices to buffer indices
    *
    * @param doubleBufferReader
    * @param iterationLeftIndex
    * @param iterationRightIndex
    * @return Returns the left and right indices corresponding to the iteration indices
    */
   protected static ImmutablePair<Integer, Integer> getBufferIndicesForIterationIndicesWithinRange(DoubleBufferReader doubleBufferReader,
         int iterationLeftIndex, int iterationRightIndex)
   {
      int dataBufferMaxIndex = doubleBufferReader.getCapacity() - 1;
      //      int dataBufferMaxIndex = doubleBufferReader.getFillCount() - 1;

      if (iterationLeftIndex < 0)
      {
         throw new IllegalArgumentException(" left index cannot be negative");
      }

      if (iterationRightIndex >= getCurrentIterationNumber(doubleBufferReader))
      {
         throw new IllegalArgumentException(" iterationRightIndex cannot go beyond iteration number");
      }

      if (iterationLeftIndex >= iterationRightIndex)
      {
         throw new IllegalArgumentException(" iterationLeftIndex is greater than or equal to iterationRightIndex. No valid range. ");
      }

      int dataBufferLeftIndex, dataBufferRightIndex;

      // Iteration index is still smaller than the buffer size, buffer hasn't been overwritten yet
      if (iterationRightIndex <= dataBufferMaxIndex)
      {
         // Last point in data buffer sent to be displayed is the last iteration index value that was displayed
         dataBufferLeftIndex = iterationLeftIndex;
         // Since the buffer hasn't been overwritten yet current data buffer index is at iteration index position
         dataBufferRightIndex = iterationRightIndex;
      }
      else // Iteration index is greater than buffer capacity, so the buffer has been overwritten.
      {
         // Number of iterations between now and last time data were rendered
         int iterationDelta = iterationRightIndex - iterationLeftIndex;

         if (iterationDelta > dataBufferMaxIndex)
         {
            iterationDelta = dataBufferMaxIndex; // set it to a max (last data available in buffer)
         }

         dataBufferLeftIndex = dataBufferMaxIndex - iterationDelta;
         dataBufferRightIndex = dataBufferMaxIndex;
      }

      ImmutablePair<Integer, Integer> indices = new ImmutablePair<>(dataBufferLeftIndex, dataBufferRightIndex);

      return indices;
   }

   protected static int getCurrentIterationNumber(DoubleBufferReader doubleBufferReader)
   {
      int currentIterationNumber;
      if (doubleBufferReader.getFillCount() < doubleBufferReader.getCapacity())
      {
         currentIterationNumber = doubleBufferReader.getFillCount();
      }
      else
      {
         currentIterationNumber = doubleBufferReader.getOverwriteCount() * doubleBufferReader.getCapacity() + doubleBufferReader.getReadPosition();
      }
      return currentIterationNumber;
   }

   /**
    *
    * Reads content of buffered variables between two indices and stores it in a Queue to be rendered by the JavaFX GUI Thread
    *
    */
   public void commitDataForDisplayFromPlotVariablesList(int leftIndex, int rightIndex)
   {
      int length = rightIndex - leftIndex;

      if (length <= 0)
      {
         throw new RuntimeException(" Not a valid range.  leftIndex:  " + leftIndex + " rightIndex: " + rightIndex);
      }

      double[] xData = xAxisTimeStamp.dataHolder.get(leftIndex, length);
      for (PlotVariable plotVariable : plotVariableConcurrentHashMap.values())
      {
         double[] yData = plotVariable.dataHolder.get(leftIndex, length);
         dynamicChart.commitDataForDisplay(plotVariable.name, xData, yData);
      }
   }

   public void renderDataOnChart()
   {
      dynamicChart.renderDataOnCharts();
   }

}
