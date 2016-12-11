package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import javafx.application.Platform;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.ObservableList;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.XYChart;
import us.ihmc.robotics.MathTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Act as an external layer to the linechart and provide methods to create and remove data series, add data to these dataSeries by queuing them and wait for render call to poll the data in ready to be displayed
 *
 * Class that should only contain methods to perform operation on chart add and remove data points and that is meant to improve the rendering time ad amount of data that can be stored on a graph
 *
 * Created by amoucheboeuf on 7/7/16.
 */
public class DynamicChart
{
   public static final int MIN_BUFFER_SIZE = 32;
   public static final int MAX_BUFFER_SIZE = 32000;

   // Contains all immutable data points in queue waiting to be rendered by the javafx thread
   private final ConcurrentLinkedQueue<DataForDisplay> newValuesToPlotQueue = new ConcurrentLinkedQueue<DataForDisplay>();

   private final ConcurrentHashMap<String, XYChart.Series> dataSeriesMap;

   private final LineChart lineChart;

   private int chartBufferCapacity;

   private final SimpleIntegerProperty chartBufferFillCount = new SimpleIntegerProperty();

   private final SimpleDoubleProperty allPointsLowerBoundX = new SimpleDoubleProperty();
   private final SimpleDoubleProperty allPointsUpperBoundX = new SimpleDoubleProperty();

   private final SimpleDoubleProperty allPointsLowerBoundY = new SimpleDoubleProperty();
   private final SimpleDoubleProperty allPointsUpperBoundY = new SimpleDoubleProperty();

   private final SimpleDoubleProperty lastPointDataSetLowerBoundX = new SimpleDoubleProperty();
   private final SimpleDoubleProperty lastPointDataSetUpperBoundX = new SimpleDoubleProperty();

   private final SimpleDoubleProperty lastPointDataSetLowerBoundY = new SimpleDoubleProperty();
   private final SimpleDoubleProperty lastPointDataSetUpperBoundY = new SimpleDoubleProperty();

   public DynamicChart(LineChart lineChart, int capacity)
   {
      this.lineChart = lineChart;
      this.setChartBufferCapacity(capacity);
      this.dataSeriesMap = new ConcurrentHashMap<>();
   }

   /**
    * // TODO implement adjust buffer capacity at runtime
    * If the buffer capacity is reduced, will keep the latest data
    * @param capacity
    */
   private void setChartBufferCapacity(int capacity)
   {
      MathTools.checkIfInRange(capacity, MIN_BUFFER_SIZE, MAX_BUFFER_SIZE);
      this.chartBufferCapacity = capacity;
   }

   public int getChartBufferCapacity()
   {
      return chartBufferCapacity;
   }

   /**
    * Make that call to offer a new DataForDisplay object to the chart queue and call {@link DataSeriesManager#()} method periodically for them to be rendered
    * @throws IllegalStateException
    */
   public void commitDataForDisplay(String name, double[] xData, double[] yData)
   {
      if (dataSeriesMap.get(name) == null)
         throw new RuntimeException("The data series called \" " + name + "\" does not exist.");

      DataForDisplay dataForDisplay = new DataForDisplay(dataSeriesMap.get(name), xData, yData);
      newValuesToPlotQueue.offer(dataForDisplay);
   }

   /**
    * Make that call to poll the queue containing the DataForDisplay items and add them to be rendered
    * @throws IllegalStateException
    */
   public void renderDataOnCharts() throws IllegalStateException
   {
      if (newValuesToPlotQueue.isEmpty())
         return;

      Platform.runLater(() ->
                        { //TODO reduce too many calls of run later, try to use bind on data or make sure thread is completed before next call to

                           if (!Platform.isFxApplicationThread())
                           {
                              throw new IllegalStateException("Method renderDataOnCharts() not on JavaFx application thread");
                           }

                           // look up the data contained in new Values to chart list and uses data for display to add data to the series
                           DataForDisplay dataForDisplay;
                           while ((dataForDisplay = newValuesToPlotQueue.poll()) != null)
                           {
                              addDataToSeries(dataForDisplay);
                              setDataSeriesBoundsValues(dataForDisplay);
                           }
                        });
   }

   private void setDataSeriesBoundsValues(DataForDisplay dataForDisplay)
   {
      lastPointDataSetLowerBoundX.set(dataForDisplay.xMin);
      lastPointDataSetUpperBoundX.set(dataForDisplay.xMax);
      lastPointDataSetLowerBoundY.set(dataForDisplay.yMin);
      lastPointDataSetUpperBoundY.set(dataForDisplay.yMax);

      allPointsLowerBoundX.set(Math.min(allPointsLowerBoundX.get(), dataForDisplay.xMin));
      allPointsUpperBoundX.set(Math.max(allPointsUpperBoundX.get(), dataForDisplay.xMax));
      allPointsLowerBoundY.set(Math.min(allPointsLowerBoundY.get(), dataForDisplay.yMin));
      allPointsUpperBoundY.set(Math.max(allPointsUpperBoundY.get(), dataForDisplay.yMax));
      //
      //      System.out.println(" xMin " + dataForDisplay.xMin + " xmax " + dataForDisplay.xMax + " ymin " + dataForDisplay.yMin + " ymax " + dataForDisplay.yMax);
      //      System.out.println(" xMIN " + allPointsLowerBoundX.get() + " xMAX " + allPointsUpperBoundX.get() + " yMIN " + allPointsLowerBoundY.get() + " yMAX "
      //            + allPointsUpperBoundY.get());
   }

   private void addDataToSeries(DataForDisplay dataForDisplay)
   {
      addDataToSeries(dataForDisplay.targetSeries, dataForDisplay.xValues, dataForDisplay.yValues);
   }

   /**
    * Create new points until the data series reaches the chartbufferCapacity limit. Then reuses created points and assign new values to them. Recycle pool of objects.
    * @param series
    * @param X
    * @param Y
    */
   private void addDataToSeries(XYChart.Series series, double[] X, double[] Y)
   {

      //      System.out.println("Adding data to series X:"+ Arrays.toString(X));
      //      System.out.println("Adding data to series Y:"+ Arrays.toString(Y));
      //      System.out.println();

      int newDataSize = X.length;
      int size = series.getData().size();

      ArrayList<XYChart.Data<Number, Number>> dataPoints = new ArrayList<>(newDataSize);  // I Don't like that array list

      if (size >= chartBufferCapacity) // if new data array length to be added is greater than the chart's buffer size consider sampling data?
      {
         // take data points from the end of the list
         dataPoints.addAll(series.getData().subList(0, newDataSize));
         series.getData().remove(0, newDataSize);
      }
      else // create new data points
      {
         for (int i = 0; i < newDataSize; i++)
            dataPoints.add(new XYChart.Data<>());
      }

      XYChart.Data<Number, Number> dataPoint = null;
      for (int i = 0; i < newDataSize; i++)
      {
         dataPoint = dataPoints.get(i);
         dataPoint.setXValue(X[i]);
         dataPoint.setYValue(Y[i]);
      }

      series.getData().addAll(dataPoints);
   }

   public void addNewDataSeries(String name)
   {
      if (dataSeriesMap.get(name) == null)
      {
         XYChart.Series<Number, Number> dataSeries = new XYChart.Series<>();
         dataSeries.setName(name);
         lineChart.getData().add(dataSeries);
         dataSeriesMap.put(name, dataSeries);
      }

   }

   public void removeDataSeries(String name)
   {
      if (dataSeriesMap.get(name) != null)
      {
         System.out.println("Removing Data Series: " + name);
         lineChart.getData().remove(dataSeriesMap.get(name));
         dataSeriesMap.remove(name);
      }
   }

   public void clearDataSeries(String name)
   {
      clearDataSeries(dataSeriesMap.get(name));
   }

   public void clearDataInAllSeries()
   {
      for (XYChart.Series series : dataSeriesMap.values())
      {
         clearDataSeries(series);
      }
   }

   private void clearDataSeries(XYChart.Series series) // TODO  Preferably not to be done while adding Data?
   {
      synchronized (series)
      {
         ObservableList<XYChart.Data<Number, Number>> oldDataPoints = series.getData();

         for (int i = 0; i < oldDataPoints.size(); i++)
         {
            oldDataPoints.get(i).setXValue(-1); // TODO temporary, need to find a better way to represent data that has disappeared without deleting points
            oldDataPoints.get(i).setYValue(0.0);
         }
         lastPointDataSetLowerBoundX.set(0);
         lastPointDataSetUpperBoundX.set(0);
         lastPointDataSetLowerBoundY.set(0);
         lastPointDataSetUpperBoundY.set(0);

         // TODO maybe zero is not appropriate
         allPointsLowerBoundX.set(0);
         allPointsUpperBoundX.set(0);
         allPointsLowerBoundY.set(0);
         allPointsUpperBoundY.set(0);

         chartBufferFillCount.set(0);
      }
   }

   public LineChart getLineChart()
   {
      return lineChart;
   }

   public double getAllPointsLowerBoundX()
   {
      return allPointsLowerBoundX.get();
   }

   public SimpleDoubleProperty allPointsLowerBoundXProperty()
   {
      return allPointsLowerBoundX;
   }

   public double getAllPointsUpperBoundX()
   {
      return allPointsUpperBoundX.get();
   }

   public SimpleDoubleProperty allPointsUpperBoundXProperty()
   {
      return allPointsUpperBoundX;
   }

   public double getAllPointsLowerBoundY()
   {
      return allPointsLowerBoundY.get();
   }

   public SimpleDoubleProperty allPointsLowerBoundYProperty()
   {
      return allPointsLowerBoundY;
   }

   public double getAllPointsUpperBoundY()
   {
      return allPointsUpperBoundY.get();
   }

   public SimpleDoubleProperty allPointsUpperBoundYProperty()
   {
      return allPointsUpperBoundY;
   }

   public double getLastPointDataSetLowerBoundX()
   {
      return lastPointDataSetLowerBoundX.get();
   }

   public SimpleDoubleProperty lastPointDataSetLowerBoundXProperty()
   {
      return lastPointDataSetLowerBoundX;
   }

   public double getLastPointDataSetUpperBoundX()
   {
      return lastPointDataSetUpperBoundX.get();
   }

   public SimpleDoubleProperty lastPointDataSetUpperBoundXProperty()
   {
      return lastPointDataSetUpperBoundX;
   }

   public double getLastPointDataSetLowerBoundY()
   {
      return lastPointDataSetLowerBoundY.get();
   }

   public SimpleDoubleProperty lastPointDataSetLowerBoundYProperty()
   {
      return lastPointDataSetLowerBoundY;
   }

   public double getLastPointDataSetUpperBoundY()
   {
      return lastPointDataSetUpperBoundY.get();
   }

   public SimpleDoubleProperty lastPointDataSetUpperBoundYProperty()
   {
      return lastPointDataSetUpperBoundY;
   }

   public int getChartBufferFillCount()
   {
      return chartBufferFillCount.get();
   }

   public SimpleIntegerProperty chartBufferFillCountProperty()
   {
      return chartBufferFillCount;
   }

}
