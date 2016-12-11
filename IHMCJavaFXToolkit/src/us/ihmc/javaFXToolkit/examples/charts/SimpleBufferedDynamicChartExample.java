package us.ihmc.javaFXToolkit.examples.charts;

import javafx.application.Application;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.control.Label;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.LineChartPane;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DataSeriesManager;
import us.ihmc.javaFXToolkit.framework.data.DoubleUIVariable;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentBufferedUIVariable;
import us.ihmc.javaFXToolkit.framework.uiSplitModel.UITriggerTimer;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

/**
 * Created by adrien on 12/9/16.
 */
public class SimpleBufferedDynamicChartExample extends Application
{

   public static final int BUFFER_SIZE = 5000;

   private DoubleUIVariable time = new DoubleUIVariable("Time", 0.0);
   private ConcurrentBufferedUIVariable bufferedVariableTime = new ConcurrentBufferedUIVariable(time, BUFFER_SIZE);

   private DoubleUIVariable doubleUIVariable = new DoubleUIVariable("variableA", 0.0);
   private ConcurrentBufferedUIVariable variableA = new ConcurrentBufferedUIVariable(doubleUIVariable, BUFFER_SIZE);

   private LineChart<Number, Number> lineChart;
   private DataSeriesManager dataSeriesManager;
   /*
    * Data Generator
    */
   public static final int GENERATOR_PERIOD_MS = 10;
   private UITriggerTimer generatorTimer = new UITriggerTimer("GeneratorTimer", GENERATOR_PERIOD_MS, TimeUnit.MILLISECONDS);

   private ConcurrentLinkedQueue<double[]> dataQueue = new ConcurrentLinkedQueue<double[]>();

   /*

   /*
    * Chart and animation loop
    */
   private static final int ANIMATION_PERIOD_MS = 50;
   private UITriggerTimer animationTimer = new UITriggerTimer("AnimationTimer", ANIMATION_PERIOD_MS, TimeUnit.MILLISECONDS);

      /*
    * Fields for visualization and control of example
    */

   private double beginningQueueTimestamp = 0;
   private SimpleDoubleProperty delay = new SimpleDoubleProperty(0);

   private SimpleIntegerProperty consumedItemsCount = new SimpleIntegerProperty(0);

   private int currentIteration = 0;

   public SimpleBufferedDynamicChartExample()
   {

   }

   @Override public void start(Stage primaryStage) throws Exception
   {

         /* Instantiate your axes and chart as usual */
      final NumberAxis xAxis = new NumberAxis();
      final NumberAxis yAxis = new NumberAxis();

      lineChart = new LineChart(xAxis, yAxis);
      lineChart.setCreateSymbols(false);
      lineChart.setTitle("Test Decorated LineChart");
      xAxis.setLabel("X Axis");
      yAxis.setLabel("Y Axis");

      lineChart.setAnimated(false);

       /* Feed your chart to the chart decorator*/
      LineChartPane decoratedChart = new LineChartPane(lineChart);


      /* TimeSeriesManager takes care of adding/removing plot variables as well as reading their content and rendering them on the chart */
      dataSeriesManager = new DataSeriesManager(lineChart, bufferedVariableTime.getName(), bufferedVariableTime);

      /* Add buffered variable to plot */
      dataSeriesManager.addPlotVariableToAxisY(variableA.getName(), variableA);


      /* Data Generator update */
      generatorTimer.trigger.addListener(observable -> generateNewFakeDataEntry());

      /* Chart Data Plot Update */
      animationTimer.trigger.addListener(observable -> updateChart());

      generatorTimer.startTimer();
      animationTimer.startTimer();

      primaryStage.setScene(new Scene(decoratedChart));
      primaryStage.show();
   }

   // periodic
   private void generateNewFakeDataEntry()
   {
      double[] data = new double[2];

      data[0] = currentIteration;
      data[1] = Math.sin(currentIteration / 10.0 * Math.PI);

      dataQueue.offer(data); // For visualization purposes only: time, first variable with steps for each iteration and then just bunch of data

      currentIteration++;
      beginningQueueTimestamp = currentIteration;
   }

   private void updateChart()
   {
      double[] data;
      while ((data = dataQueue.poll()) != null)
      {
         //difference between timestamps of the latest data put in queue and the latest data consumed
         delay.set(beginningQueueTimestamp - data[0]);

         bufferedVariableTime.storeValueAsDoubleInBuffer(data[0]);
         variableA.storeValueAsDoubleInBuffer(data[1]);
      }

      dataSeriesManager.updateDataDisplayedOnGraph();
      dataSeriesManager.renderDataOnChart();
   }

}
