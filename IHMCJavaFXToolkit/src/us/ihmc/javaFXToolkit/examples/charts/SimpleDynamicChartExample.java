package us.ihmc.javaFXToolkit.examples.charts;

import gnu.trove.list.array.TDoubleArrayList;
import javafx.application.Application;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.control.Button;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.LineChartPane;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DynamicChart;
import us.ihmc.javaFXToolkit.framework.uiSplitModel.UITriggerTimer;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

/**
 * Created by adrien on 12/9/16.
 */
public class SimpleDynamicChartExample extends Application
{

   public static final int BUFFER_SIZE = 5000;

   private DynamicChart dynamicChart;
   private LineChart<Number, Number> lineChart;

   private List<String> plotVariablesList = Collections.synchronizedList(new ArrayList<String>());

   /*
    * Data Generator
    */
   public static final int GENERATOR_PERIOD_MS = 20;
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

   private int currentIteration = 0;

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

      dynamicChart = new DynamicChart(lineChart, BUFFER_SIZE);


      /* Data Generator update */
      generatorTimer.trigger.addListener(observable -> generateNewFakeDataEntry());

      /* Chart Data Plot Update */
      animationTimer.trigger.addListener(observable -> updateChart());

      generatorTimer.startTimer();
      animationTimer.startTimer();

      Button addVariable = new Button("Add Variable");
      addVariable.setOnAction(event -> addVariableToPlot());

      Button removeVariable = new Button("Remove Variable");
      removeVariable.setOnAction(event -> removeVariableFromPlot());

      FlowPane flowPane = new FlowPane(addVariable, removeVariable);

      VBox verticalBox = new VBox();

      verticalBox.getChildren().add(flowPane);
      verticalBox.getChildren().add(decoratedChart);

      primaryStage.setScene(new Scene(verticalBox));
      primaryStage.show();
   }

   private void addVariableToPlot()
   {
      if (plotVariablesList.size() >= 10)
         return;

      String varName = "var" + plotVariablesList.size();
      plotVariablesList.add(varName);
      dynamicChart.addNewDataSeries(varName);
   }

   private void removeVariableFromPlot()
   {
      if (plotVariablesList.isEmpty())
         return;

      dynamicChart.removeDataSeries(plotVariablesList.remove(plotVariablesList.size() - 1));
   }

   // periodic
   private void generateNewFakeDataEntry()
   {
      if (!plotVariablesList.isEmpty())
      {
         int length = plotVariablesList.size();
         double[] data = new double[length + 1];

         data[0] = currentIteration;
         for (int i = 0; i < length; i++)
         {
            data[i + 1] = Math.sin(currentIteration / (10.0 * (i + 1)) * Math.PI);
         }

         dataQueue.offer(data); // For visualization purposes only: time, first variable with steps for each iteration and then just bunch of data
      }

      currentIteration++;
      beginningQueueTimestamp = currentIteration;
   }

   private void updateChart()
   {
      if (plotVariablesList.isEmpty())
         return;

      double[] data;
      ArrayList<TDoubleArrayList> xData = new ArrayList<>();
      ArrayList<TDoubleArrayList> yData = new ArrayList<>();

      // dataQueue does not always contain arrays that have the same number of element depending on the number of plotted variables
      while ((data = dataQueue.poll()) != null)
      {
         int min = Math.min(data.length - 1, plotVariablesList.size());
         if (yData.size() < min)
         {
            while (yData.size() < min)
            {
               TDoubleArrayList tempX = new TDoubleArrayList();
               TDoubleArrayList tempY = new TDoubleArrayList();
               xData.add(tempX);
               yData.add(tempY);
            }
         }
         else if (yData.size() > min)
         {
            while (yData.size() > min)
            {
               xData.remove(0);
               yData.remove(0);
            }
         }

         for (int i = 1; i <= min; i++)
         {
            xData.get(i - 1).add(data[0]);
            yData.get(i - 1).add(data[i]);
         }
      }

      for (int i = 0; i < yData.size(); i++)
      {
         dynamicChart.commitDataForDisplay(plotVariablesList.get(i),  xData.get(i).toArray(), yData.get(i).toArray());
      }

      dynamicChart.renderDataOnCharts();
   }

}
