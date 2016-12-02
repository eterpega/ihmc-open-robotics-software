package us.ihmc.javaFXToolkit.examples.charts;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.LineChartPane;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DataSeriesManager;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentBufferedUIVariable;
import us.ihmc.javaFXToolkit.framework.data.DoubleUIVariable;
import us.ihmc.javaFXToolkit.uiSplitModel.UITriggerTimer;

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

/**

 */
public class DynamicChartExample extends Application
{

   public static final int BUFFER_SIZE = 5000;

   /*
    * Data Generator fields
    */
   private static final int GENERATOR_PERIOD_MS = 10;
   private UITriggerTimer generatorTimer = new UITriggerTimer("GeneratorTimer", GENERATOR_PERIOD_MS, TimeUnit.MILLISECONDS);

   public static final int DATA_QUEUE_MAX_NUMBER_OF_ITEMS = 100;
   public static final double DATA_QUEUE_MAX_DELAY_MS = 1000.0;
   private ConcurrentLinkedQueue<double[]> dataQueue = new ConcurrentLinkedQueue<double[]>();

   /*
    * Chart and animation loop fields
    */

   private static final int ANIMATION_PERIOD_MS = 50;
   private UITriggerTimer animationTimer = new UITriggerTimer("AnimationTimer", ANIMATION_PERIOD_MS, TimeUnit.MILLISECONDS);
   private LineChart<Number, Number> lineChart;
   private DataSeriesManager dataSeriesManager;

   /*
    * Readable variables populated by the generator, read by the TimeSeriesManager and rendered on chart
    */

   private DoubleUIVariable time = new DoubleUIVariable("Time", 0.0);
   private DoubleUIVariable variableA = new DoubleUIVariable("VariableA", 0.0);
   private DoubleUIVariable variableB = new DoubleUIVariable("VariableB", 0.0);
   private DoubleUIVariable variableC = new DoubleUIVariable("VariableC", 0.0);
   private DoubleUIVariable variableD = new DoubleUIVariable("VariableD", 0.0);

   private ConcurrentBufferedUIVariable bufferedVariableTime = new ConcurrentBufferedUIVariable(time, BUFFER_SIZE);
   private ConcurrentBufferedUIVariable bufferedVariableA = new ConcurrentBufferedUIVariable(variableA, BUFFER_SIZE);
   private ConcurrentBufferedUIVariable bufferedVariableB = new ConcurrentBufferedUIVariable(variableB, BUFFER_SIZE);
   private ConcurrentBufferedUIVariable bufferedVariableC = new ConcurrentBufferedUIVariable(variableC, BUFFER_SIZE);
   private ConcurrentBufferedUIVariable bufferedVariableD = new ConcurrentBufferedUIVariable(variableD, BUFFER_SIZE);



   /*
    * Fields for visualization and control of example
    */

   private SimpleIntegerProperty dataQueueCurrentSize = new SimpleIntegerProperty(0);
   private double beginningQueueTimestamp = 0;
   private SimpleDoubleProperty delay = new SimpleDoubleProperty(0);

   private SimpleIntegerProperty producedItemsCount = new SimpleIntegerProperty(0);
   private SimpleIntegerProperty consumedItemsCount = new SimpleIntegerProperty(0);

   private Label producerNumberItemsLabel;
   private Label consumerNumberItemsLabel;

   private boolean isPaused = true;
   private double currentTimeMs = 0;
   private int currentIteration = 0;

   private long timestampStart;

   private synchronized void incrementDataQueueCurrentSize()
   {
      dataQueueCurrentSize.set(dataQueueCurrentSize.get() + 1);

   }

   private synchronized void decrementDataQueueCurrentSize()
   {
      dataQueueCurrentSize.set(dataQueueCurrentSize.get() - 1);
   }

   private void reset()
   {
      if (isPaused)
      {

         dataQueueCurrentSize.set(0);
         producedItemsCount.set(0);
         consumedItemsCount.set(0);
         currentTimeMs = 0;
         currentIteration = 0;
         dataQueue.clear();

         bufferedVariableTime.reset();
         bufferedVariableA.reset();
         bufferedVariableB.reset();
         bufferedVariableC.reset();
         bufferedVariableD.reset();

         dataSeriesManager.clearDataInAllSeries();
      }
      else
      {
         throw new RuntimeException("Cannot reset data in the queue while the example is running");
      }
   }

   @Override public void start(Stage primaryStage) throws Exception
   {
      init(primaryStage);
   }

   private void init(Stage primaryStage)
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

       /* Add plot variables to the chart (can be done at runtime) */
      dataSeriesManager.addPlotVariableToAxisY(bufferedVariableA.getName(), bufferedVariableA);
      dataSeriesManager.addPlotVariableToAxisY(bufferedVariableB.getName(), bufferedVariableB);
      //      timeSeriesManager.addPlotVariableToAxisY(bufferedVariableC.getName(), bufferedVariableC);
      //      timeSeriesManager.addPlotVariableToAxisY(bufferedVariableD.getName(), bufferedVariableD);

      /* Data Generator update */

      generatorTimer.trigger.addListener(observable -> {
         generateNewDataEntry();
      });


      /* Chart Data Plot Update */
      animationTimer.trigger.addListener(observable -> {
         updateChart();
      });



      /*
       *  Layout and graphical components for this example
       */

      BorderPane borderPane = new BorderPane();
      borderPane.setCenter(decoratedChart);

      VBox verticalBox = new VBox();

      FlowPane flowPane = new FlowPane();
      Label dataQueueLabel = new Label(" Data Queue Load: ");
      ProgressBar progressBar = new ProgressBar(0.0);
      progressBar.setPrefWidth(200);
      final Label queueSizeLabel = new Label("  ...");

      dataQueueCurrentSize.addListener(observable -> {
         double progress = dataQueueCurrentSize.get() / Double.valueOf(DATA_QUEUE_MAX_NUMBER_OF_ITEMS);
         Platform.runLater(() -> {
            progressBar.setProgress(progress);
            queueSizeLabel.setText(" " + dataQueueCurrentSize.get() + " items");
         });
      });

      flowPane.getChildren().addAll(dataQueueLabel, progressBar, queueSizeLabel);
      verticalBox.getChildren().addAll(flowPane);

      flowPane = new FlowPane();
      dataQueueLabel = new Label(" Data Queue Delay: ");
      ProgressBar delayProgressBar = new ProgressBar(0.0);
      delayProgressBar.setPrefWidth(200);
      final Label queueDelayLabel = new Label("  ...");
      delay.addListener(observable -> {
         Platform.runLater(() -> {
            queueDelayLabel.setText(delay.get() / DATA_QUEUE_MAX_DELAY_MS + " ms ");
            delayProgressBar.setProgress(delay.get() / DATA_QUEUE_MAX_DELAY_MS);
         });

      });

      flowPane.getChildren().addAll(dataQueueLabel, delayProgressBar, queueDelayLabel);
      verticalBox.getChildren().addAll(flowPane);

      borderPane.setBottom(verticalBox);

      primaryStage.setScene(new Scene(borderPane));
      primaryStage.setOnCloseRequest(observable -> {
         decoratedChart.closeChildrenWindows();
      });

      Button simulateButton = new Button("Start Timer");

      simulateButton.setOnAction(event -> {
         if (isPaused) // start timer
         {
            reset();
            simulateButton.setText("Stop Timer");
            generatorTimer.startTimer();
            animationTimer.startTimer();
            timestampStart = System.currentTimeMillis();
            isPaused = false;
         }
         else // stop timer
         {
            simulateButton.setText("Start Timer");
            generatorTimer.stopTimer();
            animationTimer.stopTimer();
            isPaused = true;
         }

      });

      Button addEntryButton = new Button("Add Single Entry");
      addEntryButton.setOnAction(event -> {
         generateNewDataEntry();
         updateChart();
      });

      FlowPane flowPaneButtons = new FlowPane();
      flowPaneButtons.getChildren().addAll(simulateButton, addEntryButton);

      borderPane.setTop(flowPaneButtons);

      /**
       *    Example properties
       */
      VBox propertyRows = new VBox();

      propertyRows.setPrefWidth(300);
      FlowPane propertyFlowPane = new FlowPane();
      Label generatorPeriodLabel = new Label("Generator period : ");
      Label generatorPeriodValueLabel = new Label(" " + GENERATOR_PERIOD_MS + " ms");
      propertyFlowPane.getChildren().addAll(generatorPeriodLabel, generatorPeriodValueLabel);
      propertyRows.getChildren().add(propertyFlowPane);

      propertyFlowPane = new FlowPane();
      Label consumerPeriodLabel = new Label("Consumer period : ");
      Label consumerPeriodValueLabel = new Label(" " + ANIMATION_PERIOD_MS + " ms ");
      propertyFlowPane.getChildren().addAll(consumerPeriodLabel, consumerPeriodValueLabel);
      propertyRows.getChildren().add(propertyFlowPane);

      propertyFlowPane = new FlowPane();
      Label producerLabel = new Label(" Producer Data: ");
      producerNumberItemsLabel = new Label(" ...");
      propertyFlowPane.getChildren().addAll(producerLabel, producerNumberItemsLabel);
      propertyRows.getChildren().add(propertyFlowPane);

      propertyFlowPane = new FlowPane();
      Label consumerLabel = new Label(" Consumer Data: ");
      consumerNumberItemsLabel = new Label(" ...");
      propertyFlowPane.getChildren().addAll(consumerLabel, consumerNumberItemsLabel);
      propertyRows.getChildren().add(propertyFlowPane);

      borderPane.setRight(propertyRows);

      primaryStage.show();
   }

   // periodic
   private void generateNewDataEntry()
   {
      currentTimeMs = System.currentTimeMillis() - timestampStart;

      double signal = currentIteration % 2 == 0 ? 1 : -1;
      double signal20Hz = currentIteration % 10 >= 5 ? 1 : -1;

      double[] data = new double[] { currentIteration, signal, signal20Hz, signal, signal20Hz };

      dataQueue.offer(data); // time, variableA, variableB, variableC, variableD

      incrementDataQueueCurrentSize();

      currentIteration++;
      beginningQueueTimestamp = currentIteration;
   }

   private void updateChart()
   {
      double[] data;
      while ((data = dataQueue.poll()) != null)
      {
         decrementDataQueueCurrentSize();

         //difference between timestamps of the latest data put in queue and the latest data consumed
         delay.set(beginningQueueTimestamp - data[0]);

         bufferedVariableTime.storeValueAsDoubleInBuffer(data[0]);
         bufferedVariableA.storeValueAsDoubleInBuffer(data[1]);
         bufferedVariableB.storeValueAsDoubleInBuffer(data[2]);
         //         bufferedVariableC.storeValueAsDoubleInBuffer(data[3]);
         //         bufferedVariableD.storeValueAsDoubleInBuffer(data[4]);

         consumedItemsCount.set(consumedItemsCount.get() + 1);
      }

      dataSeriesManager.updateDataDisplayedOnGraph();
      dataSeriesManager.renderDataOnChart();

   }

   public static void main(String[] args)
   {
      launch(args);
   }

}