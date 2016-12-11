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
import us.ihmc.javaFXToolkit.framework.data.dataStructures.BufferedUIVariableDataWriter;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentBufferedUIVariable;
import us.ihmc.javaFXToolkit.framework.data.DoubleUIVariable;
import us.ihmc.javaFXToolkit.framework.uiSplitModel.UITriggerTimer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

/**

 */
public class BufferedVariablesDynamicChartExample extends Application
{

   public static final int DATA_QUEUE_MAX_NUMBER_OF_ITEMS = 100;
   public static final double DATA_QUEUE_MAX_DELAY_MS = 1000.0;

   public static final int BUFFER_SIZE = 5000;


   /*
    * Data Generator fields
    */
   public static final int GENERATOR_PERIOD_MS = 10;

   private UITriggerTimer generatorTimer = new UITriggerTimer("GeneratorTimer", GENERATOR_PERIOD_MS, TimeUnit.MILLISECONDS);
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

   public static final int MAX_NUMBER_RANDOM_VARIABLES = 10;

   private DoubleUIVariable time = new DoubleUIVariable("Time", 0.0);
   private ConcurrentBufferedUIVariable bufferedVariableTime = new ConcurrentBufferedUIVariable(time, BUFFER_SIZE);

   private List<ConcurrentBufferedUIVariable> randomBufferedUIVariables = Collections.synchronizedList(new ArrayList<>());

   private int diplayedVariablesCount = 0;

   private void addRandomVariableToPlot()
   {

      String variableName = "var" + randomBufferedUIVariables.size();
      DoubleUIVariable doubleUIVariable = new DoubleUIVariable(variableName, 0.0);
      ConcurrentBufferedUIVariable randomVariable = new ConcurrentBufferedUIVariable(doubleUIVariable, BUFFER_SIZE);
      randomBufferedUIVariables.add(randomVariable);

      diplayedVariablesCount++;

      if (diplayedVariablesCount > MAX_NUMBER_RANDOM_VARIABLES)
      {
         diplayedVariablesCount = MAX_NUMBER_RANDOM_VARIABLES;
         return;
      }

      System.out.println(" displayed variable count " + diplayedVariablesCount);
      dataSeriesManager.addPlotVariableToAxisY(randomVariable.getName(), randomVariable);

   }

   private void removeLastRandomVariable()
   {
      if (diplayedVariablesCount == 0)
      {
         return;
      }

      ConcurrentBufferedUIVariable variable = randomBufferedUIVariables.get(diplayedVariablesCount - 1);
      System.out.println("removeLastRandomVariable index =  " + (diplayedVariablesCount - 1));
      dataSeriesManager.removeVariableFromAxisY(variable.getName());
      diplayedVariablesCount--;

   }

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
         // Example:         bufferedVariableA.reset();
         randomBufferedUIVariables.forEach(BufferedUIVariableDataWriter::reset);

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

      /* Data Generator update */

      generatorTimer.trigger.addListener(observable ->
                                         {
                                            generateNewFakeDataEntry();
                                         });


      /* Chart Data Plot Update */
      animationTimer.trigger.addListener(observable ->
                                         {
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

      dataQueueCurrentSize.addListener(observable ->
                                       {
                                          double progress = dataQueueCurrentSize.get() / Double.valueOf(DATA_QUEUE_MAX_NUMBER_OF_ITEMS);
                                          Platform.runLater(() ->
                                                            {
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
      delay.addListener(observable ->
                        {
                           Platform.runLater(() ->
                                             {
                                                queueDelayLabel.setText(delay.get() / DATA_QUEUE_MAX_DELAY_MS + " ms ");
                                                delayProgressBar.setProgress(delay.get() / DATA_QUEUE_MAX_DELAY_MS);
                                             });

                        });

      flowPane.getChildren().addAll(dataQueueLabel, delayProgressBar, queueDelayLabel);
      verticalBox.getChildren().addAll(flowPane);

      borderPane.setBottom(verticalBox);

      primaryStage.setScene(new Scene(borderPane));
      primaryStage.setOnCloseRequest(observable ->
                                     {
                                        decoratedChart.closeChildrenWindows();
                                     });

      Button addEntryButton = new Button("Add Single Entry");
      addEntryButton.setOnAction(event ->
                                 {
                                    generateNewFakeDataEntry();
                                    updateChart();
                                 });


      Button addVariableButton = new Button("Add Random Variable");
      addVariableButton.setOnAction(event -> addRandomVariableToPlot());

      Button removeVariableButton = new Button("Remove last Random Variable");
      removeVariableButton.setOnAction(event -> removeLastRandomVariable());


      Button simulateButton = new Button("Start Timer");

      simulateButton.setOnAction(event ->
                                 {
                                    if (isPaused) // start timer
                                    {
                                       addVariableButton.disableProperty().setValue(true);
                                       removeVariableButton.disableProperty().setValue(true);
                                       reset();
                                       simulateButton.setText("Stop Timer");
                                       generatorTimer.startTimer();
                                       animationTimer.startTimer();
                                       timestampStart = System.currentTimeMillis();
                                       isPaused = false;
                                    }
                                    else // stop timer
                                    {
                                       addVariableButton.disableProperty().setValue(false);
                                       removeVariableButton.disableProperty().setValue(false);
                                       simulateButton.setText("Start Timer");
                                       generatorTimer.stopTimer();
                                       animationTimer.stopTimer();
                                       isPaused = true;
                                    }

                                 });



      FlowPane flowPaneButtons = new FlowPane();
      flowPaneButtons.getChildren().addAll(simulateButton, addEntryButton, addVariableButton, removeVariableButton);

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
   private void generateNewFakeDataEntry()
   {
      currentTimeMs = System.currentTimeMillis() - timestampStart;

      double[] data = new double[MAX_NUMBER_RANDOM_VARIABLES + 1];

      data[0] = currentIteration;
      for (int i = 1; i < data.length; i++)
         data[i] = Math.sin(currentIteration / (10.0 * i) * Math.PI);

      dataQueue.offer(data); // For visualization purposes only: time, first variable with steps for each iteration and then just bunch of data

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
         int index = 1;
         for (ConcurrentBufferedUIVariable variable : randomBufferedUIVariables)
            variable.storeValueAsDoubleInBuffer(data[index++]);

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