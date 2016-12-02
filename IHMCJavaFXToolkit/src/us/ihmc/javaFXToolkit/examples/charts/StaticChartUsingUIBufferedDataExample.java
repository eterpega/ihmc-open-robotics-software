package us.ihmc.javaFXToolkit.examples.charts;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.LineChartPane;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DataSeriesManager;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentBufferedUIVariable;
import us.ihmc.javaFXToolkit.framework.data.DoubleUIVariable;

import java.util.Arrays;

/**
 * Created by amoucheboeuf on 7/1/16.
 */
public class StaticChartUsingUIBufferedDataExample extends Application
{

   private Stage primaryStage;
   final NumberAxis xAxis = new NumberAxis();
   final NumberAxis yAxis = new NumberAxis();

   private DoubleUIVariable time = new DoubleUIVariable("Time", 0.0);
   private DoubleUIVariable variableA = new DoubleUIVariable("VariableA", 0.0);
//   private DoubleUIVariable variableB = new DoubleUIVariable("VariableB", 0.0);
//   private DoubleUIVariable variableC = new DoubleUIVariable("VariableC", 0.0);
//   private DoubleUIVariable variableD = new DoubleUIVariable("VariableD", 0.0);

   private ConcurrentBufferedUIVariable bufferedVariableTime = new ConcurrentBufferedUIVariable(time, 1024);
   private ConcurrentBufferedUIVariable bufferedVariableA = new ConcurrentBufferedUIVariable(variableA, 1024);
//   private BufferedUIVariable bufferedVariableB = new BufferedUIVariable(variableB, 1024);
//   private BufferedUIVariable bufferedVariableC = new BufferedUIVariable(variableC, 1024);
//   private BufferedUIVariable bufferedVariableD = new BufferedUIVariable(variableD, 1024);

   @Override public void start(Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;

      /**
       *  Instantiate your axes and chart
       */

      System.out.println("Creating LineChart");
      final LineChart<Number, Number> lineChart = new LineChart(xAxis, yAxis);
      lineChart.setCreateSymbols(false);
      lineChart.setTitle("Test Static Chart Using BufferedUIVariables and TimeSeriesManager");
      xAxis.setLabel("X Axis");
      yAxis.setLabel("Y Axis");
      LineChartPane lineChartPane = new LineChartPane(lineChart);

      /**
       *  Generate data
       */

      System.out.println("Creating TimeSeriesManager");

      /* TimeSeriesManager takes care of adding/removing plot variables as well as reading their content and rendering them on the chart */
      DataSeriesManager dataSeriesManager = new DataSeriesManager(lineChart, bufferedVariableTime.getName(), bufferedVariableTime);

       /* Add plot variables to the chart (can be done at runtime) */
      dataSeriesManager.addPlotVariableToAxisY(bufferedVariableA.getName(), bufferedVariableA);

      System.out.println("Populating BufferedVariables");
      for (int i = 0; i < 10; i++)
      {
         bufferedVariableTime.storeValueInBuffer(1.0*i);
         bufferedVariableA.storeValueInBuffer( Math.cos(i / 100.0 * 2 * Math.PI));
//         series2.getData().add(new XYChart.Data<Number, Number>(i, Math.sin(i / 100.0 * 20 * Math.PI)));
      }
      System.out.println(" Buffered Variable Time : \n" + Arrays.toString(bufferedVariableTime.getBufferedDataCopy())+ " \n");
      System.out.println(" Buffered Variable A : \n" + Arrays.toString(bufferedVariableA.getBufferedDataCopy())+ " \n");


      System.out.println("Buffered Variable A fill count: "+ bufferedVariableA.getBufferFillCount());

//      System.out.println("Loading data to chart buffer");
//      timeSeriesManager.commitDataForDisplayFromPlotVariablesList(0, bufferedVariableA.getBufferFillCount());
//
//      System.out.println("Rendering data to chart plot area");
//      timeSeriesManager.renderDataOnCharts();

      /**
       *  Feed your chart to the chart decorator
       */

      this.primaryStage.setScene(new Scene(lineChartPane));
      this.primaryStage.setOnCloseRequest(observable -> {
         lineChartPane.closeChildrenWindows();
      });

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}




