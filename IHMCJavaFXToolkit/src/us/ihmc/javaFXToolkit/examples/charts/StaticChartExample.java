package us.ihmc.javaFXToolkit.examples.charts;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.LineChartPane;

/**
 *
 * Created by amoucheboeuf on 6/6/16.
 */
public class StaticChartExample extends Application
{

   private Stage primaryStage;
   final NumberAxis xAxis = new NumberAxis();
   final NumberAxis yAxis = new NumberAxis();

   @Override public void start(Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;

      /**
       *  Instantiate your axes and chart
       */
      final NumberAxis xAxis = new NumberAxis();
      final NumberAxis yAxis = new NumberAxis();

      final LineChart<Number, Number> lc = new LineChart(xAxis, yAxis);

      lc.setCreateSymbols(false);

      // setup chart
      lc.setTitle("Test Decorated LineChart");
      xAxis.setLabel("X Axis");
      yAxis.setLabel("Y Axis");


      // add starting data
      XYChart.Series<Number, Number> series1 = new XYChart.Series<Number, Number>();
      series1.setName("Data Series 1");

      XYChart.Series<Number, Number> series2 = new XYChart.Series<Number, Number>();
      series2.setName("Data Series 2");

      for (int i = 0; i < 100; i++)
      {
         series1.getData().add(new XYChart.Data<Number, Number>(i, Math.cos(i / 100.0 * 2 * Math.PI)));
         series2.getData().add(new XYChart.Data<Number, Number>(i, Math.sin(i / 100.0 * 20 * Math.PI)));
      }
      lc.getData().add(series1);
      lc.getData().add(series2);

      lc.getXAxis().setVisible(false);
      lc.getYAxis().setVisible(false);

      /**
       *  Feed your chart to the chart decorator
       */
      LineChartPane lineChartPane = new LineChartPane(lc);

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
