package us.ihmc.javaFXToolkit.charts;

import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.chart.LineChart;
import javafx.scene.layout.AnchorPane;

import java.io.IOException;
import java.net.URL;

/**
 * Contains the chart and add behavior such as Zooming, panning, scrolling, data selection, and chart information display commands
 *
 * Also listens to drag and drop events when properties objects are dropped on the chart, new series are added.
 *
 * Monitors incoming data and find min and max so as to set the axes properties for zoom etc.
 *
 * Disable certain behaviors when the chart is being edited (data added to series)
 *
 *
 *
 * Created by amoucheboeuf on 5/13/16.
 */
public class LineChartPane extends AnchorPane
{
   final CustomizableChartController customizableChartController;

   final LineChart chart;

   public LineChartPane(LineChart chart)
   {
      this.chart = chart;
      this.customizableChartController = new CustomizableChartController(chart);

      try
      {
         System.out.println(System.getProperty("user.dir"));

         final URL url = getClass().getResource("/xml/CustomizableXYChart.fxml");
         final FXMLLoader fxmlLoader = new FXMLLoader(url);
         fxmlLoader.setController(customizableChartController);
         Node node = fxmlLoader.load();
         this.getChildren().add(node);

         AnchorPane.setBottomAnchor(node, 0.0);
         AnchorPane.setTopAnchor(node, 0.0);
         AnchorPane.setLeftAnchor(node, 0.0);
         AnchorPane.setRightAnchor(node, 0.0);
      }
      catch (IOException ex)
      {
         System.err.println("Error loading: " + ex);
      }
   }






   public void closeChildrenWindows()
   {
      customizableChartController.closeChildrenWindows();
   }

}
