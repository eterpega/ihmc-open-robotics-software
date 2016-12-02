package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;


import javafx.scene.chart.XYChart;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.StackPane;

/**
  *Contains all ChartGraphicsLayer items that add enrich the appearance of the chart and possibly add new elements to interact with.
 *
 * Created by amoucheboeuf on 5/16/16.
 */
public class ChartGraphicsLayersContainer extends AnchorPane
{

   private final StackPane stackPane = new StackPane();
   private final XYChart chart;



   public ChartGraphicsLayersContainer(XYChart chart)
   {
      this.chart = chart;
      this.stackPane.getChildren().add(chart);

      AnchorPane.setTopAnchor(stackPane, 0.0);
      AnchorPane.setLeftAnchor(stackPane, 0.0);
      AnchorPane.setRightAnchor(stackPane, 0.0);
      AnchorPane.setBottomAnchor(stackPane, 0.0);
      this.getChildren().add(stackPane);
   }

   public XYChart getChart()
   {
      return chart;
   }

   public void addChartGraphicsLayer(ChartGraphicsLayer layer)
   {
      if (layer.getDecoratedChart() != null && layer.getDecoratedChart() == getChart())
      {
         stackPane.getChildren().add(layer);
      }
      else
      {
         throw new RuntimeException(
               "Attempted to add an incompatible ChartGraphicLayer to this ChartGraphicsDecorator. The two instances do not decorate the same Chart instance");
      }

   }


   // TODO add: removeChartGraphicsLayer(ChartGraphicsLayer layer)

}
