package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;

import javafx.geometry.Bounds;
import javafx.scene.Node;
import javafx.scene.chart.XYChart;
import javafx.scene.layout.Pane;

/**
 * Created by amoucheboeuf on 5/16/16.
 */
public abstract class ChartGraphicsLayer extends Pane
{

   protected final ChartGraphicsLayersContainer parentContainer;
   protected final XYChart<Number, Number> xyChart;
   protected final Node plotArea;

   public ChartGraphicsLayer(ChartGraphicsLayersContainer parentContainer)
   {
      setMouseTransparent(true);
      this.parentContainer = parentContainer;
      this.xyChart = parentContainer.getChart();
      this.plotArea = xyChart.lookup(".chart-plot-background");
   }

   public Node getDecoratedChart()
   {
      return xyChart;
   }


   protected AreaLimits getPlotAreaLimits()
   {
      Bounds plotAreaBounds = plotArea.getBoundsInParent();
      Bounds plotAreaParentBounds = plotArea.getParent().getBoundsInParent();

      double xmin = plotAreaParentBounds.getMinX() + plotAreaBounds.getMinX();
      double xmax = xmin + plotAreaBounds.getWidth();

      double ymin = plotAreaParentBounds.getMinY() + plotAreaBounds.getMinY();
      double ymax = ymin + plotAreaBounds.getHeight();

      return new AreaLimits(xmin, xmax, ymin, ymax);

   }

}
