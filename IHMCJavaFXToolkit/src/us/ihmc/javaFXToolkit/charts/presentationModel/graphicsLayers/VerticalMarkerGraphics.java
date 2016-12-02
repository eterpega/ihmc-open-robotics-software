package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.event.EventHandler;
import javafx.scene.chart.Axis;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.shape.Line;
import us.ihmc.javaFXToolkit.charts.DrawingGraphicsUtils;
import us.ihmc.robotics.MathTools;

/**
 * Created by amoucheboeuf on 5/16/16.
 */
public class VerticalMarkerGraphics extends ChartGraphicsLayer
{

   private SimpleDoubleProperty markerPosition;
   private SimpleDoubleProperty markerPositionPercentage;

   private SimpleBooleanProperty verticalMarkerGraphicsEnabled;

   private final Line line;
   private Axis<Number> xAxis;


   public VerticalMarkerGraphics(ChartGraphicsLayersContainer parentContainer)
   {
      super(parentContainer);

      // TODO offer possibility to create customizable marker

      this.line = DrawingGraphicsUtils.makeLine(Color.BLACK, 2.0);
      this.xAxis = xyChart.getXAxis();
      this.getChildren().add(line);

      this.markerPosition = new SimpleDoubleProperty();
      this.verticalMarkerGraphicsEnabled = new SimpleBooleanProperty(false);
      this.verticalMarkerGraphicsEnabled.addListener((observable, oldValue, newValue) -> setVerticalMarkerGraphicsEnabled(newValue));

      parentContainer.addChartGraphicsLayer(this);
   }

   private void setVerticalMarkerGraphicsEnabled(boolean isEnabled)
   {
      if (isEnabled)
      {
         this.xyChart.addEventHandler(MouseEvent.MOUSE_PRESSED, drawMarker);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_DRAGGED, adjustMarkerPosition);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_RELEASED, hideMarker);
      }
      else
      {
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_PRESSED, drawMarker);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_DRAGGED, adjustMarkerPosition);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_RELEASED, hideMarker);
      }
   }

   private AreaLimits tempDrawingArea;

   private EventHandler<MouseEvent> drawMarker = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if(event.getButton() != MouseButton.PRIMARY) return;

         tempDrawingArea = getPlotAreaLimits();

         double x = MathTools.clipToMinMax(event.getX(), tempDrawingArea.xmin, tempDrawingArea.xmax);

         line.setStartX(x);
         line.setEndX(x);
         line.setStartY(tempDrawingArea.ymin);
         line.setEndY(tempDrawingArea.ymax);

         double xOnAxis = x - tempDrawingArea.xmin;
         markerPosition.set(xAxis.getValueForDisplay(xOnAxis).doubleValue());
         line.setVisible(true);
      }
   };

   private EventHandler<MouseEvent> adjustMarkerPosition = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if(event.getButton() != MouseButton.PRIMARY) return;

         double x = MathTools.clipToMinMax(event.getX(), tempDrawingArea.xmin, tempDrawingArea.xmax);
         double xOnAxis = x - tempDrawingArea.xmin;
         markerPosition.set(xAxis.getValueForDisplay(xOnAxis).doubleValue());
         line.setStartX(x);
         line.setEndX(x);
      }
   };

   private EventHandler<MouseEvent> hideMarker = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if(event.getButton() != MouseButton.PRIMARY) return;

         double x = MathTools.clipToMinMax(event.getX(), tempDrawingArea.xmin, tempDrawingArea.xmax);
         double xOnAxis = x - tempDrawingArea.xmin;
         markerPosition.set(xAxis.getValueForDisplay(xOnAxis).doubleValue());
         line.setVisible(false);
      }
   };


   public SimpleDoubleProperty markerPositionProperty()
   {
      return markerPosition;
   }

   public SimpleBooleanProperty verticalMarkerGraphicsEnabledProperty()
   {
      return verticalMarkerGraphicsEnabled;
   }
}
