package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.chart.Axis;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import us.ihmc.javaFXToolkit.charts.DrawingGraphicsUtils;
import us.ihmc.robotics.MathTools;

/**
 * Class responsible for drawing a rectangle when a mouse click is performed on the chart
 * Created by amoucheboeuf on 5/16/16.
 */
public class SelectionRectangleGraphics extends ChartGraphicsLayer
{

   private final Axis<Number> xAxis, yAxis;
   private final Rectangle rectangle;

   private final SimpleBooleanProperty enableSelectionRectangle = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty selectionContinouslyTracked = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty isSelecting = new SimpleBooleanProperty(false);

   private AreaLimits axisSelectionRectangle;

   private SimpleObjectProperty<SelectionRectangle> selectionRectangle = new SimpleObjectProperty();

   public SelectionRectangleGraphics(ChartGraphicsLayersContainer parentContainer)
   {
      super(parentContainer);

      this.rectangle = DrawingGraphicsUtils.makeSelectionRectangle(Color.LIGHTBLUE, Color.DODGERBLUE, 2.0f);

      this.getChildren().add(rectangle);

      this.xAxis = xyChart.getXAxis();
      this.yAxis = xyChart.getYAxis();

      this.axisSelectionRectangle = new AreaLimits();

      this.enableSelectionRectangle.addListener((observable, oldValue, newValue) -> setEnableSelectionRectangle(newValue));
      this.selectionContinouslyTracked.addListener((observable, oldValue, newValue) -> setContinuousTrackingOfSelection(newValue));

      parentContainer.addChartGraphicsLayer(this);
   }

   private void setEnableSelectionRectangle(boolean isEnabled)
   {
      if (isEnabled)
      {
         this.xyChart.addEventHandler(MouseEvent.MOUSE_PRESSED, startDrawingSelectionRectangle);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_DRAGGED, adjustSelectionRectangleSize);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_RELEASED, hideSelectionRectangle);
      }
      else
      {
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_PRESSED, startDrawingSelectionRectangle);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_DRAGGED, adjustSelectionRectangleSize);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_RELEASED, hideSelectionRectangle);
      }
   }

   public void setContinuousTrackingOfSelection(boolean isTracking)
   {
      selectionContinouslyTracked.set(isTracking);
   }

   /**
    *   Mouse Events Handling
    */

   private AreaLimits drawingArea;
   private AreaLimits selectedArea;

   private EventHandler<MouseEvent> startDrawingSelectionRectangle = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;

         drawingArea = getPlotAreaLimits();
         double x = MathTools.clipToMinMax(event.getX(), drawingArea.xmin, drawingArea.xmax);
         double y = MathTools.clipToMinMax(event.getY(), drawingArea.ymin, drawingArea.ymax);

         // Update Graphics
         rectangle.setX(x);
         rectangle.setY(y);
         rectangle.setWidth(0);
         rectangle.setHeight(0);
         rectangle.setVisible(true);

         // Update values

         double xMinOnAxis = x - drawingArea.xmin;
         double yMaxOnAxis = y - drawingArea.ymin; // Max on data chart Min in graphics coordinates

         selectedArea = new AreaLimits();
         selectedArea.xmin = xMinOnAxis;
         selectedArea.ymin = yMaxOnAxis;

         axisSelectionRectangle.xmin = xAxis.getValueForDisplay(xMinOnAxis).doubleValue();
         axisSelectionRectangle.ymax = yAxis.getValueForDisplay(yMaxOnAxis).doubleValue();
      }
   };

   private EventHandler<MouseEvent> adjustSelectionRectangleSize = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;

         double x = MathTools.clipToMinMax(event.getX(), drawingArea.xmin, drawingArea.xmax);
         double y = MathTools.clipToMinMax(event.getY(), drawingArea.ymin, drawingArea.ymax);

         isSelecting.set(true);

         double w = x - rectangle.getX();
         double h = y - rectangle.getY();

         rectangle.setWidth(w);
         rectangle.setHeight(h);

         if (w > 0 && h > 0)
         {

            double xMinOnChart = x - drawingArea.xmin;
            double yMinOnChart = y - drawingArea.ymin;

            selectedArea.xmax = xMinOnChart;
            selectedArea.ymax = yMinOnChart;
            axisSelectionRectangle.xmax = xAxis.getValueForDisplay(xMinOnChart).doubleValue();
            axisSelectionRectangle.ymin = yAxis.getValueForDisplay(yMinOnChart).doubleValue();

            // TODO MAY NEED TO CHANGE THIS NOTIFICATION SYSTEM, THE EVENT IS FIRED EVENT IF THE OBJECT HAS THE SAME VALUES, look at observable list for AreaLimits
            if (selectionContinouslyTracked.get() && selectedArea.getWidth() > 0 && selectedArea.getHeight() > 0)
            {
               updateAxisSelectionRectangleProperty();
            }
         }
      }
   };

   private EventHandler<MouseEvent> hideSelectionRectangle = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;

         System.out.println("Ended Selection rectangle Display");

         double x = MathTools.clipToMinMax(event.getX(), drawingArea.xmin, drawingArea.xmax);
         double y = MathTools.clipToMinMax(event.getY(), drawingArea.ymin, drawingArea.ymax);

         double xMinOnChart = x - drawingArea.xmin;
         double yMinOnChart = y - drawingArea.ymin;

         selectedArea.xmax = xMinOnChart;
         selectedArea.ymax = yMinOnChart;
         axisSelectionRectangle.xmax = xAxis.getValueForDisplay(xMinOnChart).doubleValue();
         axisSelectionRectangle.ymin = yAxis.getValueForDisplay(yMinOnChart).doubleValue();

         if (isSelecting.get() && selectedArea.getWidth() > 0 && selectedArea.getHeight() > 0)
         {
            updateAxisSelectionRectangleProperty();
         }

         rectangle.setVisible(false);
         drawingArea = null;
         selectedArea = null;
         isSelecting.set(false);
      }
   };

   private void updateAxisSelectionRectangleProperty() // Use a property with object instead
   {

      AreaLimits areaLimits = new AreaLimits(axisSelectionRectangle);

      double widthRatio = selectedArea.getWidth() / drawingArea.getWidth();
      double heightRatio = selectedArea.getHeight() / drawingArea.getHeight();

      SelectionRectangle selectionRectangle = new SelectionRectangle(areaLimits, widthRatio, heightRatio);

      this.selectionRectangle.set(selectionRectangle);
   }

   public SimpleObjectProperty<SelectionRectangle> selectionRectangleProperty()
   {
      return selectionRectangle;
   }

   public SimpleBooleanProperty enableSelectionRectangleProperty()
   {
      return enableSelectionRectangle;
   }

   public SimpleBooleanProperty selectionContinouslyTrackedProperty()
   {
      return selectionContinouslyTracked;
   }

   public SimpleBooleanProperty isSelectingProperty()
   {
      return isSelecting;
   }
}
