package us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.AreaLimits;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.SelectionRectangle;

/**
 *
 *
 * Created by amoucheboeuf on 5/13/16.
 */
public class ZoomManager
{
   private static final double MIN_ZOOM_SELECTION_RATIO_X = 0.05;
   private static final double MIN_ZOOM_SELECTION_RATIO_Y = 0.05;
   private double zoomStep = 0.2;

   private final SimpleBooleanProperty selectionRectangleZoomEnabled = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableHorizontalSelectionZoom = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableVerticalSelectionZoom = new SimpleBooleanProperty(false);

   private final SimpleBooleanProperty enableAutoRangeOnMouseMiddleClick = new SimpleBooleanProperty(false);

   private final SimpleBooleanProperty enableZoomOnScroll = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableHorizontalZoomOnScroll = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableVerticalZoomOnScroll = new SimpleBooleanProperty(false);

   private final XYChart chart;
   private final NumberAxis xAxis;
   private final NumberAxis yAxis;
   private final Node plotArea;
   private boolean autoRange = false;

   /**
    * TODO: Documentation
    * Class in charge of handling zoom behaviors for the chart passed as parameter. <br>
    * Zooming behavior include: <br>
    * <ul>
    *    <li>Middle mouse button scroll for zoom in/ zoom out </li>
    *    <li>Rectangle Selection</li>
    *    <li>Auto Range on mouse's middle button click</li>
    * </ul>
    * @param chart
    */
   public ZoomManager(XYChart chart)
   {
      this.chart = chart;
      this.plotArea = chart.lookup(".chart-plot-background");
      this.xAxis = (NumberAxis) chart.getXAxis();
      this.yAxis = (NumberAxis) chart.getYAxis();

      selectionRectangleZoomEnabled.bind(enableHorizontalSelectionZoom.or(enableVerticalSelectionZoom));

      enableAutoRangeOnMouseMiddleClick.addListener((observable, oldValue, newValue) -> setAutoRangeOnMiddleMouseButtonClick(newValue));

      enableZoomOnScroll.bind(enableHorizontalZoomOnScroll.or(enableVerticalZoomOnScroll));
      enableZoomOnScroll.addListener((observable, oldValue, newValue) -> setEnableZoomOnScroll(newValue));
   }


   public void enableAllBehaviors(boolean enabled) //TODO: TO BE REMOVED. Remove once GUI persistence mechanism is implemented
   {
      enableHorizontalSelectionZoom.set(enabled);
      enableVerticalSelectionZoom.set(enabled);
      enableAutoRangeOnMouseMiddleClick.set(enabled);
      enableHorizontalZoomOnScroll.set(enabled);
      enableVerticalZoomOnScroll.set(enabled);
   }


   /**
    * Enabled by default
    * @param isEnabled
    */
   private void setAutoRangeOnMiddleMouseButtonClick(boolean isEnabled)
   {
      if (isEnabled)
      {
         this.chart.addEventHandler(MouseEvent.MOUSE_CLICKED, startAutorangeOnMouseMiddleClick);
      }
      else
      {
         this.chart.removeEventHandler(MouseEvent.MOUSE_CLICKED, startAutorangeOnMouseMiddleClick);
      }
   }

   /**
    * Enabled by default
    * @param isEnabled
    */
   private void setEnableZoomOnScroll(boolean isEnabled)
   {
//      System.out.println("Set Enable zoom on scroll changed " + isEnabled);
      if (isEnabled)
      {
         this.chart.addEventHandler(ScrollEvent.SCROLL, doZoomOnScroll);
      }
      else
      {
         this.chart.removeEventHandler(ScrollEvent.SCROLL, doZoomOnScroll);
      }

   }

   private EventHandler<MouseEvent> startAutorangeOnMouseMiddleClick = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() == MouseButton.MIDDLE && enableAutoRangeOnMouseMiddleClick.get())
         {
            autoRange = true;
            xAxis.setAutoRanging(autoRange);
            yAxis.setAutoRanging(autoRange);
            chart.addEventHandler(MouseEvent.MOUSE_MOVED, autoZoomCanceller);
         }
      }
   };

   private EventHandler autoZoomCanceller = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (autoRange == true && enableAutoRangeOnMouseMiddleClick.get())
         {
            autoRange = false;
            xAxis.setAutoRanging(autoRange);
            yAxis.setAutoRanging(autoRange);
            chart.removeEventHandler(MouseEvent.MOUSE_MOVED, this);
         }
      }
   };

   private EventHandler<ScrollEvent> doZoomOnScroll = new EventHandler<ScrollEvent>()
   {

      @Override public void handle(ScrollEvent scrollEvent)
      {

         double xMouseOnPlot = scrollEvent.getSceneX() - plotArea.getLayoutX();
         double xAxisData = xAxis.getValueForDisplay(xMouseOnPlot).doubleValue();

         double yMouseOnPlot = scrollEvent.getSceneY() - plotArea.getLayoutY();
         double yAxisData = yAxis.getValueForDisplay(yMouseOnPlot).doubleValue();

         double xZoomNmz = getNormalizedValue(xAxisData, xAxis.getLowerBound(), xAxis.getUpperBound());
         double yZoomNmz = getNormalizedValue(yAxisData, yAxis.getLowerBound(), yAxis.getUpperBound());

         double direction = -Math.signum(scrollEvent.getDeltaY());

         double zoomAmount = zoomStep * direction;

         if (enableHorizontalZoomOnScroll.get())
         {
            double xZoomDelta = (xAxis.getUpperBound() - xAxis.getLowerBound()) * zoomAmount;
            xAxis.setAutoRanging(false);
            xAxis.setLowerBound(xAxis.getLowerBound() - xZoomDelta * xZoomNmz);
            xAxis.setUpperBound(xAxis.getUpperBound() + xZoomDelta * (1 - xZoomNmz));
         }

         if (enableVerticalZoomOnScroll.get())
         {
            double yZoomDelta = (yAxis.getUpperBound() - yAxis.getLowerBound()) * zoomAmount;
            yAxis.setAutoRanging(false);
            yAxis.setLowerBound(yAxis.getLowerBound() - yZoomDelta * yZoomNmz);
            yAxis.setUpperBound(yAxis.getUpperBound() + yZoomDelta * (1 - yZoomNmz));
         }

      }
   };

   private static double getNormalizedValue(double val, double min, double max)
   {
      if (val <= min)
         return 0.0;
      else if (val >= max)
         return 1.0;

      return (val - min) / (max - min);
   }

   public ChangeListener<SelectionRectangle> getSelectionRectangleChangeListener()
   {
      return selectionRectangleChangeListener;
   }

   private final ChangeListener<SelectionRectangle> selectionRectangleChangeListener = new ChangeListener<SelectionRectangle>()
   {
      @Override public void changed(ObservableValue<? extends SelectionRectangle> observable, SelectionRectangle oldValue, SelectionRectangle selectionRectangle)
      {
         if ((selectionRectangle.getWidthRatio() < MIN_ZOOM_SELECTION_RATIO_X || selectionRectangle.getHeightRatio() < MIN_ZOOM_SELECTION_RATIO_Y) && selectionRectangleZoomEnabled.get())
            return;

         AreaLimits newAxesBoundLimits = selectionRectangle.getAreaLimits();

        // xAxis.setAutoRanging(true);
         if (enableHorizontalSelectionZoom.get())
         {
            xAxis.setAutoRanging(false);
            xAxis.setLowerBound(newAxesBoundLimits.xmin);
            xAxis.setUpperBound(newAxesBoundLimits.xmax);
         }
         if (enableVerticalSelectionZoom.get())
         {
            yAxis.setAutoRanging(false);
            yAxis.setLowerBound(newAxesBoundLimits.ymin);
            yAxis.setUpperBound(newAxesBoundLimits.ymax);
         }
      }
   };

   public SimpleBooleanProperty enableHorizontalSelectionZoomProperty()
   {
      return enableHorizontalSelectionZoom;
   }

   public SimpleBooleanProperty enableVerticalSelectionZoomProperty()
   {
      return enableVerticalSelectionZoom;
   }

   public SimpleBooleanProperty enableAutoRangeOnMouseMiddleClickProperty()
   {
      return enableAutoRangeOnMouseMiddleClick;
   }

   public SimpleBooleanProperty enableHorizontalZoomOnScrollProperty()
   {
      return enableHorizontalZoomOnScroll;
   }

   public SimpleBooleanProperty enableVerticalZoomOnScrollProperty()
   {
      return enableVerticalZoomOnScroll;
   }

}
