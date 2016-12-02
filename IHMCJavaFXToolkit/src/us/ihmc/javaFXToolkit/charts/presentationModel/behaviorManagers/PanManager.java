package us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers;

import javafx.beans.property.SimpleBooleanProperty;

import javafx.event.EventHandler;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.robotics.MathTools;

/**
 * Created by amoucheboeuf on 5/13/16.
 */
public class PanManager
{

   private final SimpleBooleanProperty enableHorizontalPanning = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableVerticalPanning = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableChartPanning = new SimpleBooleanProperty(false);
   private final SimpleBooleanProperty enableChartAutoPanning = new SimpleBooleanProperty(false);

   private final XYChart xyChart;
   private final NumberAxis xAxis, yAxis;

   private double xMin;
   private double xMax;
   private double yMin;
   private double yMax;

   private double xMouseStart, yMouseStart;
   private double chartDimensionDataRatio;

   public PanManager(XYChart xyChart, double xMin, double xMax, double yMin, double yMax)
   {
      this.xyChart = xyChart;
      this.xAxis = (NumberAxis) xyChart.getXAxis();
      this.yAxis = (NumberAxis) xyChart.getYAxis();
      this.setPanRegionLimits(xMin, xMax, yMin, yMax);
      this.enableChartPanning.bind(enableHorizontalPanning.or(enableVerticalPanning));
      this.enableChartPanning.addListener((observable, oldValue, newValue) -> {
         setChartPanningEnabled(newValue);
      });

      setChartPanningEnabled(true);
   }

   public void setPanRegionLimits(double xMin, double xMax, double yMin, double yMax)
   {
      MathTools.checkIfGreaterOrEqual(xMax, xMin);
      MathTools.checkIfGreaterOrEqual(yMax, yMin);
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
   }

   public SimpleBooleanProperty enableHorizontalPanningProperty()
   {
      return enableHorizontalPanning;
   }

   public SimpleBooleanProperty enableVerticalPanningProperty()
   {
      return enableVerticalPanning;
   }

   private void setChartPanningEnabled(boolean enabled)
   {
      if (enabled)
      {
         this.xyChart.addEventHandler(MouseEvent.MOUSE_PRESSED, startPanningChart);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_DRAGGED, doPanningChart);
         this.xyChart.addEventHandler(MouseEvent.MOUSE_RELEASED, stopPanningChart);
      }
      else
      {
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_PRESSED, startPanningChart);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_DRAGGED, doPanningChart);
         this.xyChart.removeEventHandler(MouseEvent.MOUSE_RELEASED, stopPanningChart);
      }
   }

   private final EventHandler<MouseEvent> startPanningChart = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.SECONDARY)
            return;

         xMouseStart = event.getX();
         yMouseStart = event.getY();
      }
   };

   private final EventHandler<MouseEvent> doPanningChart = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.SECONDARY)
            return;

         double chartDimensionRange = xyChart.getWidth();

         if (enableHorizontalPanning.get())
         {
            double chartDataRange = xAxis.getUpperBound() - xAxis.getLowerBound();
            chartDimensionDataRatio = chartDataRange / chartDimensionRange;

            double dx = xMouseStart - event.getX();
            xMouseStart = event.getX();


            double dxBounds = chartDimensionDataRatio * dx;

            double lb = xAxis.getLowerBound() + dxBounds;
            double ub = xAxis.getUpperBound() + dxBounds;

            if (lb < xMin)
            {
               lb = xMin;
               ub = xAxis.getUpperBound();
            }

            if (ub > xMax)
            {
               lb = xAxis.getLowerBound();
               ub = xMax;
            }

            //            System.out.println("mouse event! panning X lb: "+ lb + " ub: "+ ub);
            xAxis.setLowerBound(lb);
            xAxis.setUpperBound(ub);
         }

         if (enableVerticalPanning.get())
         {
            double chartDataRange = yAxis.getUpperBound() - yAxis.getLowerBound();
            chartDimensionDataRatio = chartDataRange / chartDimensionRange;

            double dy = yMouseStart - event.getY();
            yMouseStart = event.getY();

            double dyBounds = -1 * chartDimensionDataRatio * dy;

            double lb = yAxis.getLowerBound() + dyBounds;
            double ub = yAxis.getUpperBound() + dyBounds;

            if (lb < yMin)
            {
               lb = yMin;
               ub = yAxis.getUpperBound();
            }

            if (ub > yMax)
            {
               lb = yAxis.getLowerBound();
               ub = yMax;
            }

            yAxis.setLowerBound(lb);
            yAxis.setUpperBound(ub);
         }

      }
   };

   private final EventHandler<MouseEvent> stopPanningChart = new EventHandler<MouseEvent>()
   {
      @Override public void handle(MouseEvent event)
      {
         if (event.getButton() != MouseButton.SECONDARY)
            return;
         xMouseStart = 0;
         yMouseStart = 0;
      }
   };
}
