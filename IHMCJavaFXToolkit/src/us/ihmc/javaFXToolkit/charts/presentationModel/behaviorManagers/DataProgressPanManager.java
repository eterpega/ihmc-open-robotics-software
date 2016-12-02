package us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers;

import javafx.application.Platform;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DataSeriesManagerChangeListener;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DynamicChart;
import us.ihmc.robotics.MathTools;

/**
 * TODO constrain types NumberAxism Type of chart used...
 * panning depends on latests data rendered on chart
 * Created by amoucheboeuf on 6/14/16.
 */
public class DataProgressPanManager
{
   private final SimpleIntegerProperty slidingWindowHorizontalSpan = new SimpleIntegerProperty();
   private final SimpleIntegerProperty slidingWindowVerticalSpan = new SimpleIntegerProperty();

   private final DynamicChart dynamicChart;

   private final XYChart xyChart;
   private final NumberAxis xAxis, yAxis;

   private final SimpleDoubleProperty slidingWindowSpanToBufferSizeRatio = new SimpleDoubleProperty();
   private double slidingWindowToBufferSizeRatio;

   private double lowerBoundLimit = 0;

   private double slidingWindowSpan;
   private double lastUpperBoundValue = 0;
   private double delta;

   private int countLimit = 10;
   private int counter = 0;


   /**
    * Look up the ten first data points to determine the delta between data points and the estimated lowest value of the data series
    * @param dynamicChart
    */
   public DataProgressPanManager(DynamicChart dynamicChart)
   {
      this.dynamicChart = dynamicChart;
      this.xyChart = dynamicChart.getLineChart();
      this.xAxis = (NumberAxis) xyChart.getXAxis();
      this.yAxis = (NumberAxis) xyChart.getYAxis();

      this.xAxis.setAutoRanging(false);
      this.yAxis.setAutoRanging(false);

      this.dynamicChart.getLineChart().setAxisSortingPolicy(LineChart.SortingPolicy.NONE);

      this.dynamicChart.allPointsUpperBoundXProperty().addListener((observable, oldValue, newValue) -> {
         estimateSlidingWindowPosition(newValue.intValue());
      });

      this.dynamicChart.allPointsUpperBoundYProperty().addListener((observable, oldValue, newValue) -> yAxis.setUpperBound(Math.round(newValue.doubleValue())));
      this.dynamicChart.allPointsLowerBoundYProperty().addListener((observable, oldValue, newValue) -> yAxis.setLowerBound(Math.round(newValue.doubleValue())));

      // throw exception if value is not within range
      this.slidingWindowSpanToBufferSizeRatio.addListener((observable, oldValue, newValue) -> setSlidingWindowToBufferSizeRatio(newValue.doubleValue()));

      this.slidingWindowSpanToBufferSizeRatio.set(1.0 / 50.0);

   }

   /**
    * @param dynamicChart
    * @param estimatedDelta estimated delta between two data points
    */
   public DataProgressPanManager(DynamicChart dynamicChart, double estimatedDelta, double estimatedLowerBoundLimit)
   {
      this.dynamicChart = dynamicChart;
      this.xyChart = dynamicChart.getLineChart();
      this.xAxis = (NumberAxis) xyChart.getXAxis();
      this.yAxis = (NumberAxis) xyChart.getYAxis();
      this.xAxis.setAutoRanging(false);
      this.yAxis.setAutoRanging(false);

      this.setDelta(estimatedDelta);
      this.lowerBoundLimit = estimatedLowerBoundLimit;

      this.slidingWindowHorizontalSpan.set((int) (estimatedDelta * dynamicChart.getChartBufferCapacity() * slidingWindowToBufferSizeRatio));

      this.setSlidingWindowPosition(lowerBoundLimit);

      this.dynamicChart.allPointsUpperBoundXProperty().addListener((observable, oldValue, newValue) -> {
         setSlidingWindowPosition(newValue.intValue());
      });
   }

   private void setDelta(double delta)
   {
      if (delta <= 0)
         throw new IllegalArgumentException(" delta cannot be negative nor null");

      this.delta = delta;
   }

   //   /**
   //    * works when the delta is constant
   //    * @param upperBoundX
   //    */
   private void estimateSlidingWindowPosition(double upperBoundX)
   {
      System.out.println("upperBound x " + upperBoundX);
      if (counter < countLimit)
      {
         setDelta((1.0 - 1.0 / countLimit) * delta + 1.0 / countLimit * Math.abs(lastUpperBoundValue - upperBoundX));
         slidingWindowSpan = delta * dynamicChart.getChartBufferCapacity() * slidingWindowToBufferSizeRatio;
         counter++;
      }

      double lowerBoundX = lastUpperBoundValue - slidingWindowSpan;

      if (lowerBoundX < lowerBoundLimit)
      {
         lowerBoundX = lowerBoundLimit;
      }

      xAxis.setLowerBound(lowerBoundX);

      if (upperBoundX < slidingWindowSpan)
      {
         xAxis.setUpperBound(slidingWindowSpan);
      }
      else
      {
         xAxis.setUpperBound(upperBoundX);
      }

      lastUpperBoundValue = upperBoundX;
   }


//   private double alpha = 0.7;
//   private double averageSlidingWindowSpan;
   //   /**
   //    * using a running average for the sliding window span
   //    * @param upperBoundX
   //    */
   //   private void estimateSlidingWindowPosition(double upperBoundX)
   //   {
   //
   //      setDelta((1.0 - 1.0 / countLimit) * delta + 1.0 / countLimit * Math.abs(lastUpperBoundValue - upperBoundX));
   //      slidingWindowSpan = (int) (delta * dynamicChart.getChartBufferCapacity() * slidingWindowSpanToBufferSizeRatio.get());
   //
   //      // initialize average value
   //      if (counter < countLimit)
   //      {
   //         averageSlidingWindowSpan = slidingWindowSpan;
   //         counter++;
   //      }
   //      else
   //      {
   //         averageSlidingWindowSpan = (int) alpha * averageSlidingWindowSpan + (1.0 - alpha) * slidingWindowSpan;
   //      }
   //      System.out.println("upperBound x " + upperBoundX + " sliding window span " + slidingWindowSpan + " average window span " + averageSlidingWindowSpan);
   //
   //      double lowerBoundX = lastUpperBoundValue - slidingWindowSpan;
   //
   //      if (lowerBoundX < lowerBoundLimit)
   //      {
   //         lowerBoundX = lowerBoundLimit;
   //      }
   //
   //      xAxis.setLowerBound(lowerBoundX);
   //
   //      if (upperBoundX < slidingWindowSpan)
   //      {
   //         xAxis.setUpperBound(slidingWindowSpan);
   //      }
   //      else
   //      {
   //         xAxis.setUpperBound(upperBoundX);
   //      }
   //
   //      lastUpperBoundValue = upperBoundX;
   //
   //   }

   private void setSlidingWindowPosition(double upperBoundX)
   {
      if (upperBoundX < xAxis.getLowerBound() + slidingWindowHorizontalSpan.get())
         return;

      xAxis.setLowerBound(upperBoundX - slidingWindowHorizontalSpan.get());
      xAxis.setUpperBound(upperBoundX);
   }

   public void reset()
   {
      if (counter == 0)
         return;

      this.counter = 0;
      this.xAxis.setLowerBound(lowerBoundLimit);
      this.xAxis.setUpperBound(lowerBoundLimit + delta * dynamicChart.getChartBufferCapacity() * slidingWindowSpanToBufferSizeRatio.get());
   }

   public SimpleDoubleProperty slidingWindowSpanToBufferSizeRatioProperty()
   {
      return slidingWindowSpanToBufferSizeRatio;
   }

   public double getSlidingWindowToBufferSizeRatio()
   {
      return slidingWindowToBufferSizeRatio;
   }

   public void setSlidingWindowToBufferSizeRatio(double slidingWindowToBufferSizeRatio)
   {
      MathTools.checkIfInRange(slidingWindowToBufferSizeRatio, 0, 1);
      this.slidingWindowToBufferSizeRatio = slidingWindowToBufferSizeRatio;
   }
}
