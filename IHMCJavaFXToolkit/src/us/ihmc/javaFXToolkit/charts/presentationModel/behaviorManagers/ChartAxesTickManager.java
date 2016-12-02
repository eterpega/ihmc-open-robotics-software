package us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.chart.NumberAxis;
import javafx.util.StringConverter;

/**
 * Class responsible for formatting the chart axes tick values to be human readable
 *
 * Created by amoucheboeuf on 7/21/16.
 */
public class ChartAxesTickManager
{
   private final String xAxisInitialLabel;
   private final String yAxisInitialLabel;

   private final NumberAxis xAxis;
   private final NumberAxis yAxis;

   private final SimpleDoubleProperty xAxisVisibleRange = new SimpleDoubleProperty();
   private final SimpleIntegerProperty xAxisTickUnitScaler = new SimpleIntegerProperty();

   private int lastScaleMultiplier = -99;

   /**
    *
    * first role: automatically modify tick unit intervals depending on zoom level
    * TODO second role: provide option for the user to modify the tick label formatting
    *
    * @param xAxis
    * @param yAxis
    */
   public ChartAxesTickManager(NumberAxis xAxis, NumberAxis yAxis)
   {
      this.xAxis = xAxis;
      this.yAxis = yAxis;

      xAxisInitialLabel = xAxis.getLabel();
      yAxisInitialLabel = yAxis.getLabel();

      // compute difference between the chart's lower and upper bounds
      this.xAxisVisibleRange.bind(xAxis.upperBoundProperty().subtract(xAxis.lowerBoundProperty()));

      this.xAxisVisibleRange.addListener((observable, oldValue, newValue) -> {

         int scaleMultiplier = (int) Math.log10(newValue.doubleValue());

         System.out
               .println(" xAxisVisibleRange " + xAxisVisibleRange + " tick unit log: " + scaleMultiplier + " raw value " + Math.log10(newValue.doubleValue()));

         if (scaleMultiplier == 0)
         {
            xAxis.tickUnitProperty().set(2);
         }
         else if (scaleMultiplier == 1)
         {
            xAxis.tickUnitProperty().set(5);
         }
         else
         {
            xAxis.tickUnitProperty().set(Math.pow(10, scaleMultiplier));
         }

//         if (lastScaleMultiplier != scaleMultiplier)
//         {
//            this.xAxis.setTickLabelFormatter(new StringConverter<Number>()
//            {
//               @Override public String toString(Number object)
//               {
//                  return String.format(". ", object.intValue());
//               }
//
//               @Override public Number fromString(String string)
//               {
//                  return 0;
//               }
//            });
//            lastScaleMultiplier = scaleMultiplier;
//         }

      });

      this.xAxis.upperBoundProperty().addListener(upperBoundChangeListener);
      this.xAxis.tickUnitProperty().addListener((observable, oldValue, newValue) -> {
         xAxis.setLabel(xAxisInitialLabel + " (" + newValue + ")");
      });

   }

   ChangeListener<Number> upperBoundChangeListener = (observable, oldValue, newValue) -> {

   };

}
