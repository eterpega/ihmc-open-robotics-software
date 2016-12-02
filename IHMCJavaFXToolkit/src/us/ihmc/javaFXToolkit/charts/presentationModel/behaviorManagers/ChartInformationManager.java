package us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.chart.XYChart;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class ChartInformationManager
{

   private double xAxisPreferredHeight, yAxisPreferredWidth;
   private String xAxisLabelValue, yAxisLabelValue;
   private String title;

   private XYChart chart;

   private SimpleBooleanProperty titleVisible = new SimpleBooleanProperty();
   private SimpleBooleanProperty legendVisible = new SimpleBooleanProperty();
   private SimpleBooleanProperty xAxisVisible = new SimpleBooleanProperty();
   private SimpleBooleanProperty xAxisLabelVisible = new SimpleBooleanProperty();
   private SimpleBooleanProperty yAxisVisible = new SimpleBooleanProperty();
   private SimpleBooleanProperty yAxisLabelVisible = new SimpleBooleanProperty();

   public ChartInformationManager(XYChart chart)
   {

      this.chart = chart;
      this.title = this.chart.getTitle();
      this.xAxisPreferredHeight = this.chart.getXAxis().getPrefHeight();
      this.yAxisPreferredWidth = this.chart.getYAxis().getPrefWidth();
      this.xAxisLabelValue = this.chart.getXAxis().getLabel();
      this.yAxisLabelValue = this.chart.getYAxis().getLabel();

      titleVisible.addListener(titleVisibleChangeListener);
      legendVisible.addListener(legendVisibleChangeListener);

      xAxisVisible.addListener(xAxisVisibleChangeListener);
      xAxisLabelVisible.addListener(xAxisLabelVisibleChangeListener);

      yAxisVisible.addListener(yAxisVisibleChangeListener);
      yAxisLabelVisible.addListener(yAxisLabelVisibleChangeListener);

   }

   public SimpleBooleanProperty titleVisibleProperty()
   {
      return titleVisible;
   }

   public SimpleBooleanProperty legendVisibleProperty()
   {
      return legendVisible;
   }

   public SimpleBooleanProperty xAxisVisibleProperty()
   {
      return xAxisVisible;
   }

   public SimpleBooleanProperty xAxisLabelVisibleProperty()
   {
      return xAxisLabelVisible;
   }

   public SimpleBooleanProperty yAxisVisibleProperty()
   {
      return yAxisVisible;
   }

   public SimpleBooleanProperty yAxisLabelVisibleProperty()
   {
      return yAxisLabelVisible;
   }

   // hide/show legend

   private ChangeListener<Boolean> legendVisibleChangeListener = (observable, oldValue, newValue) -> {
      chart.legendVisibleProperty().set(newValue);
   };

   // hide show X axis values
   private ChangeListener<Boolean> xAxisVisibleChangeListener = (observable, oldValue, isEnabled) -> {

      if (isEnabled)
      {
         chart.getXAxis().setOpacity(1);
         chart.getXAxis().setPrefHeight(xAxisPreferredHeight);
      }
      else
      {
         chart.getXAxis().setOpacity(0);
         chart.getXAxis().setPrefHeight(0);
      }
   };

   private ChangeListener<Boolean> xAxisLabelVisibleChangeListener = (observable, oldValue, isEnabled) -> {
      if (isEnabled)
      {
         chart.getXAxis().setLabel(xAxisLabelValue);
      }
      else
      {
         chart.getXAxis().setLabel("");
      }
   };

   // hide show Y axis values
   private ChangeListener<Boolean> yAxisVisibleChangeListener = (observable, oldValue, isEnabled) -> {

      if (isEnabled)
      {
         chart.getYAxis().setOpacity(1);
         chart.getYAxis().setPrefWidth(yAxisPreferredWidth);
      }
      else
      {
         chart.getYAxis().setOpacity(0);
         chart.getYAxis().setPrefWidth(0);
      }
   };

   private ChangeListener<Boolean> yAxisLabelVisibleChangeListener = (observable, oldValue, isEnabled) -> {
      if (isEnabled)
      {
         chart.getYAxis().setLabel(yAxisLabelValue);
      }
      else
      {
         chart.getYAxis().setLabel("");
      }
   };

   // hide/ show chart title
   private ChangeListener<Boolean> titleVisibleChangeListener = (observable, oldValue, isEnabled) -> {
      if (isEnabled)
      {
         chart.setTitle(title);
      }
      else
      {
         chart.setTitle(null);
      }
   };

}

