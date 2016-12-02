package us.ihmc.javaFXToolkit.charts.presentationModel;

import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.XYChart;

import java.util.stream.Collectors;

/**
 * Created by amoucheboeuf on 6/14/16.
 */
public class DataSeriesAppearanceManager
{

   private final LineChart chart;

   private SimpleObjectProperty<XYChart.Series> currentDataSeries = new SimpleObjectProperty<>();
   private SimpleIntegerProperty currentDataSeriesIndex = new SimpleIntegerProperty();
   private SimpleIntegerProperty lineWidth = new SimpleIntegerProperty();
   private SimpleStringProperty lineColor = new SimpleStringProperty();
   private SimpleStringProperty lineDash = new SimpleStringProperty();

   public DataSeriesAppearanceManager(LineChart chart)
   {
      this.chart = chart;

      lineColor.addListener(lineColorChangeListener);
      lineDash.addListener(lineDashChangeListener);
      lineWidth.addListener(lineWidthChangeListener);

      currentDataSeriesIndex.addListener(currentDataSeriesChangeListener);

   }

   public ObservableList<String> getDataSeriesNames()
   {
      ObservableList<XYChart.Series> dataSeriesObservableList = chart.getData();
      return FXCollections.observableList(dataSeriesObservableList.stream().map(XYChart.Series::getName).collect(Collectors.toList()));
   }

   public SimpleObjectProperty<XYChart.Series> currentDataSeriesProperty()
   {
      return currentDataSeries;
   }

   public SimpleIntegerProperty currentDataSeriesIndexProperty()
   {
      return currentDataSeriesIndex;
   }

   private final ChangeListener<Number> currentDataSeriesChangeListener = new ChangeListener<Number>()
   {
      @Override public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue)
      {
         if(newValue.intValue() < 0) return;

         XYChart.Series series = (XYChart.Series) chart.getData().get(newValue.intValue());
         currentDataSeries.set(series);
      }
   };

   private final ChangeListener<Number> lineWidthChangeListener = (observable, oldValue, newValue) -> currentDataSeries.get().getNode()
         .setStyle("-fx-stroke-width: " + newValue.toString() + ";");

   private final ChangeListener<String> lineColorChangeListener = (observable, oldValue, newColorCode) -> currentDataSeries.get().getNode()
         .setStyle("-fx-stroke:" + newColorCode + ";");

   private final ChangeListener<String> lineDashChangeListener = (observable, oldValue, newDashLineValues) -> currentDataSeries.get().getNode()
         .setStyle("-fx-stroke-dash-array:" + newDashLineValues + ";");

   public SimpleIntegerProperty lineWidthProperty()
   {
      return lineWidth;
   }

   public SimpleStringProperty lineColorProperty()
   {
      return lineColor;
   }

   public SimpleStringProperty lineDashProperty()
   {
      return lineDash;
   }
}
