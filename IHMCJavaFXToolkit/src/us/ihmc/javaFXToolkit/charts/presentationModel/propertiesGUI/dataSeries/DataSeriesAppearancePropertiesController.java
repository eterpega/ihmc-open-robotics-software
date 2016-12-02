package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.dataSeries;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.*;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import us.ihmc.javaFXToolkit.charts.presentationModel.DataSeriesAppearanceManager;

import java.net.URL;
import java.util.ResourceBundle;

/**
 *
 * // TODO Figure out a different way to set the style of the data
 *
 *
 * Created by amoucheboeuf on 6/8/16.
 */
public class DataSeriesAppearancePropertiesController implements Initializable
{
   @FXML private ComboBox lineWidth;
   @FXML private ColorPicker lineColor;
   @FXML private ComboBox lineStyle;
   @FXML private ComboBox lineSymbol;
   @FXML private ChoiceBox dataSeriesChoice;

   private ObservableList<ImageView> lineWidthSelection;
   private ObservableList<ImageView> lineStyleSelection;
   private ObservableList<String> dataSeriesNames;



   private DataSeriesAppearanceManager dataSeriesAppearanceManager;


   private static final String[] strokeDashValues = { "1", "10 10", "5 7", "1 7" };

   DataSeriesAppearancePropertiesController(DataSeriesAppearanceManager dataSeriesAppearanceManager)
   {
      this.dataSeriesAppearanceManager = dataSeriesAppearanceManager;

      dataSeriesNames =  dataSeriesAppearanceManager.getDataSeriesNames();

      lineWidthSelection = FXCollections.observableArrayList();
      lineWidthSelection.add(new ImageView("img/line-width-very-bold.png"));
      lineWidthSelection.add(new ImageView("img/line-width-bold.png"));
      lineWidthSelection.add(new ImageView("img/line-width-semi-bold.png"));
      lineWidthSelection.add(new ImageView("img/line-width-normal.png"));
      lineWidthSelection.add(new ImageView("img/line-width-thin.png"));
      lineWidthSelection.add(new ImageView("img/line-width-very-thin.png"));

      lineStyleSelection = FXCollections.observableArrayList();

      lineStyleSelection.add(new ImageView("img/line-width-normal.png"));
      lineStyleSelection.add(new ImageView("img/dashed-line-big.png"));
      lineStyleSelection.add(new ImageView("img/dashed-line-medium.png"));
      lineStyleSelection.add(new ImageView("img/dashed-line-small.png"));
   }

   @Override public void initialize(URL location, ResourceBundle resources)
   {

      assert lineWidth != null : "fx:id=\"lineWidth\" was not injected: check your FXML file 'DataSeriesProperties.fxml'.";
      assert lineColor != null : "fx:id=\"lineColor\" was not injected: check your FXML file 'DataSeriesProperties.fxml'.";
      assert lineStyle != null : "fx:id=\"lineStyle\" was not injected: check your FXML file 'DataSeriesProperties.fxml'.";
      assert lineSymbol != null : "fx:id=\"lineSymbol\" was not injected: check your FXML file 'DataSeriesProperties.fxml'.";
      assert dataSeriesChoice != null : "fx:id=\"dataSeriesChoice\" was not injected: check your FXML file 'DataSeriesProperties.fxml'.";

      dataSeriesChoice.setItems(dataSeriesNames);

//      dataSeriesChoice.getSelectionModel().selectedIndexProperty().addListener((observable1, oldValue1, newValue) -> System.out.println("Selected value on data series choice: "+ newValue));

      dataSeriesAppearanceManager.currentDataSeriesIndexProperty().bind(dataSeriesChoice.getSelectionModel().selectedIndexProperty());


      lineWidth.setCellFactory(list -> new ImageViewCell());

      lineWidth.setItems(lineWidthSelection);

      lineWidth.getSelectionModel().selectedIndexProperty().addListener((observable, oldValue, selectedIndex) -> {
         int newLineWidth = lineWidthSelection.size() - selectedIndex.intValue();
         dataSeriesAppearanceManager.lineWidthProperty().set(newLineWidth);
      });

      lineStyle.setCellFactory(list -> new ImageViewCell());
      lineStyle.setItems(lineStyleSelection);
      lineStyle.getSelectionModel().selectedIndexProperty().addListener((observable, oldValue, index) -> {
         dataSeriesAppearanceManager.lineDashProperty().set(strokeDashValues[index.intValue()]);
      });

      lineColor.setOnAction(event -> {
         dataSeriesAppearanceManager.lineColorProperty().set(lineColor.getValue().toString().replace("0x", "#"));
      });

      dataSeriesChoice.getSelectionModel().selectFirst();
   }

   private class ImageViewCell extends ListCell<ImageView>
   {
      ImageViewCell()
      {
         setContentDisplay(ContentDisplay.GRAPHIC_ONLY);
      }

      @Override protected void updateItem(ImageView item, boolean empty)
      {
         super.updateItem(item, empty);
         if (item == null || empty)
         {
            setGraphic(null);
         }
         else
         {
            Image image = item.getImage();
            setGraphic(new ImageView(image));
         }
      }
   }

}
