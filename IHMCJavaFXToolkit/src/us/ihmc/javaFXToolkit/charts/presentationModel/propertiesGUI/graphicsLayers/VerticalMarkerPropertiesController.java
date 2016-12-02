package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers;

import javafx.beans.property.SimpleStringProperty;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.VerticalMarkerGraphics;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class VerticalMarkerPropertiesController implements Initializable
{

   @FXML private CheckBox enableVerticalMarker;

   @FXML private Label xDataValue;

   private SimpleStringProperty xDataValueAsString = new SimpleStringProperty("...");

   private VerticalMarkerGraphics verticalMarkerGraphics;

   public VerticalMarkerPropertiesController(VerticalMarkerGraphics verticalMarkerGraphics)
   {
      this.verticalMarkerGraphics = verticalMarkerGraphics;
   }

   @Override public void initialize(URL location, ResourceBundle resources)
   {

      assert enableVerticalMarker != null : "fx:id=\"enableVerticalMarker\" was not injected: check your FXML file 'VerticalMarkerProperties.fxml'.";
      assert xDataValue != null : "fx:id=\"xDataValue\" was not injected: check your FXML file 'VerticalMarkerProperties.fxml'.";
      this.verticalMarkerGraphics.verticalMarkerGraphicsEnabledProperty().bind(enableVerticalMarker.selectedProperty());


      this.xDataValue.textProperty().bind(this.verticalMarkerGraphics.markerPositionProperty().asString());
   }

}
