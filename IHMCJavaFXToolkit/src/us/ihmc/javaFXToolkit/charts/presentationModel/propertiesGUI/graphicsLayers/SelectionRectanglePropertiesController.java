package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.CheckBox;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.SelectionRectangleGraphics;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class SelectionRectanglePropertiesController implements Initializable
{

   private final SelectionRectangleGraphics selectionRectangleGraphics;

   @FXML
   CheckBox enableSelectionRectangle;

   SelectionRectanglePropertiesController(SelectionRectangleGraphics selectionRectangleGraphics)
   {
      this.selectionRectangleGraphics = selectionRectangleGraphics;
   }


   @Override public void initialize(URL location, ResourceBundle resources)
   {
      assert enableSelectionRectangle != null: "fx:id=\"enableSelectionRectangle\" was not injected: check your FXML file 'SelectionRectangleProperties.fxml'.";

      selectionRectangleGraphics.enableSelectionRectangleProperty().bind(enableSelectionRectangle.selectedProperty());
   }
}
