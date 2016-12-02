package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.CheckBox;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.PanManager;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 5/27/16.
 */
public class PanManagerPropertiesController implements Initializable
{

   @FXML private CheckBox horizontalPanningCheckBox;
   @FXML private CheckBox verticalPanningCheckBox;

   private final PanManager chartPanManager;

   public PanManagerPropertiesController(PanManager chartPanManager)
   {
      this.chartPanManager = chartPanManager;
   }


   @Override public void initialize(URL location, ResourceBundle resources)
   {
      assert horizontalPanningCheckBox != null : "fx:id=\"horizontalPanningCheckBox\" was not injected: check your FXML file 'mainApp.fxml'.";
      assert verticalPanningCheckBox != null : "fx:id=\"verticalPanningCheckBox\" was not injected: check your FXML file 'mainApp.fxml'.";

      chartPanManager.enableHorizontalPanningProperty().bindBidirectional(horizontalPanningCheckBox.selectedProperty());
      chartPanManager.enableVerticalPanningProperty().bindBidirectional(verticalPanningCheckBox.selectedProperty());
   }

}
