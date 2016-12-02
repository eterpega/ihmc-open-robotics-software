package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.CheckBox;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.ZoomManager;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 5/27/16.
 */
public class ZoomManagerPropertiesController implements Initializable
{
   @FXML private CheckBox scrollZoomH, scrollZoomV;
   @FXML private CheckBox selectRectH, selectRectV;
   @FXML private CheckBox autoRangeMiddleMouse;

   private final ZoomManager chartZoomManager;

   public ZoomManagerPropertiesController(ZoomManager chartZoomManager)
   {
      this.chartZoomManager = chartZoomManager;
   }

   @Override public void initialize(URL location, ResourceBundle resources)
   {
      assert scrollZoomH != null : "fx:id=\"scrollZoomH\" was not injected: check your FXML file 'ZoomManagerProperties.fxml'.";
      assert scrollZoomV != null : "fx:id=\"scrollZoomV\" was not injected: check your FXML file 'ZoomManagerProperties.fxml'.";

      assert selectRectH != null : "fx:id=\"selectRectH\" was not injected: check your FXML file 'ZoomManagerProperties.fxml'.";
      assert selectRectV != null : "fx:id=\"selectRectV\" was not injected: check your FXML file 'ZoomManagerProperties.fxml'.";

      assert autoRangeMiddleMouse != null : "fx:id=\"autoRangeMiddleMouse\" was not injected: check your FXML file 'ZoomManagerProperties.fxml'.";

      scrollZoomH.setSelected(chartZoomManager.enableHorizontalZoomOnScrollProperty().get());
      scrollZoomV.setSelected(chartZoomManager.enableVerticalZoomOnScrollProperty().get());
      selectRectH.setSelected(chartZoomManager.enableHorizontalSelectionZoomProperty().get());
      selectRectV.setSelected(chartZoomManager.enableVerticalSelectionZoomProperty().get());
      autoRangeMiddleMouse.setSelected(chartZoomManager.enableAutoRangeOnMouseMiddleClickProperty().get());

      chartZoomManager.enableHorizontalZoomOnScrollProperty().bindBidirectional(scrollZoomH.selectedProperty());
      chartZoomManager.enableVerticalZoomOnScrollProperty().bindBidirectional(scrollZoomV.selectedProperty());
      chartZoomManager.enableHorizontalSelectionZoomProperty().bindBidirectional(selectRectH.selectedProperty());
      chartZoomManager.enableVerticalSelectionZoomProperty().bindBidirectional(selectRectV.selectedProperty());
      chartZoomManager.enableAutoRangeOnMouseMiddleClickProperty().bindBidirectional(autoRangeMiddleMouse.selectedProperty());
   }

}
