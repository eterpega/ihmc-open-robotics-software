package us.ihmc.tuner.slider;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

/**
 * Displays that a parameter slider box is currently empty.
 */
public class EmptyParameterSlider extends VBox
{
   private static final String FXML_PATH = "empty_param_slider.fxml";

   private Parameter parameter;

   public EmptyParameterSlider() throws IOException
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      loader.load();
   }

   public void initialize()
   {
   }
}
