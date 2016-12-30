package us.ihmc.tuner.slider;

import java.io.IOException;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class BooleanParameterSlider extends VBox
{
   private static final String FXML_PATH = "boolean_param_slider.fxml";

   @FXML private Pane root;
   @FXML private CheckBox check;
   @FXML private Label nameLabel;

   private Parameter parameter;

   public BooleanParameterSlider() throws IOException
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      loader.load();
   }

   public void initialize()
   {
      setControlsEnabled(false);
   }

   public void setValue(boolean value)
   {
      check.setSelected(value);
   }

   public void setName(String name)
   {
      nameLabel.setText(name);
      nameLabel.getTooltip().setText(name);
   }

   public void setParameter(Parameter parameter)
   {
      this.parameter = parameter;
      setName(parameter.getShortPath());

      if (parameter instanceof BooleanParameter)
      {
         BooleanParameter booleanParameter = (BooleanParameter) parameter;
         setValue(booleanParameter.get());
         setControlsEnabled(true);
      }
      else
      {
         setControlsEnabled(false);
      }
   }

   public void setControlsEnabled(boolean enabled)
   {
      check.setDisable(!enabled);
   }
}
