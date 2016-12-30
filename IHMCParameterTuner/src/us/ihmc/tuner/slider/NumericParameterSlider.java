package us.ihmc.tuner.slider;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tuner.slider.config.SliderConfiguration;

/**
 * A parameter slider that supports entry by either text box or slider with upper and lower limit text boxes that specify the allowable range.
 */
public class NumericParameterSlider extends VBox
{
   private static final String FXML_PATH = "numeric_param_slider.fxml";

   @FXML private Pane root;
   @FXML private NumericTextField maxText;
   @FXML private NumericTextField minText;
   @FXML private Slider slider;
   @FXML private NumericTextField sliderText;
   @FXML private Label nameLabel;

   private final Parameter parameter;
   private final SliderConfiguration sliderConfiguration;

   public NumericParameterSlider(Parameter parameter, SliderConfiguration sliderConfiguration) throws IOException
   {
      this.parameter = parameter;
      this.sliderConfiguration = sliderConfiguration;

      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      loader.load();
   }

   public void initialize()
   {
      // Bind min/max values to the slider min/max.
      maxText.valueProperty().bindBidirectional(slider.maxProperty());
      minText.valueProperty().bindBidirectional(slider.minProperty());

      // Bind value properties together.
      sliderText.valueProperty().bindBidirectional(slider.valueProperty());

      // Bind slider entry max/min to the max/min specified by the upper and lower text boxes.
      sliderText.maxProperty().bind(maxText.valueProperty());
      sliderText.minProperty().bind(minText.valueProperty());

      // Recompute slider tick marks when slider range is updated.
      slider.minProperty().addListener((observable, oldValue, newValue) -> recomputeSliderTicks());
      slider.maxProperty().addListener((observable, oldValue, newValue) -> recomputeSliderTicks());

      // Set min/max to infinity before adding parameter to make sure param value doesn't get accidentally clipped.
      setMin(Double.NEGATIVE_INFINITY);
      setMax(Double.POSITIVE_INFINITY);

      minText.allowDecimalsProperty().setValue(true);
      maxText.allowDecimalsProperty().setValue(true);
      sliderText.allowDecimalsProperty().setValue(true);

      // Copy parameter configuration values over.
      setName(parameter.getShortPath());
      propagateParameterToSlider();
      setMin(sliderConfiguration.defaultLowerLimit());
      setMax(sliderConfiguration.defaultUpperLimit());
      slider.setSnapToTicks(sliderConfiguration.snapToTicks());

      maxText.setDisable(!sliderConfiguration.upperLimitMutable());
      minText.setDisable(!sliderConfiguration.lowerLimitMutable());

      // When the parameter value changes, update the slider value.
      parameter.addChangeListener(parameter -> propagateParameterToSlider());

      // When the slider value changes, update the parameter value.
      sliderText.valueProperty().addListener((observable, oldValue, newValue) -> propagateSliderToParameter(newValue.doubleValue()));

      recomputeSliderTicks();
   }

   private void setName(String name)
   {
      nameLabel.setText(name);
      nameLabel.getTooltip().setText(name);
   }

   private void setValue(double value)
   {
      sliderText.setValue(value);
   }

   private void setMin(double min)
   {
      minText.setValue(min);
   }

   private void setMax(double max)
   {
      maxText.setValue(max);
   }

   private void propagateParameterToSlider()
   {
      setValue(sliderConfiguration.getValueFromParameter(parameter));
   }

   private void propagateSliderToParameter(double newValue)
   {
      sliderConfiguration.applyValueToParameter(newValue, parameter);
   }

   private void recomputeSliderTicks()
   {
      double min = minText.valueProperty().doubleValue();
      double max = maxText.valueProperty().doubleValue();

      int majorTickCount = sliderConfiguration.majorTickCount(min, max);
      int minorTickCount = sliderConfiguration.minorTickCount(min, max);

      double range = max - min;
      double majorTickUnit = range / majorTickCount;

      slider.setMajorTickUnit(majorTickUnit > 0 ? majorTickUnit : 1);
      slider.setMinorTickCount(minorTickCount);
   }
}
