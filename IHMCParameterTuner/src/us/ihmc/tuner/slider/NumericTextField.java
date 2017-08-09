package us.ihmc.tuner.slider;

import java.util.regex.Pattern;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import us.ihmc.robotics.MathTools;

/**
 * A text field that only allows numerical entries and supports limiting entries to min/max values.
 */
public class NumericTextField extends TextField
{
   private static final Pattern DECIMAL_PATTERN = Pattern.compile("-?+\\d*\\.?\\d*");
   private static final Pattern INTEGER_PATTERN = Pattern.compile("-?+\\d*");

   /**
    * The minimum value allowed to be entered into the text field.
    */
   private DoubleProperty minProperty = new SimpleDoubleProperty(Double.NEGATIVE_INFINITY);

   /**
    * The maximum value allowed to be entered in the text field.
    */
   private DoubleProperty maxProperty = new SimpleDoubleProperty(Double.POSITIVE_INFINITY);

   /**
    * The entered value.
    */
   private DoubleProperty valueProperty = new SimpleDoubleProperty(0.0);

   /**
    * Whether or not to allow decimal values (1.01, for example) vs. only integers.
    */
   private BooleanProperty allowDecimalsProperty = new SimpleBooleanProperty(true);

   public NumericTextField()
   {
      this("");
   }

   public NumericTextField(String text)
   {
      super(text);

      // Update applied value only when enter is pressed or focus is lost.
      setOnKeyPressed(event ->
      {
         if (event.getCode() == KeyCode.ENTER)
         {
            setValueFromText();
         }
      });
      focusedProperty().addListener((observable, oldValue, newValue) ->
      {
         if (oldValue && !newValue)
         {
            setValueFromText();
         }
      });

      // Recompute the applied value if the min/max change so the applied value can be clipped if necessary.
      minProperty.addListener((observable, oldValue, newValue) -> setValueFromText());
      maxProperty.addListener((observable, oldValue, newValue) -> setValueFromText());

      // When the applied value changes, update the text itself. Don't do the other way around because it is done manually on ENTER or focus loss.
      valueProperty.addListener((observable, oldValue, newValue) -> setText(Double.toString(newValue.doubleValue())));
   }

   @Override
   public void replaceText(int start, int end, String text)
   {
      if (allowDecimalsProperty.getValue() && DECIMAL_PATTERN.matcher(text).matches())
      {
         super.replaceText(start, end, text);
      }
      else if (!allowDecimalsProperty.getValue() && INTEGER_PATTERN.matcher(text).matches())
      {
         super.replaceText(start, end, text);
      }
   }

   @Override
   public void replaceSelection(String replacement)
   {
      if (allowDecimalsProperty.getValue() && DECIMAL_PATTERN.matcher(replacement).matches())
      {
         super.replaceSelection(replacement);
      }
      else if (!allowDecimalsProperty.getValue() && INTEGER_PATTERN.matcher(replacement).matches())
      {
         super.replaceSelection(replacement);
      }
   }

   public DoubleProperty minProperty()
   {
      return minProperty;
   }

   public DoubleProperty maxProperty()
   {
      return maxProperty;
   }

   public DoubleProperty valueProperty()
   {
      return valueProperty;
   }

   public BooleanProperty allowDecimalsProperty()
   {
      return allowDecimalsProperty;
   }

   public void setValueFromText()
   {
      // If field is empty, set to min value.
      if (getText().isEmpty())
      {
         setValueFromText(Double.toString(minProperty.doubleValue()));
      }
      // Otherwise, update from current text value.
      else
      {
         setValueFromText(getText());
      }
   }

   public void setValueFromText(String text)
   {
      double newValue = Double.parseDouble(text);
      setValue(newValue);
   }

   public void setValue(double newValue)
   {
      // Ensure new value is within the limits, clipping if it isn't.
      newValue = MathTools.clamp(newValue, minProperty.get(), maxProperty.get());

      valueProperty.setValue(allowDecimalsProperty.get() ? newValue : Math.round(newValue));
      setTextFromValue();
   }

   private void setTextFromValue()
   {
      setText(allowDecimalsProperty.get()
            ? Double.toString(valueProperty.get())
            : Integer.toString(Math.round(valueProperty.intValue())));
   }
}
