package us.ihmc.robotbuilder.gui.editors;

import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.util.StringConverter;

import java.text.DecimalFormat;
import java.text.ParsePosition;
import java.util.function.UnaryOperator;

/**
 *
 */
public class DecimalTextField extends TextField
{
   private final DecimalFormat format = new DecimalFormat( "0.##########" );

   public DecimalTextField()
   {
      init();
   }

   public DecimalTextField(String text)
   {
      super(text);
      init();
   }

   public double getValue()
   {
      Object value = super.getTextFormatter().getValue();
      if (value instanceof Number)
         return ((Number)super.getTextFormatter().getValue()).doubleValue();
      return 0;
   }

   private void init()
   {

      StringConverter<Double> converter = new StringConverter<Double>() {

         @Override public String toString(Double object)
         {
            return format.format(object);
         }

         @Override public Double fromString(String string)
         {
            return format.parse(string, new ParsePosition(0)).doubleValue();
         }
      };

      UnaryOperator<Change> filter = change ->
      {
         if (change.getControlNewText().isEmpty())
            return change;
         ParsePosition parsePosition = new ParsePosition(0);
         Object object = format.parse(change.getControlNewText(), parsePosition);
         if ( object == null || parsePosition.getIndex() < change.getControlNewText().length() )
         {
            return null;
         }
         else
         {
            return change;
         }
      };

      setTextFormatter(new TextFormatter<>(converter, 0.0, filter));
   }


}
