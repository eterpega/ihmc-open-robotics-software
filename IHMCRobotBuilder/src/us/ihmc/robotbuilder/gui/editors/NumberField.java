package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;

import java.math.BigInteger;
import java.util.Optional;
import java.util.function.Function;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 * Field for holding numeric values.
 */
public class NumberField<T extends Number> extends TextField
{

   private final Property<Number> value;
   private final Function<String, Optional<Number>> stringToNumber;

   public NumberField(Class<?> cls)
   {
      Property<String> textProperty = noCycle(textProperty());

      Function<Function<String, ? extends Number>, Function<String, Optional<Number>>> stringToOptionalNumber = stringToNr -> string ->
      {
         try
         {
            return Optional.ofNullable(string).map(String::trim).map(stringToNr);
         }
         catch (Exception ex)
         {
            return Optional.empty();
         }
      };

      if (cls == byte.class || cls == Byte.class || cls == short.class || cls == Short.class || cls == int.class || cls == Integer.class || cls == long.class
            || cls == Long.class || cls == BigInteger.class)
      {
         value = noCycle(new SimpleLongProperty(0));
         stringToNumber = stringToOptionalNumber.apply(Long::valueOf);
      }
      else
      {
         value = noCycle(new SimpleDoubleProperty(0));
         stringToNumber = stringToOptionalNumber.apply(Double::valueOf);
      }

      functional(textProperty)
             .flatMapOptional(stringToNumber)
             .consume(value::setValue);

      functional(value)
             .filter(newValue -> {
                Optional<Number> oldValueOpt = stringToNumber.apply(getText());
                Optional<Number> newValueOpt = Optional.ofNullable(newValue);
                return !oldValueOpt.equals(newValueOpt) && newValueOpt.isPresent();
             })
             .map(Object::toString)
             .consume(textProperty::setValue);

      setTextFormatter(new TextFormatter<>(c ->
                                           {
                                              if (c.getControlNewText().isEmpty() || stringToNumber.apply(c.getControlNewText()).isPresent())
                                                 return c;
                                              return null;
                                           }));
   }

   public final Property<T> valueProperty()
   {
      //noinspection unchecked
      return (Property) value;
   }

   public Optional<T> getValue()
   {
      return Optional.ofNullable(valueProperty().getValue());
   }
}