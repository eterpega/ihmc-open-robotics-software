package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.scene.Node;
import javafx.scene.control.Control;

import java.util.Optional;
import java.util.function.Function;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;

/**
 *
 */
public abstract class Editor<T>
{
   private final Property<T> valueProperty;

   public Editor(Property<T> valueProperty)
   {
      this.valueProperty = valueProperty;
   }

   public abstract Node getEditor();

   public final Property<T> valueProperty()
   {
      return valueProperty;
   }

   public static <C extends Control, T> Editor<T> wrapControl(C control, Function<C, ? extends Property<T>> controlPropertySupplier, Property<T> valueProperty)
   {
      return wrapControl(control, controlPropertySupplier.apply(control), valueProperty);
   }

   public static <C extends Control, T> Editor<T> wrapControl(C control, Property<T> controlValue, Property<T> valueProperty)
   {
      functional(valueProperty).consume(controlValue::setValue);
      functional(controlValue).consume(valueProperty::setValue);
      return new Editor<T>(valueProperty)
      {
         @Override public Node getEditor()
         {
            return control;
         }
      };
   }

   public interface Factory
   {
      Optional<Editor<?>> create(Class<?> clazz, Property<?> property);
   }
}
