package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.scene.Node;
import javafx.scene.control.Control;
import javaslang.Function2;

import java.util.Optional;
import java.util.function.Function;

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

   public static <C extends Control, T> Editor<T> wrapControl(C control, Function<C, ? extends Property<T>> valuePropertySupplier)
   {
      return wrapControl(control, valuePropertySupplier.apply(control));
   }

   public static <C extends Control, T> Editor<T> wrapControl(C control, Property<T> valueProperty)
   {
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
      Optional<Editor<?>> create(Class<?> clazz, Property<?> property, String name);
   }
}
