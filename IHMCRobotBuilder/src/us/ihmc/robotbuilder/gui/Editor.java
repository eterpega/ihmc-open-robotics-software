package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.scene.Group;
import javafx.scene.layout.GridPane;
import javaslang.Function2;
import javaslang.Tuple;
import javaslang.collection.HashMap;
import javaslang.collection.Map;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;

/**
 * Represents a JavaFX component that can be used to edit a value of the given type.
 */
public abstract class Editor<T> extends GridPane
{
   private final String label, help;

   protected Editor(String label, String help)
   {
      this.label = label;
      this.help = help;
   }

   public abstract Class<T> getValueType();

   public abstract Property<T> valueProperty();

   public final String getLabel() {
      return label;
   }

   public final String getHelp() {
      return help;
   }

   private static Map<Class<?>, Function2<String, String, Editor<?>>> getEditors() {
      return HashMap.ofEntries(
         Tuple.of(String.class, StringEditor::new),
         Tuple.of(List.class, ListEditor::new)
      );
   }

   public static <T> Optional<Editor<T>> editorForClass(Class<? extends T> clazz, String label, String help) {
      Map<Class<?>, Function2<String, String, Editor<?>>> editors = getEditors();
      //noinspection unchecked
      return editors.find(entry -> entry._1.isAssignableFrom(clazz))
                    .map(entry -> entry._2)
                    .map(supplier -> (Editor<T>)supplier.apply(label, help))
                    .toJavaOptional();
   }

   public static <T> Optional<Editor<T>> editorForObject(T object, String label, String help) {
      //noinspection unchecked
      return editorForClass((Class<T>)object.getClass(), label, help);
   }

   public static Optional<Editor<Object>> editorForField(Object object, Field field) {
      return Editor.<Object>editorForClass(field.getType(), field.getName(), field.getName())
            .map(editor -> {
               try
               {
                  editor.valueProperty().setValue(field.get(object));
               }
               catch (IllegalAccessException e)
               {
                  // ignore
               }
               return editor;
            });
   }
}
