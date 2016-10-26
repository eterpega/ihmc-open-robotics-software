package us.ihmc.robotbuilder.gui.editors2;

import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.editors.NumberField;

import java.util.Optional;

/**
 *
 */
public class PropertyEditorFactory implements Editor.Factory
{
   @Override public Optional<Editor<?>> create(Class<?> clazz, Property<?> value, String name)
   {
      if (String.class.isAssignableFrom(clazz))
      {
         return Optional.of(Editor.wrapControl(new TextField(), TextField::textProperty));
      }

      if (Number.class.isAssignableFrom(clazz))
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new NumberField<>((Class<Number>) (Class) clazz), NumberField::valueProperty));
      }

      if (Boolean.class.isAssignableFrom(clazz))
      {
         return Optional.of(Editor.wrapControl(new CheckBox(), CheckBox::selectedProperty));
      }

      return Optional.empty();
   }
}
