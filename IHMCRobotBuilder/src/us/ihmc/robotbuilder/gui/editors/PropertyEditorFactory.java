package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Vector3d;
import java.awt.Color;
import java.util.List;
import java.util.Optional;

/**
 *
 */
public class PropertyEditorFactory implements Editor.Factory
{
   @Override public Optional<Editor<?>> create(Class<?> clazz, List<Class<?>> genericParameters, Property<?> value)
   {
      if (String.class.equals(clazz))
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new TextField(), TextField::textProperty, (Property<String>)value));
      }

      if (Number.class.isAssignableFrom(clazz) || value.getValue() instanceof Number)
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new NumberField<>(value.getValue().getClass()), NumberField::valueProperty, (Property<Number>)value));
      }

      if (boolean.class.equals(clazz) || Boolean.class.equals(clazz))
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new CheckBox(), CheckBox::selectedProperty, (Property<Boolean>)value));
      }

      if (Vector3d.class.isAssignableFrom(clazz))
      {
         //noinspection unchecked
         return Optional.of(new Vector3DEditor((Property<Vector3d>)value));
      }

      if (TransformDescription.class.isAssignableFrom(clazz))
      {
         //noinspection unchecked
         return Optional.of(new TransformEditor((Property<TransformDescription>)value));
      }

      if (Iterable.class.isAssignableFrom(clazz))
      {
         Class<?> itemType = genericParameters.size() > 0 ? genericParameters.get(0) : Object.class;
         //noinspection unchecked
         return Optional.of(new IterableEditor<>((Property<Iterable<Object>>) value, (Class<Object>)itemType, this));
      }

      if (Optional.class.isAssignableFrom(clazz))
      {
         //noinspection unchecked
         return Optional.of(new OptionalEditor<>(clazz, (Property<Optional<Object>>) value, this, new CreatorFactory()));
      }

      if (Color.class.isAssignableFrom(clazz))
      {
         //noinspection unchecked
         return Optional.of(new ColorEditor((Property<Color>)value));
      }

      return Optional.empty();
   }
}
