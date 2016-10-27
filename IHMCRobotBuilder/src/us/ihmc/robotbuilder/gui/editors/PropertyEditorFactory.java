package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Vector3d;
import java.util.Optional;

/**
 *
 */
public class PropertyEditorFactory implements Editor.Factory
{
   @Override public Optional<Editor<?>> create(Class<?> clazz, Property<?> value, String name)
   {
      if (value.getValue() instanceof String)
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new TextField(), TextField::textProperty, (Property<String>)value));
      }

      if (value.getValue() instanceof Number)
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new NumberField<>(value.getValue().getClass()), NumberField::valueProperty, (Property<Number>)value));
      }

      if (value.getValue() instanceof Boolean)
      {
         //noinspection unchecked
         return Optional.of(Editor.wrapControl(new CheckBox(), CheckBox::selectedProperty, (Property<Boolean>)value));
      }

      if (value.getValue() instanceof Vector3d)
      {
         //noinspection unchecked
         return Optional.of(new Vector3DEditor((Property<Vector3d>)value));
      }

      if (value.getValue() instanceof TransformDescription)
      {
         //noinspection unchecked
         return Optional.of(new TransformEditor((Property<TransformDescription>)value));
      }

      return Optional.empty();
   }
}
