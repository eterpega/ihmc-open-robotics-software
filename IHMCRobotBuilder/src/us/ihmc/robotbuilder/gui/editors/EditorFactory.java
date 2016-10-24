package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.value.ObservableValue;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.DefaultPropertyEditorFactory;
import org.controlsfx.property.editor.PropertyEditor;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Vector3d;

/**
 *
 */
public class EditorFactory extends DefaultPropertyEditorFactory
{
   @Override public PropertyEditor<?> call(Item item)
   {
      Class<?> type = item.getType();

      if (item.getValue() instanceof Number)
         return createNumericEditor(item);
      if (type == Vector3d.class)
         return new Vector3DEditor(item);
      if (Iterable.class.isAssignableFrom(type))
         return new IterableEditor<>(item, this);
      if (TransformDescription.class.isAssignableFrom(type))
         return new TransformEditor(item);

      PropertyEditor<?> result = super.call(item);
      if (result != null)
         return result;

      return null;
   }


   private static PropertyEditor<?> createNumericEditor(Item property) {

      //noinspection unchecked
      return new AbstractPropertyEditor<Number, NumberField<Number>>(property, new NumberField<>((Class) property.getType())) {

         @Override protected ObservableValue<Number> getObservableValue() {
            //noinspection unchecked
            return getEditor().valueProperty();
         }

         @Override public Number getValue() {
            return getEditor().valueProperty().getValue();
         }

         @Override public void setValue(Number value) {
            getEditor().valueProperty().setValue(value);
         }

      };
   }
}
