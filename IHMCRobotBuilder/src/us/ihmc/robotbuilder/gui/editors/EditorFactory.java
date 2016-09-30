package us.ihmc.robotbuilder.gui.editors;

import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.DefaultPropertyEditorFactory;
import org.controlsfx.property.editor.PropertyEditor;

import javax.vecmath.Vector3d;
import java.util.List;

/**
 *
 */
public class EditorFactory extends DefaultPropertyEditorFactory
{
   @Override public PropertyEditor<?> call(Item item)
   {
      PropertyEditor<?> result = super.call(item);
      if (result != null)
         return result;

      Class<?> type = item.getType();

      if (type == Vector3d.class)
         return new Vector3DEditor();
      if (Iterable.class.isAssignableFrom(type))
         return new IterableEditor<>(this);

      return null;
   }
}
