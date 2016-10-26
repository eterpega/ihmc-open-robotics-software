package us.ihmc.robotbuilder.gui;

import java.util.Objects;

/**
 * Represents a single item in the property editor.
 */
public interface ReadOnlyProperty<T> extends EditorProperty<T>
{
   default String getReadOnlyRepresentation()
   {
      return Objects.toString(value().getValue());
   }
}
