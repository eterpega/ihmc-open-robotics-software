package us.ihmc.robotbuilder.gui;

import javafx.beans.value.ObservableValue;

import java.util.Optional;

/**
 * Base interface for items displayed in the property editor.
 */
public interface EditorProperty<T>
{
   String getName();

   Class<T> getValueType();

   ObservableValue<T> value();

   default Optional<String> getDescription()
   {
      return Optional.empty();
   }
}
