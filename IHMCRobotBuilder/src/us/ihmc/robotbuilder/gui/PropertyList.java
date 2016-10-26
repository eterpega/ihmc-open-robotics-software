package us.ihmc.robotbuilder.gui;

import javafx.beans.value.ObservableValue;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;

import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

/**
 * Represents a set of items that are editable in the GUI using various editors.
 */
public interface PropertyList<T> extends Iterable<EditorProperty<T>>, EditorProperty<PropertyList<T>>
{
   EditorProperty<T> getProperty(int index);

   int size();

   default Stream<EditorProperty<T>> stream()
   {
      return StreamSupport.stream(spliterator(), false);
   }

   @Override default ObservableValue<PropertyList<T>> value()
   {
      return FunctionalObservableValue.join(stream().map(EditorProperty::value).collect(Collectors.toList()))
                                      .map(value -> this);
   }
}
