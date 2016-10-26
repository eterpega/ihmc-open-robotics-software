package us.ihmc.robotbuilder.gui;

import com.sun.javafx.collections.ObservableListWrapper;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ObservableList;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.stream.Collectors;

/**
 *
 */
public class ModifiablePropertyList<T> implements PropertyList<T>, ModifiableProperty<PropertyList<T>>
{
   private final ObservableList<EditorProperty<T>> properties = new ObservableListWrapper<>(new ArrayList<>());
   private final Property<PropertyList<T>> valueProperty = new ThisObjectProperty();
   private final String name;

   public ModifiablePropertyList(Iterable<EditorProperty<T>> properties, String name)
   {
      properties.forEach(this.properties::add);
      this.name = name;

      this.properties.addListener((Observable observable) -> valueProperty.setValue(this));
      FunctionalObservableValue.join(stream().map(EditorProperty::value).collect(Collectors.toList()))
                               .map(value -> this).consume(valueProperty::setValue);
   }

   @Override public EditorProperty<T> getProperty(int index)
   {
      return properties.get(index);
   }

   @Override public int size()
   {
      return properties.size();
   }

   @Override public Iterator<EditorProperty<T>> iterator()
   {
      return properties.iterator();
   }

   @Override public String getName()
   {
      return name;
   }

   @Override public Class<PropertyList<T>> getValueType()
   {
      //noinspection unchecked
      return (Class<PropertyList<T>>)(Class)PropertyList.class;
   }

   @Override public Property<PropertyList<T>> value()
   {
      return valueProperty;
   }

   private class ThisObjectProperty extends SimpleObjectProperty<PropertyList<T>>
   {
      @Override public void setValue(PropertyList<T> v)
      {
         super.setValue(v);
         invalidated();
         fireValueChangedEvent();
      }
   }
}
