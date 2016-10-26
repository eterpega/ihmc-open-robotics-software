package us.ihmc.robotbuilder.gui;

import javaslang.collection.List;

import java.util.Iterator;

/**
 *
 */
public class ReadOnlyPropertyList<T> implements PropertyList<T>, ReadOnlyProperty<PropertyList<T>>
{
   private final List<EditorProperty<T>> properties;
   private final String name;

   public ReadOnlyPropertyList(List<EditorProperty<T>> properties, String name)
   {
      this.properties = properties;
      this.name = name;
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
}
