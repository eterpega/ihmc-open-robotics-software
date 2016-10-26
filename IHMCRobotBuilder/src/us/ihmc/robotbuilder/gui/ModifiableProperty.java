package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;

/**
 * Represents a single item in the property editor.
 */
public interface ModifiableProperty<T> extends EditorProperty<T>
{
   @Override
   Property<T> value();
}
