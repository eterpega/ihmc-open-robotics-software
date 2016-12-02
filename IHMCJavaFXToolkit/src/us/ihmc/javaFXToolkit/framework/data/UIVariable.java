package us.ihmc.javaFXToolkit.framework.data;

import javafx.beans.property.SimpleObjectProperty;

/**
 *
 *
 * Created by amoucheboeuf on 11/18/15.
 */
public abstract class UIVariable<T> extends SimpleObjectProperty<T>
{


   public UIVariable(String name, T initialValue)
   {
      super(null, name, initialValue);
   }


   /**
    * {@inheritDoc}
    */
   @Override public void set(T newValue)
   {
      super.set(newValue);
   }

   abstract public double getValueAsDouble();

   abstract public double getValueAsDouble(T value);

   abstract public String getValueAsAString();

   abstract public String getValueAsAString(T value);

   abstract public T getValueFromDouble(double value);

   abstract public void setValueFromString(String stringValue);

   abstract public UIVariableType getType();

   abstract public String getUnitsAsString();
}
