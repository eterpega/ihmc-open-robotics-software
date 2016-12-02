package us.ihmc.javaFXToolkit.framework.data;

/**
 * Created by amoucheboeuf on 11/18/15.
 */
public class DoubleUIVariable extends UIVariable<Double>
{
   public DoubleUIVariable(String name, Double initialValue)
   {
      super(name, initialValue);
   }

   //   public DoubleUIVariable(Double initialValue)
   //   {
   //      super(initialValue);
   //   }

   @Override public UIVariableType getType()
   {
      return UIVariableType.DOUBLE;
   }

   @Override public String getUnitsAsString()
   {
      return null;
   }

   @Override public void set(Double newValue)
   {
      super.set(newValue);
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public double getValueAsDouble(Double value)
   {
      if (value == null)
         return Double.NaN;// TODO replace that with possibly an exception

      return value;
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(Double value)
   {
      if (value == null)
         return "UNDEFINED";// TODO replace that with possibly an exception

      return value.toString();
   }

   @Override public void setValueFromString(String stringValue)
   {
      set(Double.parseDouble(stringValue));
   }

   @Override public Double getValueFromDouble(double value)
   {
      return value;
   }
}
