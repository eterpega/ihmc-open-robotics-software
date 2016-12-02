package us.ihmc.javaFXToolkit.framework.data;

/**
 * Created by amoucheboeuf on 11/18/15.
 */
public class IntegerUIVariable extends UIVariable<Integer>
{
   public IntegerUIVariable(String name, Integer initialValue)
   {
      super(name, initialValue);
   }

//   public IntegerUIVariable(Integer initialValue)
//   {
//      super(initialValue);
//   }

   @Override public UIVariableType getType()
   {
      return UIVariableType.INTEGER;
   }

   @Override public String getUnitsAsString()
   {
      return null;
   }

   @Override public void set(Integer newValue)
   {
      super.set(newValue);
   }

   @Override public void setValueFromString(String stringValue)
   {
      set(Integer.parseInt(stringValue));
   }

   @Override public Integer getValueFromDouble(double value)
   {
      return (int) value;
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public double getValueAsDouble(Integer value)
   {
      if (value == null)
         return Double.NaN;// TODO replace that with possibly an exception

      return value;
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(Integer value)
   {
      if (value == null)
         return "UNDEFINED";// TODO replace that with possibly an exception

      return value.toString();
   }
}
