package us.ihmc.javaFXToolkit.framework.data;

/**
 * Created by amoucheboeuf on 11/18/15.
 */
public class LongUIVariable extends UIVariable<Long>
{
   public LongUIVariable(String name, Long initialValue)
   {
      super(name, initialValue);
   }
//
//   public LongUIVariable(Long initialValue)
//   {
//      super(initialValue);
//   }

   @Override public UIVariableType getType()
   {
      return UIVariableType.LONG;
   }

   @Override public String getUnitsAsString()
   {
      return null;
   }

   @Override public void set(Long newValue)
   {
      super.set(newValue);
   }

   @Override public void setValueFromString(String stringValue)
   {
      set(Long.parseLong(stringValue));
   }

   @Override public Long getValueFromDouble(double value)
   {
      return (long) value;
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public double getValueAsDouble(Long value)
   {
      if (value == null)
         return Double.NaN;// TODO replace that with possibly an exception

      return value;
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(Long value)
   {
      if (value == null)
         return "UNDEFINED"; // TODO replace that with possibly an exception

      return value.toString();
   }
}
