package us.ihmc.javaFXToolkit.framework.data;

/**
 * Created by amoucheboeuf on 11/18/15.
 */
public class BooleanUIVariable extends UIVariable<Boolean>
{
   public BooleanUIVariable(String name, Boolean initialValue)
   {
      super(name, initialValue);
   }

//   public BooleanUIVariable(Boolean initialValue)
//   {
//      super(initialValue);
//   }

   @Override public UIVariableType getType()
   {
      return UIVariableType.BOOLEAN;
   }

   @Override public String getUnitsAsString()
   {
      return null;
   }

   @Override public void set(Boolean newValue)
   {
      super.set(newValue);
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public double getValueAsDouble(Boolean value)
   {
      if (value == null)
         return Double.NaN;// TODO replace that with possibly an exception

      return value ? 1.0 : 0.0;
   }

   @Override public Boolean getValueFromDouble(double value)
   {
      if (value == 0.0)
         return Boolean.FALSE;
      return Boolean.TRUE;
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(Boolean value)
   {
      if (value == null)
         return "UNDEFINED";// TODO replace that with possibly an exception

      return value ? "True" : "False";
   }

   @Override public void setValueFromString(String stringValue)
   {
      if (stringValue == null || stringValue.isEmpty())
      {
         set(false);
         return;
      }

      char firstChar = stringValue.trim().toLowerCase().charAt(0);
      if (firstChar == 't' || firstChar == '1' || firstChar == 'y')
      {
         set(true);
         return;
      }

      set(false);
   }
}
