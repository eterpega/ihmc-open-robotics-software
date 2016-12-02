package us.ihmc.javaFXToolkit.framework.data;

/**
 * Created by amoucheboeuf on 11/18/15.
 */
public class EnumUIVariable extends UIVariable<Enum>
{
   public EnumUIVariable(String name, Enum initialValue)
   {
      super(name, initialValue);
   }
//
//   public EnumUIVariable(Enum initialValue)
//   {
//      super(initialValue);
//   }
//


   @Override public UIVariableType getType()
   {
      return UIVariableType.ENUM;
   }

   @Override public String getUnitsAsString()
   {
      return null;
   }

   @Override public void set(Enum newValue)
   {
      super.set(newValue);
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public Enum getValueFromDouble(double value)
   {
      // TODO
      return null;
   }

   @Override public double getValueAsDouble(Enum value)
   {
      if (value == null)
         return Double.NaN;// TODO replace that with possibly an exception

      return value.ordinal();
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(Enum value)
   {
      if (value == null)
         return "UNDEFINED";// TODO replace that with possibly an exception

      return value.toString();
   }

   @Override public void setValueFromString(String stringValue)
   {
      // TODO
   }
}
