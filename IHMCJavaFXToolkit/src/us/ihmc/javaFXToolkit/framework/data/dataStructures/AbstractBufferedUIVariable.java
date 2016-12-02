package us.ihmc.javaFXToolkit.framework.data.dataStructures;

import us.ihmc.javaFXToolkit.framework.data.UIVariable;
import us.ihmc.javaFXToolkit.framework.data.UIVariableType;

/**
 *
 * AbstractBufferedUIVariable implementation are used as a convenient way to add buffer capabilities to UIVariables.
 *
 * In BufferedUIVariable  buffer access should be thread safe: Implementation made with write lock and read locks.
 *
 *  This implementation uses a ring buffer of double primitive type, data are overwritten when one goes beyond buffer's limit
 *
 * Created by amoucheboeuf on 12/16/15.
 */
public abstract class AbstractBufferedUIVariable<T> extends UIVariable<T> implements BufferedUIVariableDataReader<T>, BufferedUIVariableDataWriter<T>
{

   /**
    * A reference of uiVariable is kept in the BufferedUIVariable so that overwritten methods can benefit from the UIVariable's implementation and provide additional behaviors related to the buffer.
    */
   protected final UIVariable<T> uiVariable;
   protected DoubleBufferInterface buffer;

   /**
    *
    * @param uiVariable
    * @param capacity
    */
   public AbstractBufferedUIVariable(UIVariable<T> uiVariable, DoubleBufferInterface buffer, int capacity)
   {
      super(uiVariable.getName(), uiVariable.get()); // gets the value
      this.uiVariable = uiVariable;
      this.buffer = buffer;
   }

   /************************************************************************
    *
    *                       UIVariable related methods
    *
    ************************************************************************/

   /**
    * Call of set implies that the value will be stored in the buffer regardless of the fact that this new value could differ from the last.
    *
    * For a BUfferedUIvariable this is equivalent to call store value in buffer
    *
    * @param newValue
    */
   @Override public void set(T newValue)
   {
      storeValueInBuffer(newValue);
   }

   @Override public String getName()
   {
      return uiVariable.getName();
   }

   @Override public double getValueAsDouble()
   {
      return getValueAsDouble(get());
   }

   @Override public double getValueAsDouble(T value)
   {
      return uiVariable.getValueAsDouble(value);
   }

   @Override public String getValueAsAString()
   {
      return getValueAsAString(get());
   }

   @Override public String getValueAsAString(T value)
   {
      return uiVariable.getValueAsAString(value);
   }

   @Override public void setValueFromString(String stringValue)
   {
      uiVariable.setValueFromString(stringValue);
   }

   @Override public T getValueFromDouble(double value)
   {
      return (T) uiVariable.getValueFromDouble(value);
   }

   @Override public UIVariableType getType()
   {
      return uiVariable.getType();
   }

   @Override public String getUnitsAsString()
   {
      return uiVariable.getUnitsAsString();
   }

   /************************************************************************
    *
    *                  BufferedUIVariableDataReader related methods
    *
    ************************************************************************/

   /**
    *
    * @return
    */
   @Override public int getBufferCapacity()
   {
      return buffer.getCapacity();
   }

   @Override public int getBufferFillCount()
   {
      return buffer.getFillCount();
   }

   @Override public T getValueForIndex(int index) throws IndexOutOfBoundsException
   {
      return getValueFromDouble(buffer.get(index));
   }

   @Override public T[] getDataInRange(int start, int end) throws IndexOutOfBoundsException
   {
      if (end - start <= 0)
         throw new IllegalArgumentException();

      double[] data = getDataInRangeAsDoubleArray(start, end);
      Object[] ret = new Object[data.length];

      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = getValueFromDouble(data[i]);
      }

      return (T[]) ret;
   }

   @Override public T[] getBufferedDataCopy()
   {
      return getDataInRange(0, getBufferFillCount());
   }

   @Override public double getValueAsDoubleForIndex(int index) throws IndexOutOfBoundsException
   {
      return buffer.get(index);
   }

   @Override public double[] getDataInRangeAsDoubleArray(int start, int end)
   {
      return buffer.get(start, end - start);
   }

   @Override public double[] getBufferedDataCopyAsDoubleArray()
   {
      return buffer.get(0, buffer.getFillCount());
   }

   /************************************************************************
    *
    *                 BufferedUIVariableDataWriter related methods
    *
    ************************************************************************/

   /**
    *
    * @param value
    */
   public void storeValueInBuffer(T value)
   {
      super.set(value);
      uiVariable.set(value);
      buffer.add(getValueAsDouble(value));
   }

   /**
    *
    * @param values
    */
   public void storeValuesInBuffer(T[] values)
   {
      double[] data = new double[values.length];

      for (int i = 0; i < data.length; i++)
      {
         data[i] = getValueAsDouble(values[i]);
      }

      buffer.add(data);

      super.set(values[values.length - 1]);
      uiVariable.set(values[values.length - 1]);
   }

   public void storeValueAsDoubleInBuffer(double value)
   {
      buffer.add(value);

      T newValue = getValueFromDouble(value);
      super.set(newValue);
      uiVariable.set(newValue);
   }

   public void storeValuesAsDoubleInBuffer(double[] data)
   {
      buffer.add(data);

      T newValue = getValueFromDouble(data[data.length - 1]);
      super.set(newValue);
   }

   /**
    *
    *  special case, what to do with current value?
    */
   public void clearBuffer()
   {
      buffer.reset();
   }


   public int getReadPosition()
   {
      return buffer.getReadPosition();
   }

   public int getWritePosition()
   {
      return buffer.getWritePosition();
   }


   public int getOverwriteCount()
   {
      return buffer.getOverwriteCount();
   }
}
