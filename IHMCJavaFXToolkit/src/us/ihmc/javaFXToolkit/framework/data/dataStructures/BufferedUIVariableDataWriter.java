package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Created by amoucheboeuf on 6/27/16.
 */
public interface BufferedUIVariableDataWriter<T> extends DoubleBufferWriter
{
   /**
    *
    * @param value
    */
   void storeValueInBuffer(T value);

   /**
    *
    * @param values
    */
   void storeValuesInBuffer(T[] values);

   /**
    *
    * @param value
    */
   void storeValueAsDoubleInBuffer(double value);

   /**
    *
    * @param values
    */
   void storeValuesAsDoubleInBuffer(double[] values);

   /**
    *
    */
   void clearBuffer();



   default void add(double element)
   {
      storeValueAsDoubleInBuffer(element);
   }

   /**
    * default implementation of {@code add()} equivalent to a call to the method {@link #storeValuesAsDoubleInBuffer(double[])}
    * @param elements
    * @throws ArrayIndexOutOfBoundsException
    */
   default void add(double[] elements) throws ArrayIndexOutOfBoundsException
   {
      storeValuesAsDoubleInBuffer(elements);
   }

   /**
    *
    * TODO See if this is correct!
    * Default implementation for this UIBUfferedVarible can only add data to the end of the buffer
    * @param elements
    * @param startIndex
    * @param numberOfElementsToAdd
    * @throws ArrayIndexOutOfBoundsException
    */
   default void add(double[] elements, int startIndex, int numberOfElementsToAdd) throws ArrayIndexOutOfBoundsException
   {
      storeValuesAsDoubleInBuffer(elements);
   }

   default void reset()
   {
      clearBuffer();
   }

}
