package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Method available to access to the content of a buffered data structure based on the "double" primitive type
 *
 * Created by amoucheboeuf on 6/27/16.
 */
public interface BufferedUIVariableDataReader<T> extends DoubleBufferReader
{

   /**
    *
    * @return
    */
   int getBufferCapacity();

   /**
    *
    * @return
    */
   int getBufferFillCount();

   /**
    *
    * @param index
    * @return
    * @throws IndexOutOfBoundsException
    */
   T getValueForIndex(int index) throws IndexOutOfBoundsException;

   /**
    *
    * @param start
    * @param end
    * @return
    * @throws IndexOutOfBoundsException
    */
   T[] getDataInRange(int start, int end) throws IndexOutOfBoundsException;

   /**
    *
    * @return
    */
   T[] getBufferedDataCopy();



   // NOTE: may not need getValueAsDouble for access to buffered

   /**
    *
    * @param index
    * @return
    * @throws IndexOutOfBoundsException
    */
   double getValueAsDoubleForIndex(int index) throws IndexOutOfBoundsException;

   /**
    *
    * @param start
    * @param end
    * @return
    * @throws IndexOutOfBoundsException
    */
   double[] getDataInRangeAsDoubleArray(int start, int end) throws IndexOutOfBoundsException;

   /**
    *
    * @return
    */
   double[] getBufferedDataCopyAsDoubleArray();







   /**
    * Default implementations makes the buffered Variable appear as a simple DoubleBufferReader
    * @return
    */

   default int getCapacity()
   {
      return getBufferCapacity();
   }


   default int getFillCount()
   {
      return getBufferFillCount();
   }

   default double get(int index) throws ArrayIndexOutOfBoundsException
   {
      return getValueAsDoubleForIndex(index);
   }


   default double[] get(int index, int length) throws ArrayIndexOutOfBoundsException
   {
      return getDataInRangeAsDoubleArray(index, index+length);
   }

}
