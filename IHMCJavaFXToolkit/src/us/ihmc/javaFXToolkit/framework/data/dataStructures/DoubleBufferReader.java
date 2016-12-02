package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Created by amoucheboeuf on 7/2/16.
 */
public interface DoubleBufferReader
{

   /**
    * Return the buffer's capacity
    *
    * @return capacity
    */
   int getCapacity();

   /**
    * Returns the current number of elements present in the data source's buffer.
    *
    * @return fillcount
    */
   int getFillCount();

   /**
    * Returns data contained in the data source's buffer from the index to the index + length element
    * Throws a bufferUnderflowException if the buffer's capacity is exceeded or if the index + length element position is greater than the number of elements in the buffer.
    * @param index
    * @return double array of data
    */
   double get(int index) throws ArrayIndexOutOfBoundsException;

   /**
    * Returns a copy of data contained in the data source's buffer from the index to the index + length element
    * Throws a bufferUnderflowException if the buffer's capacity is exceeded or if the index + length element position is greater than the number of elements in the buffer.
    * @param index
    * @param length
    * @return double array of data
    */
   double[] get(int index, int length) throws ArrayIndexOutOfBoundsException;


   int getReadPosition();

   /**
    *  If the buffer is circular, return the number of time the buffer capacity has been exceeded as next data entry start to be overwrite data in the buffer
    *  A call to reset() method resets this value to 0.
    */
   int getOverwriteCount();


}
