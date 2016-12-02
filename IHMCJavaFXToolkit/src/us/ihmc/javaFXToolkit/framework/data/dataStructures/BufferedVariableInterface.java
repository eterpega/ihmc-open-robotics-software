package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Created by amoucheboeuf on 6/17/16.
 */
public interface BufferedVariableInterface
{


   /**
    * Return the data source's buffer capacity
    *
    * @return capacity
    */
   int getBufferCapacity();

   /**
    * Returns the current number of elements present in the data source's buffer.
    *
    * @return fillcount
    */
   int getBufferFillCount();



   public void add(double element);


   public void add(double[] elements);


   public void add(double[] elements, int startIndex, int numberOfElementsToAdd) throws ArrayIndexOutOfBoundsException;




   /**
    * Returns data contained in the data source's buffer from the index to the index + length element
    * Throws a bufferUnderflowException if the buffer's capacity is exceeded or if the index + length element position is greater than the number of elements in the buffer.
    * @param index
    * @param length
    * @return double array of data
    */
   double[] getData(int index, int length) throws ArrayIndexOutOfBoundsException;


   /**
    * Returns data contained in the data source's buffer from the index to the index + length element
    * Throws a bufferUnderflowException if the buffer's capacity is exceeded or if the index + length element position is greater than the number of elements in the buffer.
    * @param index
    * @return double array of data
    */
   double getData(int index) throws ArrayIndexOutOfBoundsException;


   /**
    * Each time a new data is added to this dataSource iteration number is incremented
    * @return the current iteration number represent the number of times entries have been made to the buffer
    */
   long getLastElementIterationNumber();


   /**
    *
    * @return
    */
   default long getFirstElementIterationNumber()
   {
      if(getLastElementIterationNumber() - getBufferFillCount() < 0 )
      {
         throw new ArrayIndexOutOfBoundsException();
      }
      return getLastElementIterationNumber() - getBufferFillCount();
   }


   void reset();

}
