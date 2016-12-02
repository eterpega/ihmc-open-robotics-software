package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Created by amoucheboeuf on 7/2/16.
 */
public interface DoubleBufferWriter
{
   /**
    *
    * @param element
    */
   public void add(double element);

   /**
    *
    * @param elements
    */
   public void add(double[] elements) throws ArrayIndexOutOfBoundsException;

   /**
    *
    * @param elements
    * @param startIndex
    * @param numberOfElementsToAdd
    * @throws ArrayIndexOutOfBoundsException
    */
   public void add(double[] elements, int startIndex, int numberOfElementsToAdd) throws ArrayIndexOutOfBoundsException;


   /**
    * Resets read/write position and fill count of the buffer to zero
    */
   void reset();

   int getWritePosition();
}
