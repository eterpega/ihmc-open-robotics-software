package us.ihmc.javaFXToolkit.framework.data.dataStructures;

/**
 * Fixed length ring buffer allowing overwriting of data when capacity is exceeded. Elements can be inserted and read one by one or by batches.
 * Not thread safe.
 * Created by amoucheboeuf on 2/3/16.
 */
public class RingBufferDouble extends DoubleBufferInterface
{
   private final double[] elements;
   private int capacity = 0;
   private int overwriteCount = 0;


   private int fillCount = 0;
   private int writePosition = 0;
   private int readPosition  = 0; // position of the first element inserted to the ring buffer or the oldest element


   public RingBufferDouble(int capacity)
   {
      this.capacity = capacity;
      this.elements = new double[capacity]; // needs to be of a power of two: implement setter

   }


   public void reset()
   {
         this.readPosition = 0;
         this.writePosition = 0;
         this.fillCount = 0;
         this.overwriteCount = 0;
   }

   public double[] getBuffer() // TODO to be removed
   {
      return elements;
   }

   public int getCapacity()
   {
      return this.capacity;
   }

   public int getFillCount()
   {
      return this.fillCount;
   }

   /**
    *
    * @param element
    */
   public void add(double element)
   {
      elements[writePosition] = element;
      writePosition++;
      fillCount ++;

      if(writePosition >= capacity)
      {
         writePosition = 0;
         overwriteCount++;
      }

      if(fillCount >= capacity)
      {
         fillCount = capacity;
         readPosition = writePosition;
      }
   }

   public void add(double[] elements)
   {
      this.add(elements, 0, elements.length);
   }


   public void add(double[] elements, int startIndex, int numberOfElementsToAdd) throws ArrayIndexOutOfBoundsException
   {
      // more elements are added to the buffer than its maximal capacity return error out of bounds exception or buffer overflow
      if(numberOfElementsToAdd > capacity)
      {
         throw new ArrayIndexOutOfBoundsException("Cannot add "+elements.length +" elements to buffer: maximum capacity["+capacity+"] ");
      }

      int sourcePosition = startIndex;
      int destinationPosition = writePosition;
      int length = numberOfElementsToAdd;

      if(length > capacity - writePosition)
      {
         length = capacity - writePosition;
         System.arraycopy(elements, sourcePosition, this.elements, destinationPosition, length);
         destinationPosition = 0;
         sourcePosition = length + startIndex;

         length = numberOfElementsToAdd - length;

         readPosition = writePosition;
         fillCount = capacity;
         writePosition = 0;
         overwriteCount++;
      }

      System.arraycopy(elements, sourcePosition, this.elements, destinationPosition, length);
      writePosition += length;

      fillCount+=length;

      if(fillCount >= capacity)
      {
         fillCount = capacity;
         readPosition = writePosition;
      }

      // TODO maybe make use of fork join here if buffer to be copied is really big
   }

   public double get(int index) throws RuntimeException // Always peek between read and write position
   {
      if(index >= fillCount)
      {
         throw new ArrayIndexOutOfBoundsException("Index ["+index+"] is greater than number of elements in the buffer ["+fillCount+"]");
      }
      else if (index < 0)
      {
         throw new ArrayIndexOutOfBoundsException("Index ["+index+"] cannot be negative");
      }

      int elementIndex = readPosition + index;
      if(elementIndex >= capacity)
      {
         elementIndex = elementIndex - capacity;
      }

      return elements[elementIndex];
   }



   public double[] get(int lowerIndex, int copyLength) throws RuntimeException // TODO get a copy optimize later
   {

      if(lowerIndex < 0)
      {
         throw new ArrayIndexOutOfBoundsException("Lower Index ["+lowerIndex+"] cannot be negative");
      }

      if(lowerIndex + copyLength > fillCount)
      {
         throw new ArrayIndexOutOfBoundsException("Trying to access more elements ["+(lowerIndex + copyLength) +"] than current number of elements in the buffer ["+fillCount+"]");
      }

      double[] copy = new double[copyLength];

      int sourcePosition = readPosition + lowerIndex;
      if(sourcePosition >= capacity)
      {
         sourcePosition = sourcePosition - capacity;
      }

      int destinationPosition = 0;
      int length = copyLength;

      if(length > capacity - sourcePosition)
      {
         length = capacity - sourcePosition;
         System.arraycopy(elements, sourcePosition, copy, destinationPosition, length);
         destinationPosition = length;
         sourcePosition = 0;
         length = copyLength - length;
      }

      System.arraycopy(elements, sourcePosition, copy, destinationPosition, length);

      return  copy;
   }

   public int getWritePosition()
   {
      return this.writePosition;
   }

   public int getReadPosition()
   {
      return this.readPosition;
   }


   public int getOverwriteCount()
   {
      return this.overwriteCount;
   }
}
