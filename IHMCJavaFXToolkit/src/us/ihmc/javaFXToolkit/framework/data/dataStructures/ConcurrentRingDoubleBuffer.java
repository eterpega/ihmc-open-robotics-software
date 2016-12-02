package us.ihmc.javaFXToolkit.framework.data.dataStructures;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Using ReadWriteLock:
 *
 * The buffer can be accessed by multiple readers or a single writer at a time but not both types at once.
 *
 * Created by amoucheboeuf on 6/23/16.
 */
public class ConcurrentRingDoubleBuffer extends RingBufferDouble
{

   private final ReentrantReadWriteLock reentrantReadWriteLock;
   private final Lock readLock;
   private final Lock writeLock;

   public ConcurrentRingDoubleBuffer(int capacity)
   {
      super(capacity);
      this.reentrantReadWriteLock = new ReentrantReadWriteLock(false);
      this.readLock = reentrantReadWriteLock.readLock();
      this.writeLock = reentrantReadWriteLock.writeLock();
   }

   /**
    *  {@inheritDoc}
    */
   @Override public void reset()
   {
      synchronized (this)
      {
         super.reset();
      }
   }

   /**
    *  {@inheritDoc}
    * @param element
    */
   @Override public void add(double element)
   {
      writeLock.lock();
      try
      {
         super.add(element);
      }
      finally
      {
         writeLock.unlock();
      }

   }

   @Override public void add(double[] elements)
   {
      writeLock.lock();
      try
      {
         super.add(elements, 0, elements.length);
      }
      finally
      {
         writeLock.unlock();
      }
   }


   @Override public void add(double[] elements, int startIndex, int numberOfElementsToAdd) throws ArrayIndexOutOfBoundsException
   {
      writeLock.lock();
      try
      {
         super.add(elements, startIndex,  numberOfElementsToAdd);
      }
      finally
      {
         writeLock.unlock();
      }

   }

   @Override public double get(int index) throws RuntimeException
   {
      readLock.lock();
      try
      {
         return super.get(index);
      }
      finally
      {
         readLock.unlock();
      }
   }



   @Override public double[] get(int lowerIndex, int copyLength) throws RuntimeException
   {
      readLock.lock();
      try
      {
         return super.get(lowerIndex, copyLength);
      }
      finally
      {
         readLock.unlock();
      }
   }

   @Override public int getOverwriteCount()
   {
      readLock.lock();
      try
      {
         return super.getOverwriteCount();
      }
      finally
      {
         readLock.unlock();
      }
   }


}
