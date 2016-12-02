package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.ConcurrentRingDoubleBuffer;
import us.ihmc.robotics.MathTools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Created by amoucheboeuf on 7/22/16.
 */
public class DataSeriesManagerTest
{
   public static final int BUFFER_SIZE = 32;

   private ConcurrentRingDoubleBuffer ringBufferIterationBeyondCapacity;
   private ConcurrentRingDoubleBuffer ringBufferIterationWithinCapacity;

   @Before public void setup()
   {
      ringBufferIterationWithinCapacity = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < BUFFER_SIZE; i++)
      {
         ringBufferIterationWithinCapacity.add(i);
      }

      ringBufferIterationBeyondCapacity = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < 2 * BUFFER_SIZE + 1; i++)
      {
         ringBufferIterationBeyondCapacity.add(i);
      }
   }

   @Test public void testGetCurrentIterationNumber()
   {
      ConcurrentRingDoubleBuffer concurrentRingDoubleBuffer = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < BUFFER_SIZE * 2 + 1; i++)
      {
         concurrentRingDoubleBuffer.add(i);
         int iterationCount = DataSeriesManager.getCurrentIterationNumber(concurrentRingDoubleBuffer);
         assertEquals(i + 1, iterationCount);
      }
   }

   @Test public void testAccessedToDataInBufferFilledWithinCapacity()
   {
      ImmutablePair<Integer, Integer> indices = DataSeriesManager.getBufferIndicesForIterationIndicesWithinRange(ringBufferIterationWithinCapacity, 0, BUFFER_SIZE);

      assertTrue(indices.getLeft() == 0);
      assertTrue(indices.getRight() == BUFFER_SIZE - 1);
   }

   @Test public void testAccessedToDataInBufferFilledBeyondCapacity()
   {
      ConcurrentRingDoubleBuffer concurrentRingDoubleBuffer = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < BUFFER_SIZE; i++)
      {
         concurrentRingDoubleBuffer.add(i);
      }
      System.out.println("Buffer fill count: " + concurrentRingDoubleBuffer.getFillCount());

      assertTrue(concurrentRingDoubleBuffer.getFillCount() == BUFFER_SIZE);
   }

   @Test public void testGetBufferIndicesForIterationIndicesWithinRange()
   {
      // fill buffer
      ConcurrentRingDoubleBuffer concurrentRingDoubleBuffer = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < BUFFER_SIZE; i++)
      {
         concurrentRingDoubleBuffer.add(i);
      }

      int iterationLeftIndex = 0;
      int iterationRightIndex = BUFFER_SIZE - 1;

      int expectedBufferLeftIndex = 0;
      int expectedBufferRightIndex = BUFFER_SIZE - 1;

      ImmutablePair<Integer, Integer> indices = DataSeriesManager.getBufferIndicesForIterationIndicesWithinRange(concurrentRingDoubleBuffer, iterationLeftIndex, iterationRightIndex);

      assertEquals(expectedBufferLeftIndex, (int)indices.getLeft());
      assertEquals(expectedBufferRightIndex, (int)indices.getRight());

   }

   @Test public void testGetBufferIndicesForIterationIndicesWithinRangeWithWrongLeftIndex()
   {

   }

   @Test public void testGetBufferIndicesForIterationIndicesWithinRangeWithWrongRightIndex()
   {

   }

   @Test public void testGetBufferIndicesForIterationIndicesWithinRangeWithWrongRange()
   {

   }

   @Test public void testGetBufferIndicesForIterationIndicesWithinRangeWithIterationBeyondBufferFillCount()
   {

   }

   @Test public void testDataContinuityForDataWithinRangeOfBufferSize()
   {
      int iterationLimit = BUFFER_SIZE;

      // fill buffer
      ConcurrentRingDoubleBuffer concurrentRingDoubleBuffer = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < iterationLimit; i++)
      {
         concurrentRingDoubleBuffer.add(i);
      }

      for (int i = iterationLimit - BUFFER_SIZE + 1; i < iterationLimit; i++)
      {
         ImmutablePair<Integer, Integer> indices = DataSeriesManager.getBufferIndicesForIterationIndicesWithinRange(concurrentRingDoubleBuffer, 0, i);
         assertEquals(0, (int)indices.getLeft());
         assertEquals(i, (int)indices.getRight());
      }
   }

   /**
    * Test whether data stored in the buffer are correctly retrieved from it using the indices returned by the method DataSeriesManager.getBufferIndicesForIterationIndicesWithinRange()
    */
   @Test public void testDataContinuityForDataBeyondBufferSize()
   {
      int iterationLimit = 2 * BUFFER_SIZE + 10;

      // fill buffer + 10
      double[] iterationRecording = new double[iterationLimit];
      ConcurrentRingDoubleBuffer concurrentRingDoubleBuffer = new ConcurrentRingDoubleBuffer(BUFFER_SIZE);
      for (int i = 0; i < iterationLimit; i++)
      {
         concurrentRingDoubleBuffer.add(i);
         iterationRecording[i] = i;

         int iterationLeftIndex;
         if (i < BUFFER_SIZE)
         {
            iterationLeftIndex = 0;
         }
         else
         {
            iterationLeftIndex = i + 1 - BUFFER_SIZE;
         }

         if (iterationLeftIndex < i)
         {
            ImmutablePair<Integer, Integer> indices = DataSeriesManager.getBufferIndicesForIterationIndicesWithinRange(concurrentRingDoubleBuffer, iterationLeftIndex, i);

            //            System.out.println();
            //            System.out.println("Indices " + Arrays.toString(indices));

            double[] tempCopy = concurrentRingDoubleBuffer.get(indices.getLeft(), indices.getRight() - indices.getLeft());

            for (int j = 0; j < tempCopy.length; j++)
            {
               assertTrue(MathTools.epsilonEquals(iterationRecording[iterationLeftIndex + j], tempCopy[j], 0.0001));
            }

            //            System.out.println("tempCopy " + Arrays.toString(tempCopy));
         }

      }
   }

}
