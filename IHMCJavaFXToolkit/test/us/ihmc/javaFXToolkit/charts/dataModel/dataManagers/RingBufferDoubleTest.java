package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.RingBufferDouble;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.assertTrue;

/**
 * Created by amoucheboeuf on 2/5/16.
 */
public class RingBufferDoubleTest
{
   private static final int BUFFER_CAPACITY = 20;
   private static final int TEST_DATA_SET_SIZE = 40;

   private double[] data;
   private RingBufferDouble instance;
   private Random random;

   @Before public void setUp()
   {
      data = new double[TEST_DATA_SET_SIZE];
      random = new Random();
      for (int i = 0; i < data.length; i++)
      {
         //         data[i] = random.nextDouble();
         data[i] = i;
      }
   }

   @Test public void testDataAddedToAndReadFromRingBufferBellowCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY - 1; i++)
      {
         instance.add(data[i]);
      }

      for (int i = 0; i < BUFFER_CAPACITY - 1; i++)
      {
         assertTrue(data[i] == instance.get(i));
      }
   }

   @Test public void testDataAddedToAndReadFromRingBufferAtCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY; i++)
      {
         instance.add(data[i]);
      }

      for (int i = 0; i < BUFFER_CAPACITY; i++)
      {
         assertTrue(data[i] == instance.get(i));
      }
      //      System.out.println("testDataAddedToAndReadFromRingBufferAtCapacity");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
      //      System.out.println();
   }

   @Test public void testDataAddedToAndReadFromRingBufferAtCapacityPlusOne()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 1; i++)
      {
         instance.add(data[i]);
      }

      //      System.out.println("testDataAddedToAndReadFromRingBufferAtCapacityPlusOne");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
      //      System.out.println();

      for (int i = 1; i < BUFFER_CAPACITY + 1; i++)
      {
         assertTrue(data[i] == instance.get(i - 1));
      }
   }

   @Test public void testDataAddedToAndReadFromRingBufferAtCapacityPlusTwo()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 2; i++)
      {
         instance.add(data[i]);
      }

      //      System.out.println("testDataAddedToAndReadFromRingBufferAtCapacityPlusTwo");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));

      System.out.println();
      for (int i = 2; i < BUFFER_CAPACITY + 2; i++)
      {
         assertTrue(data[i] == instance.get(i - 2));
      }
   }

   @Test public void testDataAddedToAndReadFromRingBufferExceedsCapacityByLots()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 10; i++)
      {
         instance.add(data[i]);
      }

      //      System.out.println("testDataAddedToAndReadFromRingBufferExceedsCapacityByLots");
      //      System.out.println("Reference: "+ Arrays.toString(data));
      //      System.out.println("Buffer content: "+Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
      //      System.out.println();

      for (int i = 10; i < BUFFER_CAPACITY + 10; i++)
      {
         assertTrue(data[i] == instance.get(i - 10));
      }
   }

   @Test public void testDataAddedInBatchesToAndReadFromRingBufferAtCapacityPlusOne()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      instance.add(data[0]);
      //      instance.add(data, 0, 1);
      instance.add(data, 1, BUFFER_CAPACITY);

      //
      //      System.out.println("testDataAddedInBatchesToAndReadFromRingBufferAtCapacityPlusOne");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.getBuffer()));
      //      System.out.println("readPosition: "+ instance.getReadPosition());
      //      System.out.println("fillcount: "+ instance.getFillCount());
      //      System.out.println();

      for (int i = 1; i < BUFFER_CAPACITY + 1; i++)
      {
         double temp = instance.get(i - 1);
         //         System.out.println("index =  "+i+" data "+data[i] +" buffer "+ temp);
         assertTrue(data[i] == temp);
      }
   }

   @Test public void testDataAddedInBatchesToAndReadFromRingBufferAtCapacityPlusTwo()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      instance.add(data, 0, 2);
      instance.add(data, 2, BUFFER_CAPACITY);

      for (int i = 2; i < BUFFER_CAPACITY + 2; i++)
      {
         assertTrue(data[i] == instance.get(i - 2));
      }
   }

   @Test public void testDataAddedInBatchesToAndReadFromRingBufferExceedsCapacityByLots()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      instance.add(data, 0, 10);
      instance.add(data, 10, BUFFER_CAPACITY);

      for (int i = 10; i < BUFFER_CAPACITY + 10; i++)
      {
         assertTrue(data[i] == instance.get(i - 10));
      }
   }

   @Test public void testDataAddedToAndReadBatchFromRingBufferAtCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY; i++)
      {
         instance.add(data[i]);
      }

      double[] dataCopy = instance.get(0, BUFFER_CAPACITY);

      //      System.out.println("testDataAddedToAndReadFromRingBufferWayBeyondCapacity");
      //      System.out.println("Data copy: "+ Arrays.toString(dataCopy));
      //      System.out.println("Data :"+ Arrays.toString(data));
      //      System.out.println();
      for (int i = 3; i < BUFFER_CAPACITY; i++)
      {
         assertTrue(data[i] == dataCopy[i]);
      }
   }

   @Test public void testDataAddedToAndReadBatchFromRingBufferAtCapacityPlusOne()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 1; i++)
      {
         instance.add(data[i]);
      }

      double[] dataCopy = instance.get(0, BUFFER_CAPACITY);

      //      System.out.println("testDataAddedToAndReadBatchFromRingBufferWayBeyondCapacity");
      //      System.out.println("Data copy: "+ Arrays.toString(dataCopy));
      //      System.out.println("Data :"+ Arrays.toString(data));
      //      System.out.println();

      for (int i = 1; i < BUFFER_CAPACITY + 1; i++)
      {
         assertTrue(data[i] == dataCopy[i - 1]);
      }
   }

   @Test public void testDataAddedToAndReadBatchFromRingBufferBeyondCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 10; i++)
      {
         instance.add(data[i]);
      }

      //      System.out.println("testDataAddedToAndReadFromRingBufferBeyondCapacity");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
      //
      //      System.out.println();
      for (int i = 10; i < BUFFER_CAPACITY + 10; i++)
      {
         assertTrue(data[i] == instance.get(i - 10));
      }
   }

   @Test public void testDataAddedToAndReadBatchFromRingBufferWayBeyondCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 10; i++)
      {
         instance.add(data[i]);
      }

      //      System.out.println("testDataAddedToAndReadFromRingBufferWayBeyondCapacity");
      //      System.out.println(Arrays.toString(data));
      //      System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
      //
      //      System.out.println();
      for (int i = 10; i < BUFFER_CAPACITY + 10; i++)
      {
         assertTrue(data[i] == instance.get(i - 10));
      }
   }

   @Test public void testAccessToDataAtIndexBeyondReadPositionWhenBufferIsFull()
   {
      for (int index = 0; index < BUFFER_CAPACITY; index++)
      {
         int dataBeyondCapacity = 10;
         int dataToWrite = BUFFER_CAPACITY + dataBeyondCapacity;
         instance = new RingBufferDouble(BUFFER_CAPACITY);
         for (int i = 0; i < dataToWrite; i++)
         {
            instance.add(data[i]);
         }

         //         System.out.println("testAccessToDataAtIndexBeyondReadPositionWhenBufferIsFull");
         //         System.out.println("Read Position: " + instance.getReadPosition());
         //         System.out.println(Arrays.toString(data));
         //         System.out.println(Arrays.toString(instance.get(0, BUFFER_CAPACITY)));
         //         System.out.println("instance.get(" + index + "): " + instance.get(index));
         //         System.out.println("data[" + (dataToWrite - (BUFFER_CAPACITY - index)) + "]: " + data[dataToWrite - (BUFFER_CAPACITY - index)]);

         //      assertTrue(instance.getReadPosition() == data[dataToWrite - dataBeyondCapacity]);
         assertTrue(instance.get(index) == data[dataToWrite - (BUFFER_CAPACITY - index)]);
      }
   }

   // TODO testAccessToDataBetweenLowerAndUpperIndexBeyondReadIndexWhenBufferIsFull

   @Test public void testAccessToDataBetweenLowerAndUpperIndexBeyondReadIndexWhenBufferIsFull()
   {
      int lowerIndex = 0;
      int upperIndex = 12;
      int dataBeyondCapacity = 10;
      int dataToWrite = BUFFER_CAPACITY + dataBeyondCapacity;
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < dataToWrite; i++)
      {
         instance.add(data[i]);
      }

      double[] dataCopy = instance.get(lowerIndex, upperIndex);
      for (int i = 0; i < dataCopy.length; i++)
      {
         assertTrue(data[dataToWrite - (BUFFER_CAPACITY - i)] == dataCopy[i]);
      }
   }

   @Test public void testOverwriteCountBelowCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY - 1; i++)
      {
         instance.add(i);
      }

      assertTrue(instance.getOverwriteCount() == 0);
   }

   @Test public void testOverwriteCountAtCapacity()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY; i++)
      {
         instance.add(i);
      }

      assertTrue(instance.getOverwriteCount() == 1);
   }

   @Test public void testOverwriteCountAt1()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY + 1; i++)
      {
         instance.add(i);
      }

      assertTrue(instance.getOverwriteCount() == 1);
   }

   @Test public void testOverwriteCountAt11()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY * 11; i++)
      {
         instance.add(i);
      }

      assertTrue(instance.getOverwriteCount() == 11);
   }

   @Test public void testOverwriteCountAt11andMore()
   {
      instance = new RingBufferDouble(BUFFER_CAPACITY);
      for (int i = 0; i < BUFFER_CAPACITY * 11 + BUFFER_CAPACITY - 1; i++)
      {
         instance.add(i);
      }

      assertTrue(instance.getOverwriteCount() == 11);
   }

}
