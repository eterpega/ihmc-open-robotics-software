package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.javaFXToolkit.framework.data.dataStructures.RingBufferDouble;

import java.util.Arrays;

/**
 * Created by amoucheboeuf on 7/5/16.
 */
public class TimeDataPlotVariableTest
{

   //An ordered set of data

   double[] data = new double[]{0.0, 0.5, 5.0, 8.0, 9.0, 9.5};
   double[] bigArray = new double[10000000];
   double[] smallArray = new double[]{0.0, 0.5};

   private TimeDataPlotVariable timeDataPlotVariable;
   private TimeDataPlotVariable timeDataPlotVariableSmallArray;
   private TimeDataPlotVariable timeDataPlotVariableBigArray;

   @Before public void setUp()
   {
      // simple initialization
      for (int i = 0; i < bigArray.length; i++)
      {
         bigArray[i] = i;
      }

      RingBufferDouble ringBufferDouble = new RingBufferDouble(data.length);
      ringBufferDouble.add(data);
      timeDataPlotVariable = new TimeDataPlotVariable("TestTimeDataPlot", ringBufferDouble);

      ringBufferDouble = new RingBufferDouble(smallArray.length);
      ringBufferDouble.add(smallArray);
      timeDataPlotVariableSmallArray = new TimeDataPlotVariable("TestTimeDataPlotSmallArray", ringBufferDouble);

      ringBufferDouble = new RingBufferDouble(bigArray.length);
      ringBufferDouble.add(bigArray);
      timeDataPlotVariableBigArray = new TimeDataPlotVariable("TestTimeDataPlotBigArray", ringBufferDouble);
   }

   @Test public void testFindTimestampInDataBufferBelowLowerBound()
      {
         // Test lower limit
         try
         {
            int index = timeDataPlotVariable.getTimestampClosestBufferIndex(-1);
            fail();
         }
         catch (Exception e)
         {
            // exception expected: good
         }
      }

      @Test public void testFindTimestampInDataBufferAboveUpperBound()
      {
         // Test upper limit
         try
         {
            int index = timeDataPlotVariable.getTimestampClosestBufferIndex(Double.MAX_VALUE);
            fail();
         }
         catch (Exception e)
         {
            // exception expected: good
         }

   }

   // approximate value, same distance from both indices mapped values, should return left index by default
   @Test public void testFindTimestampInDataBufferWithExactValueAsParameter()
   {
      for (int i = 0; i < data.length; i++)
      {
         int index = timeDataPlotVariable.getTimestampClosestBufferIndex(data[i]);
         assertEquals(index, i);
      }
   }

   @Test public void testFindTimestampInDataBufferWithApproximativeValueWithinRange()
   {
      for (int i = 0; i < data.length-1; i++)
      {
         System.out.println("-----");
         System.out.println("INPUT "+ (data[i] + 0.05));
         int index =  timeDataPlotVariable.getTimestampClosestBufferIndex(data[i] + 0.05);
         System.out.println("OUTPUT data #: "+ i +" val = "+ data[i]+" val approx =  "+ (data[i] + 0.05) + " index found "+ index);
         assertEquals(i, index);
         System.out.println("-----");

      }
   }

   @Test public void testFindTimestampInDataBufferWithApproximativeValueWithinRangeInSmallArray()
   {
      for (int i = 0; i < smallArray.length -1; i++)
      {
         int index =  timeDataPlotVariableSmallArray.getTimestampClosestBufferIndex(smallArray[i] + 0.05);
         assertEquals(i, index);
      }
   }


   @Test public void testFindIndexValueInBufferWithinRangeInBigArrayDeltaBelowLimit()
   {
      int step = 1000;
      double delta = 0.5;
      for (int i = 0; i < 10;  i++) // Looking for 10 values int the array
      {
         int j = i * step;
         int index =  timeDataPlotVariableBigArray.getTimestampClosestBufferIndex(bigArray[j] + delta);
         assertEquals(j, index);
      }
   }

   @Test public void testFindIndexValueInBufferWithinRangeInBigArrayDeltaAboveLimit()
   {
      int step = 1000;
      double delta = 0.6;
      for (int i = 0; i < 10;  i++) // Looking for 10 values int the array
      {
         int j = i * step;
         int index =  timeDataPlotVariableBigArray.getTimestampClosestBufferIndex(bigArray[j] + delta);
         assertEquals(j+1, index);
      }
   }




}