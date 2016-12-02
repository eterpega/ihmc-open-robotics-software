package us.ihmc.javaFXToolkit.charts.dataModel.dataManagers;

import us.ihmc.javaFXToolkit.framework.data.dataStructures.DoubleBufferReader;
import us.ihmc.robotics.MathTools;

/**
 * Created by amoucheboeuf on 7/5/16.
 */
public class TimeDataPlotVariable extends PlotVariable
{

   public TimeDataPlotVariable(String name, DoubleBufferReader dataHolder)
   {
      super(name, dataHolder);
   }

   public double getCurrentTimestamp()
   {
      if(dataHolder.getFillCount() > 0)
         return dataHolder.get(dataHolder.getFillCount() - 1);
      else
         return 0.0;
   }

   public double getOldestTimestamp()
   {
      if(dataHolder.getFillCount() > 0)
         return dataHolder.get(0);
      else
         return 0.0; // TODO see if this is a good
   }


   /**
    *
    * @param timestamp
    * @return
    */
   public synchronized int getTimestampClosestBufferIndex(double timestamp)
   {

      double minTimeStamp = getOldestTimestamp();
      double maxTimeStamp = getCurrentTimestamp();

      if (timestamp < 0 || timestamp > maxTimeStamp || timestamp < minTimeStamp)
      {
         throw new IndexOutOfBoundsException(
               "The timestamp value entered \" " + timestamp + " \" as a parameter is invalid. It should be positive or null and comprised between "
                     + minTimeStamp + " and " + maxTimeStamp);
      }

      return closestIndexToDoubleEntryBinarySearch(dataHolder.get(0, dataHolder.getFillCount()), timestamp, 0, dataHolder.getFillCount() - 1);
   }

   /**
    * TODO move to another class
    *  Modification of the original binary search algorithm. Looks for the direct neighbors indices of the current middle index if they exist.
    *  Return the index in the array that maps the closest to the desired value.
    *  Finds the exact or closest value within 20 steps or less for a array of 10E7 entries.
    * @param array
    * @param value
    * @param lowerBound
    * @param upperBound
    * @return
    */
   public static int closestIndexToDoubleEntryBinarySearch(double[] array, double value, int lowerBound, int upperBound)
   {
      if (upperBound >= array.length)
         throw new IllegalArgumentException("Upper bound index is greater than array length");

      if (lowerBound > upperBound)
      {
         throw new IllegalArgumentException("Lower bound index is greater than upper bound index");
      }

      double leftNeighbor = Double.MAX_VALUE;
      double rightNeighbor = - Double.MAX_VALUE;

      int midPoint = (upperBound - lowerBound) / 2 + lowerBound;

      if (midPoint > lowerBound)
         leftNeighbor = array[midPoint - 1];

      if (midPoint < upperBound)
         rightNeighbor = array[midPoint + 1];

      // First Check if value is equal to midpoint value
      if (MathTools.epsilonEquals(array[midPoint], value, 0.000001))
      {
         return midPoint;
      }
      // Not equal to midpoint but between midpoint and its left neighbor
      else if ((leftNeighbor <= value && value < array[midPoint]))
      {
         // Return index with min distance
         if (Math.abs(value - leftNeighbor) <= Math.abs(array[midPoint] - value))
         {
            return midPoint - 1;
         }
         else
         {
            return midPoint;
         }
      }
      // Not equal to midpoint but between midpoint and its left neighbor
      else if ((array[midPoint] < value && value <= rightNeighbor))
      {
         // Keep smallest index by default
         if (Math.abs(value - rightNeighbor) < Math.abs(array[midPoint] - value))
         {
            return midPoint + 1;
         }
         else
         {
            return midPoint;
         }
      }
      // Else continue splitting the array looking for solution
      else if (array[midPoint] > value)
      {
         return closestIndexToDoubleEntryBinarySearch(array, value, lowerBound, midPoint - 1);
      }
      else
      {
         return closestIndexToDoubleEntryBinarySearch(array, value, midPoint + 1, upperBound);
      }
   }

}

