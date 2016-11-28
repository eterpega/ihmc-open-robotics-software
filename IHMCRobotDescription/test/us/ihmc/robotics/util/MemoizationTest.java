package us.ihmc.robotics.util;

import org.junit.Test;

import java.util.function.Function;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.robotics.util.Memoization.memoized;

public class MemoizationTest
{
   @Test
   public void testMemoizedFunctionGivesTheSameResults()
   {
      Function<Object, String> memoizedToString = memoized(Object::toString);

      Object[] testObjects = {12346746, 546.456, true, "Test", new Object()};
      for (int i = 0; i < 10; i++)
      {
         for (Object testObject : testObjects)
         {
            assertEquals(testObject.toString(), memoizedToString.apply(testObject));
         }
      }
   }

   @Test
   public void testRepeatedCallsToMemoizedReturnTheSameInstance()
   {
      Function<Object, String> toString = Object::toString;
      Function<Object, String> memoizedToString = memoized(toString);
      Function<Object, String> memoizedToStringAgain = memoized(toString);
      assertTrue(memoizedToString == memoizedToStringAgain);

      Function<Object, String> previousMemoized = null;
      for (int i = 0; i < 10; i++)
      {
         Function<Object, String> nextMemoized = memoized(Object::toString);
         if (previousMemoized != null)
            assertTrue(previousMemoized == nextMemoized);
         previousMemoized = nextMemoized;
      }
   }

}
