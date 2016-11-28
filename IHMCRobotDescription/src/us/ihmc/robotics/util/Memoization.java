package us.ihmc.robotics.util;

import java.util.*;
import java.util.function.Function;

/**
 * Collection of methods that memoize (cache) existing functions
 * to avoid expensive re-computation.
 */
public class Memoization
{
   private static final Map<Function<?, ?>, Function<?, ?>> memoizedFunctions = new WeakHashMap<>();

   /**
    * Memoizes a function. The memoized function remembers the outputs
    * for the given inputs and does not re-evaluate the base function
    * when called again with the same argument. A {@link WeakHashMap}
    * is used as the caching mechanism which means that the caching does
    * not prevent the arguments from being garbage collected.
    * When called multiple times given the same function, the same memoized
    * function is returned (i.e. this function is itself memoized). This
    * allows for an idiomatic way of using this method:
    * ...
    * value = memoized(MyClass::expensive).apply(50);
    * ...
    * @param baseFn base function to memoize
    * @param <T> argument type
    * @param <R> result type
    * @return result
    */
   public static <T, R> Function<T, R> memoized(Function<T, R> baseFn)
   {
      //noinspection unchecked
      return (Function<T, R>)memoizedFunctions.computeIfAbsent(baseFn, function ->
      {
         Map<T, R> cachedValues = new WeakHashMap<>();
         return (T t) -> cachedValues.computeIfAbsent(t, baseFn);
      });
   }
}
