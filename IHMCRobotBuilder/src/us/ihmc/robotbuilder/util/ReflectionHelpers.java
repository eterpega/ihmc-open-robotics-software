package us.ihmc.robotbuilder.util;

import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Methods facilitating the use of reflection in Java.
 */
public class ReflectionHelpers
{
   /**
    * Returns generic parameters for a field represented by the given getter.
    * For example returns [Integer, String] for Map&lt;Integer, String&gt; getSomeMap().
    * An empty list is returned if the return type is not generic.
    * @param getter field getter
    * @return generic parameters
    */
   public static List<Class<?>> getGenericParameters(Method getter)
   {
      Type returnType = getter.getGenericReturnType();
      if (returnType instanceof ParameterizedType)
      {
         ParameterizedType genericType = (ParameterizedType)returnType;
         return Arrays.stream(genericType.getActualTypeArguments())
                      .filter(type -> type instanceof Class)
                      .map(type -> (Class<?>)type)
                      .collect(Collectors.toList());
      }
      return Collections.emptyList();
   }
}
