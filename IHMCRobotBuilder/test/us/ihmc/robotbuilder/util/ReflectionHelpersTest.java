package us.ihmc.robotbuilder.util;

import javaslang.control.Try;
import org.junit.Test;
import us.ihmc.robotbuilder.util.ReflectionHelpers.*;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.*;
import static us.ihmc.robotbuilder.util.ReflectionHelpers.*;

public class ReflectionHelpersTest
{
   @Test
   public void testGetGenericParametersReturnsProperGenericParametersForAGetter() throws NoSuchMethodException
   {
      class TestClass
      {
         public List<String> getTest() { return null; };
      }

      List<Class<?>> generics = getGenericParameters(TestClass.class.getMethod("getTest"));
      assertEquals(String.class, generics.get(0));
   }

   @Test
   public void testGetGenericParametersReturnsAnEmptyListForNonGenericTypes() throws NoSuchMethodException
   {
      class TestClass
      {
         public String getTest() { return null; }
      }

      List<Class<?>> generics = getGenericParameters(TestClass.class.getMethod("getTest"));
      assertTrue(generics.isEmpty());
   }

   @Test
   public void testListGettersReturnsAllGetters()
   {
      class TestClass
      {
         public List<String> getList() { return null; }
         public String getString() { return null; }
      }

      List<Method> getters = listGetters(TestClass.class);
      assertEquals(3, getters.size());

      List<String> getterNames = getters.stream().map(Method::getName).sorted().collect(Collectors.toList());
      assertEquals(Arrays.asList("getClass", "getList", "getString"), getterNames);
   }

   private static abstract class TestInterfaceWithABridgeMethod
   {
      public abstract Object getObject();
   }

   private static class TestClassWithFakeGetters extends TestInterfaceWithABridgeMethod
   {
      public static String getString() { return null; }
      public String getWithArgument(String arg) { return arg; }
      private String getPrivate() { return null; }

      @Override
      public String getObject() { return null; }
   }

   @Test
   public void testFakeGettersDoNotGetListed()
   {
      List<Method> getters = listGetters(TestClassWithFakeGetters.class);
      getters.sort(Comparator.comparing(Method::getName));
      assertEquals(2, getters.size());
      assertEquals("getClass", getters.get(0).getName());
      assertEquals("getObject", getters.get(1).getName());
      assertEquals(String.class, getters.get(1).getReturnType());
   }

   @Test
   public void testPropertiesForBeanWithImmutableSetters()
   {
      class TestClass
      {
         public String getString() { return null; }
         public TestClass withString(String string) { return this; }

         public int getInteger() { return 0; }
         public TestClass withInteger(int value) { return this; }

         public List<String> getList() { return null; }
         public TestClass withList(List<String> value) { return this; }

         public double getShouldNotBePresent() { return 0; }

         public double getHasBadWither() { return 0; }
         public TestClass withBadWither(/*missing param*/) { return this; }

         public double getHasBadWither2() { return 0; }
         public TestClass withBadWither2(int too, int many, int params) { return this; }
      }

      List<ImmutableReflectionProperty<TestClass>> properties = propertiesForBeanWithImmutableSetters(TestClass.class);
      assertEquals(3, properties.size());

      properties.sort(Comparator.comparing(ImmutableReflectionProperty::getName));

      assertEquals("Integer", properties.get(0).getName());
      assertEquals("List", properties.get(1).getName());
      assertEquals("String", properties.get(2).getName());

      assertEquals(int.class, properties.get(0).getType());
      assertEquals(List.class, properties.get(1).getType());
      assertEquals(String.class, properties.get(2).getType());

      assertEquals(0, properties.get(0).getGenericTypes().size());
      assertEquals(1, properties.get(1).getGenericTypes().size());
      assertEquals(0, properties.get(0).getGenericTypes().size());

      for (ImmutableReflectionProperty<TestClass> property : properties)
      {
         assertNotNull(property.getGetter());
         assertNotNull(property.getSetter());
      }

      assertEquals(String.class, properties.get(1).getGenericTypes().get(0));
   }

   @Test
   public void testPropertiesForBeanWithMutableSetters()
   {
      class TestClass
      {
         public String getString() { return null; }
         public void setString(String string) {  }

         public int getInteger() { return 0; }
         public void setInteger(int value) {  }

         public List<String> getList() { return null; }
         public TestClass setList(List<String> list) { return this; } // return type should not matter

         public double getShouldNotBePresent() { return 0; }

         public double getHasBadSetter() { return 0; }
         public void setBadSetter(/*missing param*/) { }

         public double getHasBadSetter2() { return 0; }
         public void setBadSetter2(int too, int many, int params) { }
      }

      List<MutableReflectionProperty<TestClass>> properties = propertiesForBeanWithMutableSetters(TestClass.class);
      assertEquals(3, properties.size());

      properties.sort(Comparator.comparing(MutableReflectionProperty::getName));

      assertEquals("Integer", properties.get(0).getName());
      assertEquals("List", properties.get(1).getName());
      assertEquals("String", properties.get(2).getName());

      assertEquals(int.class, properties.get(0).getType());
      assertEquals(List.class, properties.get(1).getType());
      assertEquals(String.class, properties.get(2).getType());

      assertEquals(0, properties.get(0).getGenericTypes().size());
      assertEquals(1, properties.get(1).getGenericTypes().size());
      assertEquals(0, properties.get(0).getGenericTypes().size());

      for (MutableReflectionProperty<TestClass> property : properties)
      {
         assertNotNull(property.getGetter());
         assertNotNull(property.getSetter());
      }

      assertEquals(String.class, properties.get(1).getGenericTypes().get(0));
   }

   static class ConstructorTestClass
   {
      public ConstructorTestClass(String string) {}
      public ConstructorTestClass(String string, int Int, List<String> list) { }

      public String getString() { return null; }
      public int getInt() { return 0; }
      public List<String> getList() { return null; }

      public String getListConcatenated() { return getList().stream().collect(Collectors.joining()); }
   }

   @Test
   public void testPropertiesForBeanWithGettersAndConstructor()
   {
      if (!ConstructorTestClass.class.getDeclaredConstructors()[0].getParameters()[0].isNamePresent())
         fail("Please compile with -parameters so that constructor parameter names are available via reflection.\n"
                    + "In Eclipse please enable 'Store information about method parameters (usable via reflection)' under Window -> Preferences -> Java -> Compiler.\n"
                    + "In IntelliJ use Project Settings -> Java Compiler -> Additional commandline parameters -> -parameters.");

      List<ImmutableReflectionProperty<ConstructorTestClass>> properties = propertiesForImmutableBeanWithBuilderConstructor(ConstructorTestClass.class);
      assertEquals(3, properties.size());

      properties.sort(Comparator.comparing(ImmutableReflectionProperty::getName));

      assertEquals("Int", properties.get(0).getName());
      assertEquals("List", properties.get(1).getName());
      assertEquals("String", properties.get(2).getName());

      assertEquals(int.class, properties.get(0).getType());
      assertEquals(List.class, properties.get(1).getType());
      assertEquals(String.class, properties.get(2).getType());

      assertEquals(0, properties.get(0).getGenericTypes().size());
      assertEquals(1, properties.get(1).getGenericTypes().size());
      assertEquals(0, properties.get(0).getGenericTypes().size());

      for (ImmutableReflectionProperty<ConstructorTestClass> property : properties)
      {
         assertNotNull(property.getGetter());
         assertNotNull(property.getSetter());
      }

      assertEquals(String.class, properties.get(1).getGenericTypes().get(0));
   }

   private static class SimpleConstructorTestClass
   {
      private final int integer;
      private final String string;

      public SimpleConstructorTestClass(int integer, String string)
      {
         this.integer = integer;
         this.string = string;
      }

      public int getInteger()
      {
         return integer;
      }

      public String getString()
      {
         return string;
      }
   }

   @Test
   public void testConstructorPropertiesReturnModifiedObject()
   {
      List<ImmutableReflectionProperty<SimpleConstructorTestClass>> properties = propertiesForImmutableBeanWithBuilderConstructor(SimpleConstructorTestClass.class);

      assertEquals(2, properties.size());

      SimpleConstructorTestClass baseObject = new SimpleConstructorTestClass(0, "test");
      Try<SimpleConstructorTestClass> tryModifiedObject = properties.get(0).getSetter().withValue(baseObject, 1);

      assertTrue(tryModifiedObject.isSuccess());

      assertEquals(1, tryModifiedObject.get().getInteger());

      // Make sure the existing object is not modified
      assertEquals(0, baseObject.getInteger());
      assertEquals(0, (int)properties.get(0).getGetter().getValue(baseObject).getOrElse(-1));

      // Make sure the unmodified attribute stays the same
      assertEquals("test", baseObject.getString());
      assertEquals("test", properties.get(1).getGetter().getValue(baseObject).getOrElse(""));
   }
}
