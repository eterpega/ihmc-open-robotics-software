package us.ihmc.robotbuilder.util;

import javafx.beans.InvalidationListener;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

public class NoCyclePropertyTest
{
   @Test
   public void testPropertyBehavesLikeAStandardPropertyWhenNoCyclesArePresent()
   {
      NoCycleProperty<Number> property = noCycle(new SimpleIntegerProperty(0));

      AtomicInteger changeCalled = new AtomicInteger(0);
      AtomicInteger invalidationCalled = new AtomicInteger(0);
      property.addListener((observable, oldValue, newValue) ->
                           {
                              assertEquals(observable, property);
                              assertEquals(oldValue.intValue() + 1, newValue);
                              changeCalled.incrementAndGet();
                           });

      property.addListener(observable ->
                           {
                              assertEquals(observable, property);
                              invalidationCalled.incrementAndGet();
                           });

      property.setValue(1);
      property.setValue(2);
      property.setValue(3);

      assertEquals(3, property.getValue());
      assertEquals(3, changeCalled.get());
      assertEquals(3, invalidationCalled.get());
   }

   @Test
   public void testCyclesAreBrokenCorrectly()
   {
      NoCycleProperty<Number> property = noCycle(new SimpleIntegerProperty(0));
      property.addListener((observable, oldValue, newValue) -> property.setValue(newValue.intValue() + 1));
      property.addListener(observable -> property.setValue(property.getValue().intValue() + 1));

      property.setValue(1);
      assertEquals(1, property.getValue());
   }


   @Test
   public void testListenersAreRemovedCorrectly()
   {
      NoCycleProperty<Number> property = noCycle(new SimpleIntegerProperty(0));

      int[] changeCalled = {0};
      int[] invalidationCalled = {0};
      ChangeListener<Number> changeListener = (observable, oldValue, newValue) -> ++changeCalled[0];
      InvalidationListener invalidationListener = observable -> {
         ++invalidationCalled[0];
         System.out.println("Invalidation called " + invalidationCalled[0]);
      };
      property.addListener(changeListener);
      property.addListener(invalidationListener);

      property.setValue(1);
      assertEquals(1, changeCalled[0]);
      assertEquals(1, invalidationCalled[0]);

      property.removeListener(invalidationListener);
      property.setValue(2);
      assertEquals(2, changeCalled[0]);
      assertEquals(1, invalidationCalled[0]);

      property.removeListener(changeListener);
      property.setValue(3);
      assertEquals(3, property.getValue());
      assertEquals(2, changeCalled[0]);
      assertEquals(1, invalidationCalled[0]);
   }

   @Test
   public void testRemoveNonExistingListener()
   {
      NoCycleProperty<Number> property = noCycle(new SimpleIntegerProperty(0));
      property.removeListener((observable, oldValue, newValue) -> fail("Should not be called"));
      property.removeListener(observable -> fail("Should not be called"));
   }

   @Test
   public void testBindUnbindGetDelegatedProperly()
   {
      NoCycleProperty<Number> property1 = noCycle(new SimpleIntegerProperty(0));
      NoCycleProperty<Number> property2 = noCycle(new SimpleIntegerProperty(0));

      property1.bind(property2);
      property2.setValue(1);
      assertTrue(property1.isBound());
      assertEquals(1, property1.getValue());

      property1.unbind();
      property2.setValue(2);
      assertEquals(1, property1.getValue());

      property1.bindBidirectional(property2);
      property2.setValue(3);
      assertEquals(3, property1.getValue());
      assertEquals(3, property2.getValue());
      property1.setValue(4);
      assertEquals(4, property1.getValue());
      assertEquals(4, property2.getValue());

      property1.unbindBidirectional(property2);
      property1.setValue(5);
      property2.setValue(6);
      assertEquals(5, property1.getValue());
      assertEquals(6, property2.getValue());
   }

   @Test
   public void testBeanAndNameArePropagatedCorrectly()
   {
      Object bean = new Object();
      SimpleIntegerProperty intProperty = new SimpleIntegerProperty(0, "TestName") {
         @Override public Object getBean()
         {
            return bean;
         }
      };
      NoCycleProperty<Number> property1 = noCycle(intProperty);

      assertEquals("TestName", property1.getName());
      assertEquals(bean, property1.getBean());
   }

   @Test
   public void testNoCycleOfNoCycleReturnsTheSameInstance()
   {
      Property<Number> property = noCycle(new SimpleIntegerProperty(0));
      assertEquals(property, noCycle(property));
   }
}
