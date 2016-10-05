package us.ihmc.robotbuilder.util;

import javafx.beans.InvalidationListener;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javaslang.collection.List;
import org.junit.Test;

import java.util.Collections;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;
import static us.ihmc.robotbuilder.util.FunctionalObservableValue.of;

public class FunctionalObservableValueTest
{

   @Test
   public void testMapConvertsBaseValue()
   {
      assertEquals("10", of(new SimpleIntegerProperty(10))
            .map(Object::toString)
            .getValue());
   }

   @Test
   public void testMapConvertsNewValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<String> mapped = of(base).map(Object::toString);
      base.setValue(20);
      assertEquals("20", mapped.getValue());
      base.setValue(30);
      assertEquals("30", mapped.getValue());
   }

   @Test
   public void testFlatMapConvertsBaseValue()
   {
      assertEquals(10, of(new SimpleIntegerProperty(10))
                                                  .flatMap(SimpleObjectProperty::new)
                                                  .getValue());

      assertEquals(10, of(new SimpleIntegerProperty(10))
                                                .flatMapIterable(Collections::singletonList)
                                                .getValue());

      assertEquals(10, of(new SimpleIntegerProperty(10))
                                                .flatMapOptional(Optional::of)
                                                .getValue());
   }

   @Test
   public void testFlatMapConvertsNewValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> mappedObservable = of(base).flatMap(SimpleObjectProperty::new);
      FunctionalObservableValue<Number> mappedIterable = of(base).flatMapIterable(Collections::singletonList);
      FunctionalObservableValue<Number> mappedOptional = of(base).flatMapOptional(Optional::of);
      List<FunctionalObservableValue<Number>> allMapped = List.of(mappedObservable, mappedIterable, mappedOptional);


      base.setValue(20);
      allMapped.map(FunctionalObservableValue::getValue).forEach(val -> assertEquals(20, val));
      base.setValue(30);
      allMapped.map(FunctionalObservableValue::getValue).forEach(val -> assertEquals(30, val));
   }

   @Test
   public void testFilterTrueKeepsAllValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = of(base).filter(x -> true);

      assertEquals(10, filtered.getValue());

      base.setValue(20);
      assertEquals(20, filtered.getValue());
   }

   @Test
   public void testFilterFalseKeepsNothing()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = of(base).filter(x -> false);

      assertNull(filtered.getValue());

      base.setValue(20);
      assertNull(filtered.getValue());
   }

   @Test
   public void testFilterKeepsTrueValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = of(base).filter(x -> x.intValue() > 20);

      assertNull(filtered.getValue());

      base.setValue(20);
      assertNull(filtered.getValue());

      base.setValue(30);
      assertEquals(30, filtered.getValue());
   }

   @Test
   public void testEmptyReduceContainsFirstValue()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> reduced = of(base).reduce((x, y) -> y);

      assertEquals(10, reduced.getValue());
   }

   @Test
   public void testEmptyReducedContainsFirstValueAfterTheValueIsSet()
   {
      SimpleObjectProperty<String> base = new SimpleObjectProperty<>();
      FunctionalObservableValue<String> reduced = of(base).reduce((x, y) -> y);

      assertNull(reduced.getValue());
      base.setValue("10");
      assertEquals("10", reduced.getValue());
   }

   @Test
   public void testReduceWithAdditionAccumulatesValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> reduced = of(base).reduce((x, y) -> x.intValue() + y.intValue());

      assertEquals(10, reduced.getValue());

      base.setValue(20);
      assertEquals(30, reduced.getValue());

      base.setValue(30);
      assertEquals(60, reduced.getValue());
   }

   @Test
   public void testReturnedObservablesDoNotFireWhenNoValueWasSet()
   {
      SimpleObjectProperty<Object> base = new SimpleObjectProperty<>();
      try
      {
         of(base).map(Object::toString);
         of(base).flatMap(SimpleObjectProperty::new).map(Object::toString);
         of(base).flatMapIterable(Collections::singletonList).map(Object::toString);
         of(base).flatMapOptional(Optional::of).map(Object::toString);
         of(base).filter(x -> x.hashCode() > 0);
         of(base).reduce((x, y) -> x.toString() + y.toString());
         of(base).consume(x -> fail("Should not call consume on empty value"));
      } catch (NullPointerException ex) {
         fail("Should not throw NPE");
         ex.printStackTrace();
      }
   }

   @Test
   public void testConsumeCalledForAllValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(1);
      AtomicInteger counter = new AtomicInteger();
      of(base).consume(x -> counter.addAndGet(x.intValue()));
      base.set(2);
      base.set(3);
      assertEquals(6, counter.get());
   }

   @Test
   public void testAddRemoveListeners()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(1);
      boolean[] called = new boolean[1];
      ChangeListener<Number> emptyListener = (observable, oldValue, newValue) -> called[0] = true;
      InvalidationListener emptyInvalidationListener = x -> called[0] = true;
      FunctionalObservableValue<Number> functional = of(base);
      functional.addListener(emptyListener);
      functional.removeListener(emptyListener);
      functional.addListener(emptyInvalidationListener);
      functional.removeListener(emptyInvalidationListener);
      base.setValue(2);

      assertFalse(called[0]);
   }
}
