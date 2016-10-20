package us.ihmc.robotbuilder.util;

import javafx.beans.InvalidationListener;
import javafx.beans.property.*;
import javafx.beans.value.ChangeListener;
import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.List;
import org.junit.Test;

import java.util.Collections;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;
import static us.ihmc.robotbuilder.util.FunctionalObservableValue.combineLatest;
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
   public void testFlatMapConvertsNewValuesFromInnerObservables()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      SimpleIntegerProperty inner = new SimpleIntegerProperty(20);
      FunctionalObservableValue<Number> mappedObservable = of(base).flatMap(x -> inner);

      assertEquals(20, (int)mappedObservable.getValue());
      inner.setValue(30);
      assertEquals(30, (int)mappedObservable.getValue());
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
   public void testCombineLatestDoesNotFireWhenNoValuesWereSet()
   {
      IntegerProperty property1 = new SimpleIntegerProperty();
      StringProperty property2 = new SimpleStringProperty();
      combineLatest(property1, property2)
            .consume(pair -> fail("Should not be called"));
   }

   @Test
   public void testCombineLatestProducesCorrectPairs()
   {
      IntegerProperty property1 = new SimpleIntegerProperty(1);
      StringProperty property2 = new SimpleStringProperty("A");

      FunctionalObservableValue<Tuple2<Number, String>> combined = combineLatest(property1, property2);
      assertEquals(Tuple.of(1, "A"), combined.getValue());
      property1.setValue(2);
      assertEquals(Tuple.of(2, "A"), combined.getValue());
      property2.setValue("B");
      assertEquals(Tuple.of(2, "B"), combined.getValue());
      property1.setValue(3);
      assertEquals(Tuple.of(3, "B"), combined.getValue());
   }

   @Test
   public void testAvoidCyclesIsNotCalledMultipleTimesInACycle()
   {
      IntegerProperty property1 = new SimpleIntegerProperty(1);
      StringProperty property2 = new SimpleStringProperty("0");

      property1.addListener((observable, oldValue, newValue) -> property2.setValue(newValue.toString()));

      FunctionalObservableValue<String> cycleBreaker = of(property2).avoidCycles();
      cycleBreaker.consume(newValue -> property1.setValue(Integer.parseInt(newValue) + 1));

      property1.setValue(2);
      assertEquals("3", property2.getValue());
   }

   @Test
   public void testNarrowFiltersOutNonNarrowableValues()
   {
      ObjectProperty<Object> property = new SimpleObjectProperty<>("Test");

      FunctionalObservableValue<String> narrowed = of(property).narrow(String.class);
      assertEquals("Test", narrowed.getValue());

      property.setValue(1234);
      assertEquals("Test", narrowed.getValue());

      property.setValue("Test2");
      assertEquals("Test2", narrowed.getValue());
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
