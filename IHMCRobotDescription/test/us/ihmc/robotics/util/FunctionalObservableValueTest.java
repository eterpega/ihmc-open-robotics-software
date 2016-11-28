package us.ihmc.robotics.util;

import javafx.beans.InvalidationListener;
import javafx.beans.property.*;
import javafx.beans.value.ChangeListener;
import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.List;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.util.FunctionalObservableValue;

import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;
import static us.ihmc.robotics.util.FunctionalObservableValue.join;
import static us.ihmc.robotics.util.FunctionalObservableValue.combineLatest;
import static us.ihmc.robotics.util.FunctionalObservableValue.functional;

public class FunctionalObservableValueTest
{

   @Test
   public void testMapConvertsBaseValue()
   {
      assertEquals("10", functional(new SimpleIntegerProperty(10))
            .map(Object::toString)
            .getValue());
   }

   @Test
   public void testMapConvertsNewValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<String> mapped = functional(base).map(Object::toString);
      base.setValue(20);
      assertEquals("20", mapped.getValue());
      base.setValue(30);
      assertEquals("30", mapped.getValue());
   }

   @Test
   public void testFlatMapConvertsBaseValue()
   {
      assertEquals(10, functional(new SimpleIntegerProperty(10))
                                                  .flatMap(SimpleObjectProperty::new)
                                                  .getValue());

      assertEquals(10, functional(new SimpleIntegerProperty(10))
                                                .flatMapIterable(Collections::singletonList)
                                                .getValue());

      assertEquals(10, functional(new SimpleIntegerProperty(10))
                                                .flatMapOptional(Optional::of)
                                                .getValue());
   }

   @Test
   public void testFlatMapConvertsNewValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> mappedObservable = functional(base).flatMap(SimpleObjectProperty::new);
      FunctionalObservableValue<Number> mappedIterable = functional(base).flatMapIterable(Collections::singletonList);
      FunctionalObservableValue<Number> mappedOptional = functional(base).flatMapOptional(Optional::of);
      List<FunctionalObservableValue<Number>> allMapped = List.of(mappedObservable, mappedIterable, mappedOptional);


      base.setValue(20);
      allMapped.map(FunctionalObservableValue::getValue).forEach(val -> Assert.assertEquals(20, val));
      base.setValue(30);
      allMapped.map(FunctionalObservableValue::getValue).forEach(val -> Assert.assertEquals(30, val));
   }

   @Test
   public void testFlatMapConvertsNewValuesFromInnerObservables()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      SimpleIntegerProperty inner = new SimpleIntegerProperty(20);
      FunctionalObservableValue<Number> mappedObservable = functional(base).flatMap(x -> inner);

      assertEquals(20, (int)mappedObservable.getValue());
      inner.setValue(30);
      assertEquals(30, (int)mappedObservable.getValue());
   }

   @Test
   public void testFilterTrueKeepsAllValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = functional(base).filter(x -> true);

      assertEquals(10, filtered.getValue());

      base.setValue(20);
      assertEquals(20, filtered.getValue());
   }

   @Test
   public void testFilterFalseKeepsNothing()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = functional(base).filter(x -> false);

      assertNull(filtered.getValue());

      base.setValue(20);
      assertNull(filtered.getValue());
   }

   @Test
   public void testFilterKeepsTrueValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> filtered = functional(base).filter(x -> x.intValue() > 20);

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
      FunctionalObservableValue<Number> reduced = functional(base).reduce((x, y) -> y);

      assertEquals(10, reduced.getValue());
   }

   @Test
   public void testEmptyReducedContainsFirstValueAfterTheValueIsSet()
   {
      SimpleObjectProperty<String> base = new SimpleObjectProperty<>();
      FunctionalObservableValue<String> reduced = functional(base).reduce((x, y) -> y);

      assertNull(reduced.getValue());
      base.setValue("10");
      assertEquals("10", reduced.getValue());
   }

   @Test
   public void testReduceWithAdditionAccumulatesValues()
   {
      SimpleIntegerProperty base = new SimpleIntegerProperty(10);
      FunctionalObservableValue<Number> reduced = functional(base).reduce((x, y) -> x.intValue() + y.intValue());

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
         functional(base).map(Object::toString);
         functional(base).flatMap(SimpleObjectProperty::new).map(Object::toString);
         functional(base).flatMapIterable(Collections::singletonList).map(Object::toString);
         functional(base).flatMapOptional(Optional::of).map(Object::toString);
         functional(base).filter(x -> x.hashCode() > 0);
         functional(base).reduce((x, y) -> x.toString() + y.toString());
         functional(base).consume(x -> fail("Should not call consume on empty value"));
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
      functional(base).consume(x -> counter.addAndGet(x.intValue()));
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
      Assert.assertEquals(Tuple.of(1, "A"), combined.getValue());
      property1.setValue(2);
      Assert.assertEquals(Tuple.of(2, "A"), combined.getValue());
      property2.setValue("B");
      Assert.assertEquals(Tuple.of(2, "B"), combined.getValue());
      property1.setValue(3);
      Assert.assertEquals(Tuple.of(3, "B"), combined.getValue());
   }

   @Test
   public void testAvoidCyclesIsNotCalledMultipleTimesInACycle()
   {
      IntegerProperty property1 = new SimpleIntegerProperty(1);
      StringProperty property2 = new SimpleStringProperty("0");

      property1.addListener((observable, oldValue, newValue) -> property2.setValue(newValue.toString()));

      FunctionalObservableValue<String> cycleBreaker = functional(property2).avoidCycles();
      cycleBreaker.consume(newValue -> property1.setValue(Integer.parseInt(newValue) + 1));

      property1.setValue(2);
      assertEquals("3", property2.getValue());
      assertEquals("2", cycleBreaker.getValue());
   }

   @Test
   public void testAvoidCyclesDoesNothingWhenNoCycleIsPresent()
   {
      IntegerProperty property1 = new SimpleIntegerProperty(1);

      FunctionalObservableValue<Number> cycleBreaker = functional(property1).avoidCycles();

      property1.setValue(2);
      assertEquals(2, cycleBreaker.getValue());
      property1.setValue(3);
      assertEquals(3, cycleBreaker.getValue());
   }

   @Test
   public void testNarrowFiltersOutNonNarrowableValues()
   {
      ObjectProperty<Object> property = new SimpleObjectProperty<>("Test");

      FunctionalObservableValue<String> narrowed = functional(property).narrow(String.class);
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
      FunctionalObservableValue<Number> functional = functional(base);
      functional.addListener(emptyListener);
      functional.removeListener(emptyListener);
      functional.addListener(emptyInvalidationListener);
      functional.removeListener(emptyInvalidationListener);
      base.setValue(2);

      assertFalse(called[0]);
   }

   @Test
   public void testJoinContainsLastGivenObservableValueAfterCreation()
   {
      SimpleIntegerProperty property1 = new SimpleIntegerProperty(1);
      SimpleIntegerProperty property2 = new SimpleIntegerProperty(100);
      SimpleIntegerProperty property3 = new SimpleIntegerProperty(10000);

      FunctionalObservableValue<Number> joined = join(Arrays.asList(property1, property2, property3));
      assertEquals(10000, joined.getValue());

   }

   @Test
   public void testJoinFiresForAnyObservable()
   {
      SimpleIntegerProperty property1 = new SimpleIntegerProperty(1);
      SimpleIntegerProperty property2 = new SimpleIntegerProperty(100);
      SimpleIntegerProperty property3 = new SimpleIntegerProperty(10000);

      FunctionalObservableValue<Number> joined = join(Arrays.asList(property1, property2, property3));
      property1.setValue(2);
      assertEquals(2, joined.getValue());
      property2.setValue(101);
      assertEquals(101, joined.getValue());
      property3.setValue(10001);
      assertEquals(10001, joined.getValue());
   }
}
