package us.ihmc.robotbuilder.util;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javaslang.Tuple;
import javaslang.Tuple2;
import org.jetbrains.annotations.NotNull;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Enhances the {@link ObservableValue} interface with functional methods such as map/filter etc.
 */
public final class FunctionalObservableValue<T> implements ObservableValue<T>
{
   private final ObservableValue<T> base;

   /**
    * Create a new functional observable value from a basic JavaFX {@link ObservableValue}.
    * @param base base value
    */
   public FunctionalObservableValue(@NotNull ObservableValue<T> base)
   {
      this.base = base;
   }

   /**
    * Create a new functional observable value from a basic JavaFX {@link ObservableValue}.
    * Syntactic sugar for {@link FunctionalObservableValue#FunctionalObservableValue(ObservableValue)}
    * @param base base value
    * @param <T> base value type
    * @return functional wrapper for the base value
    */
   public static <T> FunctionalObservableValue<T> functional(@NotNull ObservableValue<T> base)
   {
      return new FunctionalObservableValue<>(base);
   }

   /**
    * Creates a new functional observable value by combining the latest values
    * of the two given observables. A syntactic sugar around {@link #combineLatest(ObservableValue)}.
    * @param first first observable
    * @param second second observable
    * @param <T> first observable type
    * @param <U> second observable type
    * @return combined observables
    */
   public static <T, U> FunctionalObservableValue<Tuple2<T, U>> combineLatest(@NotNull ObservableValue<T> first, @NotNull ObservableValue<? extends U> second)
   {
      return functional(first).combineLatest(second);
   }

   /**
    * Joins multiple observable values to a single one that fires whenever join of the given
    * observables fire.
    * @param observables observables to monitor
    * @param <T> observable type
    * @return observable joining all the given observables
    */
   public static <T> FunctionalObservableValue<T> join(Iterable<ObservableValue<T>> observables)
   {
      SimpleObjectProperty<T> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) ->
      {
         synchronized (target)
         {
            target.setValue(newValue);
         }
      };

      observables.forEach(observable -> {
         if (observable.getValue() != null)
            target.setValue(observable.getValue());
         observable.addListener(changeListener);
      });
      return functional(target);
   }

   /**
    * Maps this observable value to another of type R. All changes from this
    * value are propagated to the mapped value.
    * @param mapper mapping function from T to R
    * @param <R> target type
    * @return mapped value
    */
   public final <R> FunctionalObservableValue<R> map(Function<? super T, ? extends R> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) -> target.setValue(mapper.apply(newValue));
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Returns an observable value consisting of the results of replacing each element of
    * this observable value stream with the elements of a mapped observable value produced by applying
    * the provided mapping function to each element.
    * @param mapper mapping function
    * @param <R> result type
    * @return mapped value
    */
   public final <R> FunctionalObservableValue<R> flatMap(Function<? super T, ? extends ObservableValue<? extends R>> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) ->
      {
         ObservableValue<? extends R> mapped = mapper.apply(newValue);
         ChangeListener<R> innerListener = (observable1, oldValue1, newValue1) -> target.setValue(newValue1);
         mapped.addListener(innerListener);
         innerListener.changed(mapped, null, mapped.getValue());
      };
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Returns an observable value consisting of the results of replacing each element of
    * this observable value stream with the contents of a mapped iterable produced by applying
    * the provided mapping function to each element.
    * @param mapper mapping function
    * @param <R> result type
    * @return mapped value
    */
   public final <R> FunctionalObservableValue<R> flatMapIterable(Function<? super T, ? extends Iterable<? extends R>> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) ->
      {
         Iterable<? extends R> mapped = mapper.apply(newValue);
         for (R r : mapped)
            target.setValue(r);
      };
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Returns an observable value consisting of the results of replacing each element of
    * this observable value stream with the contents of a mapped optional (if it has a value present)
    * produced by applying the provided mapping function to each element.
    * @param mapper mapping function
    * @param <R> result type
    * @return mapped value
    */
   public final <R> FunctionalObservableValue<R> flatMapOptional(Function<? super T, ? extends Optional<? extends R>> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) -> mapper.apply(newValue).ifPresent(target::setValue);
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Returns a new observable value whose value will only be updated from this one
    * if it matches the given predicate.
    * @param predicate predicate for the values
    * @return filtered value
    */
   public final FunctionalObservableValue<T> filter(Predicate<? super T> predicate)
   {
      SimpleObjectProperty<T> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) -> {
         if (predicate.test(newValue))
            target.setValue(newValue);
      };
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Returns an observable whose value is the accumulation
    * of all values from this observable.
    * @param combiner a function that combines two values of T into a new value
    * @return combined value
    */
   public final FunctionalObservableValue<T> reduce(BinaryOperator<T> combiner)
   {
      SimpleObjectProperty<T> target = new SimpleObjectProperty<>();
      addListener(new InvalidationListener()
      {
         private T accumulator = getValue();
         @Override public void invalidated(Observable observable)
         {
            accumulator = accumulator == null ? base.getValue() : combiner.apply(accumulator, base.getValue());
            target.setValue(accumulator);
         }
      });
      if (getValue() != null)
         target.setValue(getValue());
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Emits a pair of values whenever *join* of the two observables (this or other)
    * emits a new value. See http://reactivex.io/documentation/operators/combinelatest.html
    * for details on the behavior of this operator.
    * @param other other observable
    * @param <U> type of the other observable
    * @return combined observable
    */
   public final <U> FunctionalObservableValue<Tuple2<T, U>> combineLatest(ObservableValue<? extends U> other)
   {
      SimpleObjectProperty<Tuple2<T, U>> target = new SimpleObjectProperty<>();
      AtomicReference<T> lastThis = new AtomicReference<>(getValue());
      AtomicReference<U> lastOther = new AtomicReference<>(other.getValue());
      final Object lock = new Object();

      Runnable propagateChange = () -> {
         synchronized (lock)
         {
            T lastThisVal = lastThis.get();
            U lastOtherVal = lastOther.get();
            if (lastThisVal != null && lastOtherVal != null)
               target.setValue(Tuple.of(lastThisVal, lastOtherVal));
         }
      };

      ChangeListener<T> thisChangeListener = (observable, oldValue, newValue) -> {
         synchronized (lock)
         {
            lastThis.set(newValue);
            propagateChange.run();
         }
      };

      ChangeListener<U> otherChangeListener = (observable, oldValue, newValue) -> {
         synchronized (lock)
         {
            lastOther.set(newValue);
            propagateChange.run();
         }
      };

      addListener(thisChangeListener);
      other.addListener(otherChangeListener);

      if (lastThis.get() != null && lastOther.get() != null)
         propagateChange.run();

      return new FunctionalObservableValue<>(target);
   }

   /**
    * Avoids propagating the value if it has already
    * been called up in the call stack. This breaks
    * dependency cycles between observables.
    * @return observable avoiding dependency cycles
    */
   public final FunctionalObservableValue<T> avoidCycles()
   {
      NoCycleProperty<T> target = new NoCycleProperty<>(new SimpleObjectProperty<>());
      this.addListener(observable -> target.setValue(getValue()));
      return new FunctionalObservableValue<>(target);
   }

   /**
    * Narrows this functional observable to a subtype of T.
    * All values that are not of the target type are filtered out first.
    * @param toClass target class type
    * @param <Narrowed> narrowed type
    * @return narrowed observable value
    */
   public final <Narrowed extends T> FunctionalObservableValue<Narrowed> narrow(Class<Narrowed> toClass)
   {
      //noinspection unchecked
      return filter(value -> value != null && toClass.isAssignableFrom(value.getClass()))
            .map(value -> (Narrowed)value);
   }

   /**
    * Add a listener to this observable that consumes the observable stream.
    * All values from this observable are passed to the consumer, including
    * the existing value if it is not null.
    * @param consumer value consumer
    */
   public final void consume(Consumer<T> consumer)
   {
      addListener((observable, oldValue, newValue) -> consumer.accept(newValue));
      if (getValue() != null)
         consumer.accept(getValue());
   }

   @Override public final void addListener(ChangeListener<? super T> listener)
   {
      base.addListener(listener);
   }

   @Override public final void removeListener(ChangeListener<? super T> listener)
   {
      base.removeListener(listener);
   }

   @Override public final T getValue()
   {
      return base.getValue();
   }

   @Override public final void addListener(InvalidationListener listener)
   {
      base.addListener(listener);
   }

   @Override public final void removeListener(InvalidationListener listener)
   {
      base.removeListener(listener);
   }
}
