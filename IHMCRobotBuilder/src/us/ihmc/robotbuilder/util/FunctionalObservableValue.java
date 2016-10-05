package us.ihmc.robotbuilder.util;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.ReadOnlyProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import org.jetbrains.annotations.NotNull;

import java.util.Optional;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Enhances the {@link ReadOnlyProperty} interface with functional methods such as map/filter etc.
 */
public final class FunctionalObservableValue<T> implements ObservableValue<T>
{
   private final ObservableValue<T> base;

   public FunctionalObservableValue(@NotNull ObservableValue<T> base)
   {
      this.base = base;
   }

   public static <T> FunctionalObservableValue<T> of(@NotNull ObservableValue<T> base)
   {
      return new FunctionalObservableValue<>(base);
   }

   public final <R> FunctionalObservableValue<R> map(Function<? super T, ? extends R> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) -> target.setValue(mapper.apply(newValue));
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

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

   public final <R> FunctionalObservableValue<R> flatMapOptional(Function<? super T, ? extends Optional<? extends R>> mapper)
   {
      SimpleObjectProperty<R> target = new SimpleObjectProperty<>();
      ChangeListener<T> changeListener = (observable, oldValue, newValue) -> mapper.apply(newValue).ifPresent(target::setValue);
      addListener(changeListener);
      if (getValue() != null)
         changeListener.changed(this, null, getValue());
      return new FunctionalObservableValue<>(target);
   }

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
