package us.ihmc.robotics.util;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

/**
 * A {@link Property} that does not allow cycles in its call chain.
 * That means that calling setValue() from a callback on this property
 * won't lead to an infinite loop. This class serves as a wrapper around
 * any existing property.
 */
public class NoCycleProperty<T> implements Property<T>
{
   private final Map<Object, ChangeListenerWrapper> mappedChangeListeners = new HashMap<>();
   private final Map<Object, InvalidationListenerWrapper> mappedInvalidationListeners = new HashMap<>();
   private final Property<T> originalProperty;
   private boolean editing = false;

   /**
    * Creates a new {@link NoCycleProperty} by wrapping the given property.
    * @param originalProperty property to wrap
    */
   public NoCycleProperty(Property<T> originalProperty)
   {
      this.originalProperty = originalProperty;
   }

   /**
    * Creates a new {@link NoCycleProperty} by wrapping the given property.
    * Can be used in static import to make code more readable.
    * @param originalProperty property to wrap
    */
   public static <T> NoCycleProperty<T> noCycle(Property<T> originalProperty)
   {
      if (originalProperty instanceof NoCycleProperty)
         return (NoCycleProperty<T>) originalProperty;
      return new NoCycleProperty<>(originalProperty);
   }

   @Override public void bind(ObservableValue<? extends T> observable)
   {
      originalProperty.bind(observable);
   }

   @Override public void unbind()
   {
      originalProperty.unbind();
   }

   @Override public boolean isBound()
   {
      return originalProperty.isBound();
   }

   @Override public void bindBidirectional(Property<T> other)
   {
      originalProperty.bindBidirectional(other);
   }

   @Override public void unbindBidirectional(Property<T> other)
   {
      originalProperty.unbindBidirectional(other);
   }

   @Override public Object getBean()
   {
      return originalProperty.getBean();
   }

   @Override public String getName()
   {
      return originalProperty.getName();
   }

   @Override public void addListener(ChangeListener<? super T> listener)
   {
      ChangeListenerWrapper wrapped = new ChangeListenerWrapper(listener);
      mappedChangeListeners.put(listener, wrapped);
      originalProperty.addListener(wrapped);
   }

   @Override public void removeListener(ChangeListener<? super T> listener)
   {
      ChangeListenerWrapper wrapped = mappedChangeListeners.remove(listener);
      if (wrapped != null)
         originalProperty.removeListener(wrapped);
   }

   @Override public T getValue()
   {
      return originalProperty.getValue();
   }

   @Override public void addListener(InvalidationListener listener)
   {
      InvalidationListenerWrapper wrapped = new InvalidationListenerWrapper(listener);
      mappedInvalidationListeners.put(listener, wrapped);
      originalProperty.addListener(wrapped);
   }

   @Override public void removeListener(InvalidationListener listener)
   {
      InvalidationListenerWrapper wrapped = mappedInvalidationListeners.remove(listener);
      if (wrapped != null)
         originalProperty.removeListener(wrapped);
   }

   @Override public void setValue(T value)
   {
      synchronized (this)
      {
         if (editing)
            return;

         originalProperty.setValue(value);
      }
   }

   private class ChangeListenerWrapper implements ChangeListener<T>
   {
      private final ChangeListener<? super T> originalChange;

      private ChangeListenerWrapper(@NotNull ChangeListener<? super T> original)
      {
         this.originalChange = original;
      }

      @Override public void changed(ObservableValue<? extends T> observable, T oldValue, T newValue)
      {
         synchronized (NoCycleProperty.this)
         {
            editing = true;
            originalChange.changed(NoCycleProperty.this, oldValue, newValue);
            editing = false;
         }
      }
   }

   private class InvalidationListenerWrapper implements InvalidationListener
   {
      private final InvalidationListener originalInvalidation;

      private InvalidationListenerWrapper(@NotNull InvalidationListener original)
      {
         this.originalInvalidation = original;
      }

      @Override public void invalidated(Observable observable)
      {
         synchronized (NoCycleProperty.this)
         {
            editing = true;
            originalInvalidation.invalidated(NoCycleProperty.this);
            editing = false;
         }
      }
   }
}
