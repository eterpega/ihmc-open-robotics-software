package us.ihmc.robotbuilder.gui;

import javaslang.concurrent.Future;
import javaslang.concurrent.Promise;

import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Allows creating new values. Complements {@link Editor} which can only
 * edit existing values.
 */
public interface Creator<T>
{
   /**
    * Creates a new bean of the given type or returns {@link Optional#empty()} if the user cancels
    * the creation.
    * @return a future of the created bean
    */
   Future<Optional<T>> create();

   static <T> Creator<T> wrapEditor(Editor<T> editor, CreatorUI ui)
   {
      Promise<Optional<T>> promise = Promise.make();
      Creator<T> result = promise::future;

      ui.addCancelActionListener(() -> {
         if (!promise.isCompleted())
            promise.success(Optional.empty());
      });

      ui.addConfirmAndValidationActionListener(() -> {
         if (!promise.isCompleted())
         {
            T value = editor.valueProperty().getValue();
            if (value == null)
            {
               NullPointerException error = new NullPointerException("Cannot create an empty value.");
               promise.failure(error);
               return Optional.of(error);
            }
            else
               promise.success(Optional.of(value));
         }
         return Optional.empty();
      });

      return result;
   }

   interface CreatorUI
   {
      void addCancelActionListener(Runnable cancelled);
      void addConfirmAndValidationActionListener(Supplier<Optional<Throwable>> confirmed);
   }

   interface Factory
   {
      <T> Optional<Creator<T>> create(Class<?> clazz, List<Class<?>> genericParameters);
   }
}
