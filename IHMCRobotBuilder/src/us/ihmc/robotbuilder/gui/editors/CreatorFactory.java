package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javaslang.control.Try;
import us.ihmc.robotbuilder.gui.Creator;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.Editor.Factory;

import java.lang.reflect.Method;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

/**
 *
 */
public class CreatorFactory implements Creator.Factory
{
   private final Editor.Factory editorFactory;

   public CreatorFactory()
   {
      this(new PropertyEditorFactory());
   }

   public CreatorFactory(Factory editorFactory)
   {
      this.editorFactory = editorFactory;
   }

   @Override public <T> Optional<Creator<T>> create(Class<?> clazz, List<Class<?>> genericParameters)
   {
      Property<T> property = new SimpleObjectProperty<>();
      @SuppressWarnings("unchecked") Optional<Editor<T>> editorOpt = (Optional<Editor<T>>)(Optional)editorFactory.create(clazz, genericParameters, property);
      Optional<Creator<T>> creatorOpt = editorOpt.map(editor -> Creator.wrapEditor(editor, new DialogCreatorUI(editor.getEditor())));
      if (creatorOpt.isPresent())
         return creatorOpt;

      return builderForImmutablesBean(clazz)
            .map(beanBuilder -> (Creator<T>)() -> NewBeanEditor.createDialog(beanBuilder, beanFinalizerFor(beanBuilder), editorFactory));
   }

   private static <T> Optional<T> builderForImmutablesBean(Class<?> clazz)
   {
      String[] builderClassNames = {
            clazz.getSimpleName().replace("Immutable", "Modifiable"),
            "Modifiable" + clazz.getSimpleName()
      };

      for (String builderClassName : builderClassNames)
      {
         try
         {
            Class<?> builderClass = Class.forName(clazz.getPackage().getName() + "." + builderClassName);
            Method builderMethod = builderClass.getMethod("create");
            //noinspection unchecked
            return Optional.ofNullable((T)builderMethod.invoke(null));
         }
         catch (Exception e)
         {
            // continue to the next one
         }
      }
      return Optional.empty();
   }

   private static <T> Function<Object, Try<T>> beanFinalizerFor(Object builder)
   {
      try
      {
         Method builderMethod = builder.getClass().getMethod("toImmutable");
         //noinspection unchecked
         return o -> Try.of(() -> (T)builderMethod.invoke(o));
      }
      catch (NoSuchMethodException e)
      {
         throw new RuntimeException(e); // programmer error
      }
   }
}
