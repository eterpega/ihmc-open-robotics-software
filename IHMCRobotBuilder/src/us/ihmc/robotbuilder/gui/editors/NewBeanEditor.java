package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Insets;
import javafx.geometry.VPos;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javaslang.Tuple;
import javaslang.collection.List;
import javaslang.concurrent.Future;
import javaslang.concurrent.Promise;
import javaslang.control.Try;
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.Editor.Factory;
import us.ihmc.robotbuilder.gui.ModifiableProperty;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;
import static us.ihmc.robotbuilder.util.ReflectionHelpers.getGenericParameters;

/**
 *
 */
public class NewBeanEditor<Builder, FinalBean>
{
   private static final String GET_PREFIX = "get";
   private final GridPane editor = new GridPane();
   private final Factory editorFactory;
   private final Builder beanBuilder;
   private final Promise<Optional<FinalBean>> promise = Promise.make();
   private final Function<Builder, Try<FinalBean>> beanFinalizer;

   private NewBeanEditor(Builder beanBuilder, Function<Builder, Try<FinalBean>> beanFinalizer, Factory editorFactory)
   {
      this.beanFinalizer = beanFinalizer;
      this.beanBuilder = beanBuilder;
      this.editorFactory = editorFactory;

      createUIFromBuilder(beanBuilder);

      editor.setVgap(5);
      editor.setHgap(10);
   }

   public static <Builder, FinalBean> Future<Optional<FinalBean>> createDialog(Builder beanBuilder, Function<Builder, Try<FinalBean>> beanFinalizer, Editor.Factory editorFactory)
   {
      NewBeanEditor<Builder, FinalBean> editor = new NewBeanEditor<>(beanBuilder, beanFinalizer, editorFactory);
      DialogCreatorUI ui = new DialogCreatorUI(editor.editor);
      ui.addCancelActionListener(editor::onCancel);
      ui.addConfirmAndValidationActionListener(editor::onApply);

      return editor.getPromise().future();
   }

   private void createUIFromBuilder(@NotNull Builder builder)
   {
      editor.getChildren().clear();

      List<NewBeanProperty> properties = getProperties(builder.getClass());
      //noinspection OptionalGetWithoutIsPresent
      properties
            .map(property -> Tuple.of(property, editorFactory.create(property.getValueType(), property.genericTypes, property.value())))
            .filter(propertyAndEditor -> propertyAndEditor._2.isPresent())
            .map(propertyAndEditor -> Tuple.of(propertyAndEditor._1, propertyAndEditor._2.get()))
            .zipWithIndex()
            .forEach(indexed ->
                     {
                        int index = (int) (long) indexed._2;
                        NewBeanProperty property = indexed._1._1;
                        Editor<?> propertyEditor = indexed._1._2;

                        Label label = new Label(property.getName());
                        editor.add(label, 0, index);
                        label.setPadding(new Insets(5, 0, 0, 0));
                        GridPane.setValignment(label, VPos.TOP);
                        editor.add(propertyEditor.getEditor(), 1, index);
                     });
   }

   private Optional<Throwable> onApply() {
      Try<FinalBean> finalBean = beanFinalizer.apply(beanBuilder);
      finalBean.onSuccess(value -> promise.success(Optional.of(value)));
      if (finalBean.isFailure())
      {
         return Optional.of(getCause(finalBean.getCause()));
      }
      return Optional.empty();
   }

   private void onCancel() {
      if (!promise.isCompleted())
         promise.success(Optional.empty());
   }

   private Throwable getCause(Throwable exception)
   {
      if (exception instanceof InvocationTargetException)
         return ((InvocationTargetException) exception).getTargetException();
      if (exception.getCause() == null)
         return exception;
      return getCause(exception.getCause());
   }

   private Promise<Optional<FinalBean>> getPromise()
   {
      return promise;
   }

   private List<NewBeanProperty> getProperties(final Class<?> beanClass)
   {
      final String getPrefix = "get";
      return List.ofAll(Arrays.asList(beanClass.getMethods()))
                   .filter(NewBeanEditor::isGetter)
                   .filter(method -> !method.isSynthetic() && !method.isBridge())
                   .distinctBy(Method::getName)
                   .map(getMethod -> {
                      String propertyName = getMethod.getName().substring(getPrefix.length());
                      Method setMethod = findSetter(beanClass, propertyName, getMethod.getReturnType());
                      if (setMethod != null)
                         //noinspection unchecked
                         return new NewBeanProperty(propertyName, (Class)getMethod.getReturnType(), setMethod, getGenericParameters(getMethod));
                      return null;
                   })
                   .filter(Objects::nonNull);
   }

   private static boolean isGetter(Method method)
   {
      return isPublic(method.getModifiers()) && !isStatic(method.getModifiers()) &&
            method.getParameterCount() == 0 && method.getName().startsWith(GET_PREFIX);
   }

   private static Method findSetter(Class<?> clazz, String propertyName, Class<?> propertyType)
   {
      return Arrays.stream(clazz.getMethods())
                   .filter(method -> method.getName().startsWith("set"))
                   .filter(method -> method.getName().contains(propertyName))
                   .filter(method -> clazz.isAssignableFrom(method.getReturnType()))
                   .filter(method -> method.getParameterCount() == 1 && method.getParameterTypes()[0].isAssignableFrom(propertyType))
                   .findFirst().orElse(null);
   }



   private class NewBeanProperty implements ModifiableProperty<Object>
   {
      private final String name;
      private final Class<Object> valueType;
      private final java.util.List<Class<?>> genericTypes;
      private final Property<Object> valueProperty = new SimpleObjectProperty<Object>() {
         @Override public String getName()
         {
            return name;
         }

         @Override public Object getBean()
         {
            return beanBuilder;
         }
      };

      NewBeanProperty(String name, Class<Object> valueType, Method setter, java.util.List<Class<?>> genericTypes)
      {
         this.name = name;
         this.valueType = valueType;
         this.genericTypes = genericTypes;

         ChangeListener<Object> propertyChangeListener = (observable, oldValue, newValue) -> {
            try
            {
               //noinspection unchecked
               setter.invoke(beanBuilder, newValue);
            }
            catch (Exception e)
            {
               throw new RuntimeException(e);
            }

         };

         valueProperty.addListener(propertyChangeListener);
      }

      @Override public String getName()
      {
         return name;
      }

      @Override public Class<Object> getValueType()
      {
         return valueType;
      }

      @Override public Property<Object> value()
      {
         return valueProperty;
      }
   }
}
