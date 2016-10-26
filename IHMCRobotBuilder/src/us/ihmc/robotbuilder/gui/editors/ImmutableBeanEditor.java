package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.util.Callback;
import org.controlsfx.control.PropertySheet;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.PropertyEditor;
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;

import java.lang.reflect.Method;
import java.util.*;
import java.util.stream.Collectors;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;
import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;

/**
 *
 */
public class ImmutableBeanEditor<T> extends AbstractPropertyEditor<T, PropertySheet>
{
   private final PropertySheet currentSheet;
   private Property<T> valueProperty;
   private final boolean includeReadonlyProperties;

   public ImmutableBeanEditor(@NotNull Item itemToEdit, Callback<Item, PropertyEditor<?>> editorFactory, boolean includeReadonlyProperties)
   {
      super(itemToEdit, new PropertySheet());
      this.includeReadonlyProperties = includeReadonlyProperties;
      this.currentSheet = new PropertySheet();
      currentSheet.setPropertyEditorFactory(editorFactory);

      ChangeListener<T> itemChangeListener = (observable, oldValue, newValue) -> valueProperty.setValue(newValue);
      FunctionalObservableValue<T> funcValueProperty = functional(valueProperty);
      getProperties(valueProperty.getValue()).forEach(item ->
                                   {
                                      // Update bean when value changes
                                      item.beanProperty().addListener(itemChangeListener);
                                      funcValueProperty.filter(newBean -> item.getBean() != newBean)
                                                       .consume(item.beanProperty()::setValue);
                                      currentSheet.getItems().add(item);
                                   });
   }

   @Override protected ObservableValue<T> getObservableValue()
   {
      if (valueProperty == null)
         //noinspection unchecked
         valueProperty = new SimpleObjectProperty<>((T)getProperty().getValue());
      return valueProperty;
   }

   @Override public PropertySheet getEditor()
   {
      return currentSheet;
   }

   public Property<T> valueProperty()
   {
      return valueProperty;
   }

   @Override public T getValue()
   {
      return valueProperty.getValue();
   }

   @Override public void setValue(T value)
   {
      valueProperty.setValue(value);
   }

   private List<ImmutableBeanProperty<T>> getProperties(final T bean)
   {
      final String getPrefix = "get";
      final Set<String> usedProperties = new HashSet<>();
      return Arrays.stream(bean.getClass().getMethods())
                   .filter(method -> isPublic(method.getModifiers()))
                   .filter(method -> !isStatic(method.getModifiers()))
                   .filter(method -> method.getName().startsWith(getPrefix))
                   .filter(method -> method.getParameterCount() == 0)
                   .filter(method -> !usedProperties.contains(method.getName()))
                   .map(getMethod -> {
                      usedProperties.add(getMethod.getName());
                      String propertyName = getMethod.getName().substring(getPrefix.length());
                      Method setMethod = findSetter(bean.getClass(), propertyName, getMethod.getReturnType());
                      return new ImmutableBeanProperty<>(bean, propertyName, "", propertyName,
                                                         getMethod, Optional.ofNullable(setMethod),
                                                         getMethod.getReturnType());
                   })
                   .filter(item -> includeReadonlyProperties || item.isEditable())
                   .collect(Collectors.toList());
   }

   private static Method findSetter(Class<?> clazz, String propertyName, Class<?> propertyType)
   {
      return Arrays.stream(clazz.getMethods())
                   .filter(method -> method.getName().startsWith("with") || method.getName().startsWith("set"))
                   .filter(method -> method.getName().contains(propertyName))
                   .filter(method -> clazz.isAssignableFrom(method.getReturnType()))
                   .filter(method -> method.getParameterCount() == 1 && method.getParameterTypes()[0].isAssignableFrom(propertyType))
                   .findFirst().orElse(null);
   }
}
