package us.ihmc.robotbuilder.util;

import javaslang.Function2;
import javaslang.Tuple2;
import javaslang.collection.List;

import java.util.Optional;
import java.util.function.Predicate;

/**
 *
 */
public class TreeIterator<T extends Tree<T>>
{
   final T focus;
   final List<Breadcrumb> breadcrumbs;
   final Function2<T, Iterable<T>, T> treeConstructor;

   public TreeIterator(T focus, List<Breadcrumb> breadcrumbs, Function2<T, Iterable<T>, T> treeConstructor)
   {
      this.focus = focus;
      this.breadcrumbs = breadcrumbs;
      this.treeConstructor = treeConstructor;
   }

   public Optional<TreeIterator<T>> parent() {
      return breadcrumbs.headOption().map(breadcrumb -> {
         T newParent = treeConstructor.apply(breadcrumb.node, breadcrumb.left.append(focus).appendAll(breadcrumb.right));
         return new TreeIterator<T>(newParent, breadcrumbs.tail(), treeConstructor);
      }).toJavaOptional();
   }

   public Optional<TreeIterator<T>> findChild(Predicate<? super T> predicate) {
      Tuple2<List<T>, List<T>> findResult = List.ofAll(focus.getChildren()).splitAt(predicate);
      return findResult._2.headOption().map(newFocus -> {
         Breadcrumb breadcrumb = new Breadcrumb(newFocus, findResult._1, findResult._2.tail());
         return new TreeIterator<T>(newFocus, breadcrumbs.tail().prepend(breadcrumb), treeConstructor);
      }).toJavaOptional();
   }

   public class Breadcrumb
   {
      final T node;
      final List<T> left, right;

      public Breadcrumb(T node, List<T> left, List<T> right)
      {
         this.node = node;
         this.left = left;
         this.right = right;
      }
   }
}
