package us.ihmc.robotbuilder.util;

import javaslang.Function2;
import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.List;

import java.util.Optional;
import java.util.function.Predicate;

/**
 *
 */
public class TreeFocus<T extends TreeInterface<T>>
{
   private final T focus;
   private final List<Breadcrumb<T>> breadcrumbs;
   private final Function2<T, Iterable<T>, T> treeConstructor;

   public TreeFocus(T focus, List<Breadcrumb<T>> breadcrumbs, Function2<T, Iterable<T>, T> treeConstructor)
   {
      this.focus = focus;
      this.breadcrumbs = breadcrumbs;
      this.treeConstructor = treeConstructor;
   }

   public T getFocusedNode()
   {
      return focus;
   }

   public Optional<TreeFocus<T>> parent()
   {
      return breadcrumbs.headOption().map(breadcrumb ->
                                          {
                                             T newParent = treeConstructor
                                                   .apply(breadcrumb.parent, breadcrumb.leftReversed.prepend(focus).reverse().appendAll(breadcrumb.right));
                                             return new TreeFocus<>(newParent, breadcrumbs.tail(), treeConstructor);
                                          }).toJavaOptional();
   }

   public TreeFocus<T> root()
   {
      return parent().map(TreeFocus::root).orElse(this);
   }

   public Optional<TreeFocus<T>> findChild(Predicate<? super T> predicate)
   {
      Tuple2<List<T>, List<T>> findResult = List.ofAll(focus.getChildren()).splitAt(predicate);
      return findResult._2.headOption().map(newFocus -> focusOnChild(newFocus, findResult._1, findResult._2.tail())).toJavaOptional();
   }

   public Optional<TreeFocus<T>> firstChild()
   {
      List<T> children = List.ofAll(focus.getChildren());
      return children.headOption().map(newFocus -> focusOnChild(newFocus, List.empty(), children.tail())).toJavaOptional();
   }

   public Optional<TreeFocus<T>> nextSibling()
   {
      return breadcrumbs.headOption()
                 .flatMap(firstBreadcrumb -> firstBreadcrumb.right.headOption().map(head -> Tuple.of(head, firstBreadcrumb)))
                 .map(tuple -> {
                    T newFocus = tuple._1;
                    List<T> leftReversed = tuple._2.leftReversed.prepend(this.focus);
                    List<T> right = tuple._2.right.tail();
                    Breadcrumb<T> newBreadcrump = new Breadcrumb<>(tuple._2.parent, leftReversed, right);
                    return new TreeFocus<>(newFocus, breadcrumbs.tail().prepend(newBreadcrump), treeConstructor);
                 }).toJavaOptional();
   }

   public Optional<TreeFocus<T>> previousSibling()
   {
      return breadcrumbs.headOption()
                        .flatMap(firstBreadcrumb -> firstBreadcrumb.leftReversed.headOption().map(head -> Tuple.of(head, firstBreadcrumb)))
                        .map(tuple -> {
                           T newFocus = tuple._1;
                           List<T> leftReversed = tuple._2.leftReversed.tail();
                           List<T> right = tuple._2.right.prepend(this.focus);
                           Breadcrumb<T> newBreadcrumb = new Breadcrumb<>(tuple._2.parent, leftReversed, right);
                           return new TreeFocus<>(newFocus, breadcrumbs.tail().prepend(newBreadcrumb), treeConstructor);
                        }).toJavaOptional();
   }

   private TreeFocus<T> focusOnChild(T newFocus, List<T> left, List<T> right)
   {
      Breadcrumb<T> breadcrumb = new Breadcrumb<>(this.focus, left.reverse(), right);
      return new TreeFocus<>(newFocus, breadcrumbs.prepend(breadcrumb), treeConstructor);
   }

   @Override public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      TreeFocus<?> treeFocus = (TreeFocus<?>) o;

      return focus.equals(treeFocus.focus) && breadcrumbs.equals(treeFocus.breadcrumbs);

   }

   @Override public int hashCode()
   {
      int result = focus.hashCode();
      result = 31 * result + breadcrumbs.hashCode();
      return result;
   }

   @Override public String toString()
   {
      return "TreeFocus{" + "focus=" + focus + ", breadcrumbs=" + breadcrumbs + '}';
   }

   public static class Breadcrumb<T>
   {
      final T parent;
      final List<T> leftReversed, right;

      public Breadcrumb(T parent, List<T> leftReversed, List<T> right)
      {
         this.parent = parent;
         this.leftReversed = leftReversed;
         this.right = right;
      }

      @Override public boolean equals(Object o)
      {
         if (this == o)
            return true;
         if (o == null || getClass() != o.getClass())
            return false;

         Breadcrumb<?> that = (Breadcrumb<?>) o;

         return parent.equals(that.parent) && leftReversed.equals(that.leftReversed) && right.equals(that.right);

      }

      @Override public int hashCode()
      {
         int result = parent.hashCode();
         result = 31 * result + leftReversed.hashCode();
         result = 31 * result + right.hashCode();
         return result;
      }

      @Override public String toString()
      {
         return "Breadcrumb{" + "parent=" + parent + ", leftReversed=" + leftReversed + ", right=" + right + '}';
      }
   }
}
