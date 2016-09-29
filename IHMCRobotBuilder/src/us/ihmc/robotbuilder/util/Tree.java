package us.ihmc.robotbuilder.util;

import javaslang.collection.List;
import javaslang.collection.Stream;
import us.ihmc.robotbuilder.util.TreeFocus.Breadcrumb;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 *
 */
public final class Tree<T> implements TreeInterface<Tree<T>>
{
   private final T value;
   private final Stream<Tree<T>> children;

   public Tree(T value, Iterable<Tree<T>> children)
   {
      this.value = value;
      this.children = Stream.ofAll(children);
   }

   public static <Source> Tree<Source> adapt(Source value, Function<Source, ? extends Iterable<? extends Source>> childGetter)
   {
      return new Tree<>(value, List.ofAll(childGetter.apply(value)).map(child -> adapt(child, childGetter)));
   }

   public T getValue()
   {
      return value;
   }

   @Override
   public Iterable<Tree<T>> getChildren()
   {
      return children;
   }

   public final <R> R map(TreeNodeMapper<Tree<T>, R> mapper)
   {
      return TreeInterface.map(this, mapper);
   }

   public final <R> Tree<R> mapValues(Function<? super T, ? extends R> mapper)
   {
      return TreeInterface.map(this, new TreeNodeMapper<Tree<T>, Tree<R>>()
      {
         @Override public Tree<R> mapNode(Tree<T> node, java.util.List<Tree<R>> children)
         {
            return new Tree<>(mapper.apply(node.getValue()), List.ofAll(children));
         }
      });
   }

   public final Optional<Tree<T>> filter(Predicate<? super Tree<T>> predicate)
   {
      return TreeInterface.filter(this, predicate, Tree::nodeSupplier);
   }

   public final Optional<Tree<T>> filterValues(Predicate<? super T> predicate)
   {
      return TreeInterface.filter(this, tree -> predicate.test(tree.getValue()), Tree::nodeSupplier);
   }

   public final java.util.stream.Stream<T> stream()
   {
      return TreeInterface.flatten(this).map(Tree::getValue);
   }

   public final Optional<TreeFocus<Tree<T>>> find(Predicate<? super Tree<T>> predicate)
   {
      return find(predicate, List.empty());
   }

   public final Optional<TreeFocus<Tree<T>>> findValue(Predicate<? super T> predicate)
   {
      return find(tree -> predicate.test(tree.value), List.empty());
   }

   public final TreeFocus<Tree<T>> getFocus()
   {
      return new TreeFocus<>(this, List.empty(), Tree::nodeSupplier);
   }

   private Optional<TreeFocus<Tree<T>>> find(Predicate<? super Tree<T>> predicate, List<Breadcrumb<Tree<T>>> breadcrumbs)
   {
      if (predicate.test(this))
         return Optional.of(new TreeFocus<>(this, breadcrumbs, Tree::nodeSupplier));

      List<Tree<T>> leftReversed = List.empty();
      List<Tree<T>> right = children.toList();
      while (!right.isEmpty())
      {
         Tree<T> focus = right.head();
         right = right.tail();
         Breadcrumb<Tree<T>> newBreadcrumb = new Breadcrumb<>(this, leftReversed, right);
         leftReversed = leftReversed.prepend(focus);

         Optional<TreeFocus<Tree<T>>> findResult = focus.find(predicate, breadcrumbs.prepend(newBreadcrumb));
         if (findResult.isPresent())
            return findResult;
      }
      return Optional.empty();
   }

   @Override public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      Tree<?> tree = (Tree<?>) o;

      return value.equals(tree.value) && children.equals(tree.children);

   }

   @Override public int hashCode()
   {
      int result = value.hashCode();
      result = 31 * result + children.hashCode();
      return result;
   }

   @Override public String toString()
   {
      return "Tree{" + "value=" + value + ", children=" + children + '}';
   }

   private static <T> Tree<T> nodeSupplier(Tree<T> source, Iterable<Tree<T>> children) {
      return new Tree<>(source.getValue(), List.ofAll(children));
   }
}
