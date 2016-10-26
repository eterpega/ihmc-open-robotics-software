package us.ihmc.robotbuilder.util;

import javaslang.collection.List;
import javaslang.collection.Stream;
import us.ihmc.robotbuilder.util.TreeFocus.Breadcrumb;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * An immutable tree implementation. The tree acts as a container
 * for arbitrary immutable values.
 */
public final class Tree<T> implements TreeInterface<Tree<T>>
{
   private final T value;
   private final Stream<Tree<T>> children;

   /**
    * Creates a new tree node with the given collection of children.
    * @param value immutable tree value
    * @param children node children
    */
   public Tree(T value, Iterable<Tree<T>> children)
   {
      this.value = value;
      this.children = Stream.ofAll(children);
   }

   /**
    * Creates a leaf tree node.
    * @param value immutable node value
    */
   public Tree(T value)
   {
      this.value = value;
      this.children = Stream.empty();
   }

   /**
    * Wraps an existing tree structure to this tree.
    * @param existingTree existing tree
    * @param childGetter method that returns children given an instance of existing tree
    * @param <Source> type of the existing tree
    * @return existing tree wrapped in {@link Tree}
    */
   public static <Source> Tree<Source> adapt(Source existingTree, Function<Source, ? extends Iterable<? extends Source>> childGetter)
   {
      return new Tree<>(existingTree, List.ofAll(childGetter.apply(existingTree)).map(child -> adapt(child, childGetter)));
   }

   /**
    * Get stored user value.
    * @return node value
    */
   public T getValue()
   {
      return value;
   }

   /**
    * Get the children of this tree node.
    * @return children
    */
   @Override
   public Iterable<Tree<T>> getChildren()
   {
      return children;
   }

   /**
    * Get the child stream directly. Package private, used for optimization
    * in tree algorithms.
    * @return children
    */
   Stream<Tree<T>> children()
   {
      return children;
   }

   /**
    * Count the direct children of this tree node.
    * @return children count
    */
   public int countChildren()
   {
      return children.size();
   }

   /**
    * Get the first child of this node if it exists.
    * @return first child or {@link Optional#empty()} if this tree has no children
    */
   public Optional<Tree<T>> firstChild()
   {
      java.util.Iterator<Tree<T>> iter = getChildren().iterator();
      if (iter.hasNext())
         return Optional.of(iter.next());
      return Optional.empty();
   }

   /**
    * Get the ith child. Can throw exception if out of bounds, used
    * only in tests - package private.
    * @param index child index
    * @return ith child
    */
   Tree<T> getChild(int index)
   {
      int i = 0;
      for (Tree<T> child : getChildren())
      {
         if (index == i)
            return child;
         i++;
      }
      throw new IndexOutOfBoundsException("Child index out of bounds: " + index + " should be less than " + i);
   }

   /**
    * Map this tree onto another tree type. There are no requirements on the target
    * type other than having a tree-like structure.
    * @param mapper mapping function
    * @param <R> result tree type
    * @return mapped tree
    */
   public final <R> R map(TreeNodeMapper<Tree<T>, R> mapper)
   {
      return TreeInterface.map(this, mapper);
   }

   /**
    * Map the values of this tree, keeping the tree structure.
    * @param mapper mapping function
    * @param <R> target value type
    * @return tree containing mapped values
    */
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

   /**
    * Removes nodes (and their children) that do not match the predicate.
    * Returns a new, pruned tree.
    * @param predicate filter predicate
    * @return filtered tree
    */
   public final Optional<Tree<T>> filter(Predicate<? super Tree<T>> predicate)
   {
      return TreeInterface.filter(this, predicate, Tree::nodeSupplier);
   }

   /**
    * Filters nodes (and their children) whose value does not match the predicate.
    * Returns a new, pruned tree.
    * @param predicate filter predicate
    * @return filtered tree
    */
   public final Optional<Tree<T>> filterValues(Predicate<? super T> predicate)
   {
      return TreeInterface.filter(this, tree -> predicate.test(tree.getValue()), Tree::nodeSupplier);
   }

   /**
    * Lazy {@link java.util.stream.Stream} for accessing all the nodes in the tree (recursive).
    * @return tree node stream
    */
   public final java.util.stream.Stream<T> stream()
   {
      return TreeInterface.flatten(this).map(Tree::getValue);
   }

   /**
    * Finds the first node matching the given predicate and returns its focus.
    * The returned focus allows modifying the tree around the found node.
    * @param predicate predicate to find the child
    * @return {@link TreeFocus} on the found node or {@link Optional#empty()} if no node matches the predicate
    */
   public final Optional<TreeFocus<Tree<T>>> find(Predicate<? super Tree<T>> predicate)
   {
      return find(predicate, List.empty());
   }

   /**
    * Finds the first node whose value matches the given predicate and returns its focus.
    * The returned focus allows modifying the tree around the found node.
    * @param predicate predicate to find the child
    * @return {@link TreeFocus} on the found node or {@link Optional#empty()} if no node matches the predicate
    */
   public final Optional<TreeFocus<Tree<T>>> findValue(Predicate<? super T> predicate)
   {
      return find(tree -> predicate.test(tree.value), List.empty());
   }

   /**
    * Finds the first node whose value matches the given value and returns its focus.
    * The returned focus allows modifying the tree around the found node.
    * @param value value to find in the children
    * @return {@link TreeFocus} on the found node or {@link Optional#empty()} if no node matches the value
    */
   public final Optional<TreeFocus<Tree<T>>> findValue(T value)
   {
      return findValue(value::equals);
   }

   /**
    * Returns a root {@link TreeFocus} of this node. This effectively
    * sets this node as a root for join operations performed on the returned focus.
    * @return root focus
    */
   public final TreeFocus<Tree<T>> getFocus()
   {
      return new TreeFocus<>(this, List.empty(), Tree::nodeSupplier);
   }

   /**
    * Returns a new node with the current value replaced with the given
    * new value. Keeps child nodes intact.
    * @param newValue new value for the node
    * @return node with the new value
    */
   public final Tree<T> withValue(T newValue)
   {
      return new Tree<>(newValue, children);
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

   /**
    * Does deep structural comparison of this tree and another tree.
    * This is a linear time operation
    * @param o tree to compare to
    * @return true if the trees have the same structure
    */
   public boolean deepEquals(Tree<T> o)
   {
      return this == o ||
            value.equals(o.value) &&
                  children.size() == o.children.size() &&
                  children.zip(o.children).forAll(zipped -> zipped._1.deepEquals(zipped._2));
   }

   /**
    * Computes structural hash code instead of hash code based on object references only.
    * @return deep hash code
    */
   public int deepHashCode()
   {
      int result = value.hashCode();
      result = 31 * result + children.map(Tree::deepHashCode).fold(0, (hash1, hash2) -> 31 * hash1 + hash2);
      return result;
   }

   /**
    * Draws a nice ASCII art representation of the tree.
    * Code adapted from Javaslang's Tree.
    * @return tree drawing
    */
   public String draw() {
      StringBuilder builder = new StringBuilder();
      drawAux("", builder);
      return builder.toString();
   }

   private void drawAux(String indent, StringBuilder builder) {
      builder.append(value);
      for (Stream<Tree<T>> it = children; !it.isEmpty(); it = it.tail()) {
         final boolean isLast = it.tail().isEmpty();
         builder.append('\n')
                .append(indent)
                .append(isLast ? "└──" : "├──");
         it.head().drawAux(indent + (isLast ? "   " : "│  "), builder);
      }
   }

   @Override public String toString()
   {
      return "Tree{" + "value=" + value + ", children=" + children + '}';
   }

   private static <T> Tree<T> nodeSupplier(Tree<T> source, Iterable<Tree<T>> children) {
      return new Tree<>(source.getValue(), List.ofAll(children));
   }
}
