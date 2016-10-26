package us.ihmc.robotbuilder.util;

import javaslang.Function2;
import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.List;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;
import java.util.function.Predicate;

/**
 * Represents a focus (aka zipper) on a {@link Tree} node that allows for a simple
 * immutable tree modification. See http://learnyouahaskell.com/zippers for details.
 * This class allows focusing on join tree implementing the {@link TreeInterface}.
 */
public class TreeFocus<T extends TreeInterface<T>> implements TreeInterface<TreeFocus<T>>
{
   private final T focus;
   private final List<Breadcrumb<T>> breadcrumbs;
   private final Function2<T, Iterable<T>, T> treeConstructor;

   /**
    * Creates a new focus based on the tree node to focus on, the path (breadcrumbs) leading to the node
    * and a tree constructor.
    * @param focus node to focus on
    * @param breadcrumbs path to the node (empty for root node)
    * @param treeConstructor tree constructor
    */
   public TreeFocus(T focus, List<Breadcrumb<T>> breadcrumbs, Function2<T, Iterable<T>, T> treeConstructor)
   {
      this.focus = focus;
      this.breadcrumbs = breadcrumbs;
      this.treeConstructor = treeConstructor;
   }

   /**
    * Creates a child focus by following the given path. Each element of the path
    * represents an index to the child list.
    * @param path path to follow
    * @return given tree focus or {@link Optional#empty()} if the path is not valid
    */
   public Optional<TreeFocus<T>> focusOnPath(List<Integer> path)
   {
      return path.headOption().toJavaOptional()
                 .map(index -> getChild(index).flatMap(child -> child.focusOnPath(path.tail())))
                 .orElse(Optional.of(this));
   }

   /**
    * Get the original tree node this focus focuses on.
    * @return focused node
    */
   public T getFocusedNode()
   {
      return focus;
   }

   /**
    * Move this focus to the parent.
    * @return parent focus or {@link Optional#empty()} if this is a root focus
    */
   public Optional<TreeFocus<T>> parent()
   {
      return breadcrumbs
            .headOption()
            .map(breadcrumb ->
                 {
                    // Optimization: just return the old parent node if this node has not changed
                    T oldParent = breadcrumb.parent;
                    Optional<T> oldThis = oldParent.childStream().skip(breadcrumb.leftReversed.length()).findFirst();
                    if (oldThis.isPresent() && oldThis.get() == focus)
                       return new TreeFocus<>(oldParent, breadcrumbs.tail(), treeConstructor);

                    T newParent = treeConstructor.apply(breadcrumb.parent, breadcrumb.leftReversed.prepend(focus).reverse().appendAll(breadcrumb.right));
                    return new TreeFocus<>(newParent, breadcrumbs.tail(), treeConstructor);
                 }).toJavaOptional();
   }

   /**
    * Move this focus to the root of the tree.
    * @return root focus
    */
   public TreeFocus<T> root()
   {
      return parent().map(TreeFocus::root).orElse(this);
   }

   /**
    * Finds the first direct child of this node that matches the predicate and returns its focus.
    * @param predicate predicate
    * @return found child or {@link Optional#empty()} if no child node matches the predicate
    */
   public Optional<TreeFocus<T>> findChild(Predicate<? super T> predicate)
   {
      Tuple2<List<T>, List<T>> findResult = List.ofAll(focus.getChildren()).splitAt(predicate);
      return findResult._2.headOption()
                          .map(newFocus -> focusOnChild(newFocus, findResult._1, findResult._2.tail()))
                          .toJavaOptional();
   }

   /**
    * Returns focus on the ith child if it is in range.
    * @param index child index
    * @return found child or {@link Optional#empty()} if the given index is out of bounds
    */
   public Optional<TreeFocus<T>> getChild(int index)
   {
      if (index < 0)
         return Optional.empty();
      Tuple2<List<T>, List<T>> findResult = List.ofAll(focus.getChildren()).splitAt(index);
      return findResult._2.headOption()
                          .map(newFocus -> focusOnChild(newFocus, findResult._1, findResult._2.tail()))
                          .toJavaOptional();
   }

   /**
    * Returns the focus on the first child of this tree.
    * @return first child or {@link Optional#empty()} if this is a leaf tree
    */
   public Optional<TreeFocus<T>> firstChild()
   {
      List<T> children = List.ofAll(focus.getChildren());
      return children.headOption()
                     .map(newFocus -> focusOnChild(newFocus, List.empty(), children.tail()))
                     .toJavaOptional();
   }

   /**
    * Returns a focus on the next sibling of this tree node.
    * @return next sibling or {@link Optional#empty()} if this is the last child of its parent
    */
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
                 })
                 .toJavaOptional();
   }

   /**
    * Returns a focus on the previous sibling of this tree node.
    * @return previous sibling or {@link Optional#empty()} if this is the first child of its parent
    */
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

   /**
    * Replace this tree node with another. Returns a focus on the new node.
    * @param replacement node to replace this one
    * @return focus on the replacement node
    */
   public TreeFocus<T> replace(T replacement)
   {
      return new TreeFocus<>(replacement, breadcrumbs, treeConstructor);
   }

   /**
    * Adds a new sibling on the right from this node. Returns a focus on the new node.
    * @param newSibling new sibling to add
    * @return added sibling or {@link Optional#empty()} if this is the root focus
    */
   public Optional<TreeFocus<T>> addRightSibling(T newSibling)
   {
      return breadcrumbs.headOption().map(oldBreadcrumb -> {
         Breadcrumb<T> newBreadcrumb = new Breadcrumb<>(oldBreadcrumb.parent, oldBreadcrumb.leftReversed.prepend(this.focus), oldBreadcrumb.right);
         return new TreeFocus<>(newSibling, breadcrumbs.tail().prepend(newBreadcrumb), treeConstructor);
      }).toJavaOptional();
   }

   /**
    * Adds a new sibling on the left from this node. Returns a focus on the new node.
    * @param newSibling new sibling to add
    * @return added sibling or {@link Optional#empty()} if this is the root focus
    */
   public Optional<TreeFocus<T>> addLeftSibling(T newSibling)
   {
      return breadcrumbs.headOption().map(oldBreadcrumb -> {
         Breadcrumb<T> newBreadcrumb = new Breadcrumb<>(oldBreadcrumb.parent, oldBreadcrumb.leftReversed, oldBreadcrumb.right.prepend(this.focus));
         return new TreeFocus<>(newSibling, breadcrumbs.tail().prepend(newBreadcrumb), treeConstructor);
      }).toJavaOptional();
   }

   /**
    * Appends a new child at the end of the child list and returns its focus.
    * Note that this is a linear operation. If the order of child nodes is not
    * important, use the constant-time {@link #prependChild(TreeInterface)} instead.
    * @param newChild new child to append
    * @return focus on the newly added child
    */
   public TreeFocus<T> appendChild(T newChild)
   {
      Breadcrumb<T> newBreadcrumb = new Breadcrumb<>(this.focus, List.ofAll(focus.getChildren()).reverse(), List.empty());
      return new TreeFocus<>(newChild, breadcrumbs.prepend(newBreadcrumb), treeConstructor);
   }

   /**
    * Prepends a new child at the beginning of the child list and returns its focus.
    * Constant time operation.
    * @param newChild new child to prepend
    * @return focus on the newly added child
    */
   public TreeFocus<T> prependChild(T newChild)
   {
      Breadcrumb<T> newBreadcrumb = new Breadcrumb<>(this.focus, List.empty(), List.ofAll(focus.getChildren()));
      return new TreeFocus<>(newChild, breadcrumbs.prepend(newBreadcrumb), treeConstructor);
   }

   /**
    * Removes the current node and returns the focus on its parent.
    * If this is the tree, empty value is returned.
    * @return parent node or {@link Optional#empty()} if this is a root node
    */
   public Optional<TreeFocus<T>> remove()
   {
      return breadcrumbs.headOption().map(breadcrumb -> {
         T newParent = treeConstructor
               .apply(breadcrumb.parent, breadcrumb.leftReversed.reverse().appendAll(breadcrumb.right));
         return new TreeFocus<>(newParent, breadcrumbs.tail(), treeConstructor);
      }).toJavaOptional();
   }

   /**
    * Returns a path from the root of the tree to the given child. A path consists
    * of child index at every level of the tree that the path should follow.
    * @param child child to find the path to
    * @param childIndexOf function returning the child index for the given parent
    * @param parentGetter function returning the parent of the given node
    * @param <T> tree type
    * @return path to the given child node if it exists
    */
   public static <T> List<Integer> pathTo(T child, ChildIndexOf<T> childIndexOf, ParentOf<T> parentGetter)
   {
      java.util.List<Integer> result = new ArrayList<>();
      pathTo(child, childIndexOf, parentGetter, result);
      return List.ofAll(result);
   }

   private static <T> void pathTo(T child, ChildIndexOf<T> childIndexOf, ParentOf<T> parentGetter, java.util.List<Integer> accumulator)
   {
      Optional<T> parentOpt = parentGetter.getParentOfOptional(child);
      parentOpt.ifPresent(parent -> {
         Optional<Integer> childIndex = childIndexOf.indexOfOptional(parent, child);
         childIndex.ifPresent(index -> {
            pathTo(parent, childIndexOf, parentGetter, accumulator);
            accumulator.add(index);
         });
      });
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

   @Override public Iterable<TreeFocus<T>> getChildren()
   {
      return () -> new FocusIterator<>(TreeFocus.this.firstChild());
   }

   @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
   private static class FocusIterator<T extends TreeInterface<T>> implements Iterator<TreeFocus<T>>
   {
      private Optional<TreeFocus<T>> nextFocus;

      FocusIterator(Optional<TreeFocus<T>> nextFocus)
      {
         this.nextFocus = nextFocus;
      }

      @Override public boolean hasNext()
      {
         return nextFocus.isPresent();
      }

      @Override public TreeFocus<T> next()
      {
         if (!hasNext())
            throw new IndexOutOfBoundsException("Iterator out of bounds");
         TreeFocus<T> result = nextFocus.orElse(null);
         nextFocus = nextFocus.flatMap(TreeFocus::nextSibling);
         return result;
      }
   }

   /**
    * Represents a path element from the root to this focus.
    * @param <T> tree type
    */
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

   public interface ChildIndexOf<T>
   {
      int indexOf(T parent, T child);

      default Optional<Integer> indexOfOptional(T parent, T child)
      {
         int index = indexOf(parent, child);
         if (index < 0)
            return Optional.empty();
         return Optional.of(index);
      }
   }

   public interface ParentOf<T>
   {
      @Nullable T getParentOf(T child);

      default Optional<T> getParentOfOptional(T child)
      {
         return Optional.ofNullable(getParentOf(child));
      }
   }
}
