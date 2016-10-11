package us.ihmc.robotbuilder.util;

import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.HashMap;
import javaslang.collection.List;
import javaslang.collection.Map;
import javaslang.collection.Set;

import java.security.InvalidParameterException;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 *
 */
public class TreeDifference
{

   public static <T> DifferencesByNode<T> difference(Tree<T> original, Tree<T> otherTree)
   {
      return new DifferencesByNode<>(differenceImpl(original, otherTree));
   }

   private static <T> Map<Tree<T>, List<Difference<T>>> differenceImpl(Tree<T> original, Tree<T> otherTree)
   {
      List<Difference<T>> differences = List.empty();
      if (original == otherTree)
         return HashMap.empty();

      differences = differences.prepend(new NodeChanged<>(original, otherTree));

      Map<Tree<T>, Integer> myIndex = indexChildren(original);
      Map<Tree<T>, Integer> otherIndex = indexChildren(otherTree);

      Set<Tree<T>> sameChildren = otherIndex.keySet().filter(myIndex::containsKey);
      Set<Tree<T>> addedChildren = otherIndex.keySet().filter(otherChild -> !myIndex.containsKey(otherChild));
      Set<Tree<T>> removedChildren = myIndex.keySet().filter(myChild -> !otherIndex.containsKey(myChild));

      Map<Tree<T>, Tree<T>> replacedChildren = findReplacedChildren(addedChildren, removedChildren);
      Set<Tree<T>> replacements = replacedChildren.values().toSet();
      addedChildren = addedChildren.filter(child -> !replacements.contains(child));
      removedChildren = removedChildren.filter(child -> !replacedChildren.containsKey(child));

      differences = differences.prepend(new ChildrenAdded<>(original, addedChildren));
      differences = differences.prepend(new ChildrenRemoved<>(original, removedChildren));

      boolean childOrderChanged = addedChildren.size() > 0 ||
            removedChildren.size() > 0 ||
            replacedChildren.exists(child -> (int)myIndex.get(child._1).get() != otherIndex.get(child._2).get()) ||
            sameChildren.exists(child -> (int)myIndex.get(child).get() != otherIndex.get(child).get());

      if (childOrderChanged)
      {
         differences = differences.prepend(new ChildOrderChanged<>(original, otherIndex));
      }

      return replacedChildren
                  .map(replacement -> differenceImpl(replacement._1, replacement._2))
                  .fold(HashMap.empty(), Map::merge)
                  .put(original, differences);
   }

   public static class DifferencesByNode<T>
   {
      private final Map<Tree<T>, List<Difference<T>>> differencesByNode;

      public DifferencesByNode(Map<Tree<T>, List<Difference<T>>> differencesByNode)
      {
         this.differencesByNode = differencesByNode;
      }

      public List<Difference<T>> get(Tree<T> node)
      {
         return differencesByNode.get(node).getOrElse(List.empty());
      }

      public Stream<Difference<T>> getStream(Tree<T> node)
      {
         return get(node).toJavaStream();
      }
   }

   public static abstract class Difference<T> {
      private final Tree<T> node;

      Difference(Tree<T> node)
      {
         this.node = node;
      }

      public Tree<T> getNode()
      {
         return node;
      }

      public <R> R match(Function<NodeChanged<T>, R> matchNodeValueChanged,
                         Function<ChildrenAdded<T>, R> matchChildAdded,
                         Function<ChildrenRemoved<T>, R> matchChildRemoved,
                         Function<ChildOrderChanged<T>, R> matchChildOrderChanged)
      {
         if (this instanceof NodeChanged)
            return matchNodeValueChanged.apply((NodeChanged<T>) this);
         else if (this instanceof ChildrenAdded)
            return matchChildAdded.apply((ChildrenAdded<T>) this);
         else if (this instanceof ChildrenRemoved)
            return matchChildRemoved.apply((ChildrenRemoved<T>) this);
         else if (this instanceof ChildOrderChanged)
            return matchChildOrderChanged.apply((ChildOrderChanged<T>) this);

         throw new InvalidParameterException(getClass().getSimpleName() + " is not a recognized Difference");
      }

      public void match(Consumer<NodeChanged<T>> matchNodeValueChanged,
                        Consumer<ChildrenAdded<T>> matchChildAdded,
                        Consumer<ChildrenRemoved<T>> matchChildRemoved,
                        Consumer<ChildOrderChanged<T>> matchChildMoved)
      {
         match(consumerToFunction(matchNodeValueChanged),
               consumerToFunction(matchChildAdded),
               consumerToFunction(matchChildRemoved),
               consumerToFunction(matchChildMoved)
               );
      }

      private static <T, Void> Function<T, Void> consumerToFunction(Consumer<T> consumer)
      {
         return x -> {
            consumer.accept(x);
            return null;
         };
      }
   }


   public static final class NodeChanged<T> extends Difference<T> {
      private final Tree<T> newNode;

      NodeChanged(Tree<T> node, Tree<T> newNode)
      {
         super(node);
         this.newNode = newNode;
      }

      public Tree<T> getNewNode()
      {
         return newNode;
      }
   }

   public static final class ChildrenAdded<T> extends Difference<T> {
      private final Set<Tree<T>> newChildren;

      ChildrenAdded(Tree<T> node, Set<Tree<T>> newChildren)
      {
         super(node);
         this.newChildren = newChildren;
      }

      public Set<Tree<T>> getNewChildren()
      {
         return newChildren;
      }
   }

   public static final class ChildrenRemoved<T> extends Difference<T> {
      private final Set<Tree<T>> removedChildren;

      ChildrenRemoved(Tree<T> node, Set<Tree<T>> removedChildren)
      {
         super(node);
         this.removedChildren = removedChildren;
      }

      public Set<Tree<T>> getRemovedChildren()
      {
         return removedChildren;
      }
   }

   public static final class ChildOrderChanged<T> extends Difference<T> {
      private final Map<Tree<T>, Integer> childIndices;

      ChildOrderChanged(Tree<T> parent, Map<Tree<T>, Integer> childIndices)
      {
         super(parent);
         this.childIndices = childIndices;
      }

      public Map<Tree<T>, Integer> getChildIndices()
      {
         return childIndices;
      }
   }

   private static <T> Map<Tree<T>, Integer> indexChildren(Tree<T> tree)
   {
      return HashMap.ofEntries(tree.children().zipWithIndex()).mapValues(x -> (int)(long)x);
   }

   private static <T> Map<Tree<T>, Tree<T>> findReplacedChildren(Set<Tree<T>> addedChildren, Set<Tree<T>> removedChildren)
   {
      List<Tuple2<Tree<T>, Tree<T>>> result = List.empty();
      Set<Tree<T>> remainingChildren = addedChildren;
      for (Tree<T> removedChild : removedChildren)
      {
         if (remainingChildren.isEmpty())
            break;
         Tree<T> replacement = findClosestTree(removedChild, remainingChildren);
         result = result.prepend(Tuple.of(removedChild, replacement));
         remainingChildren = remainingChildren.remove(replacement);
      }
      return HashMap.ofEntries(result);
   }

   private static <T> Tree<T> findClosestTree(Tree<T> base, Set<Tree<T>> others)
   {
      class ScoredTree {
         private final Tree<T> tree;
         private final int score;

         private ScoredTree(Tree<T> tree, int score)
         {
            this.tree = tree;
            this.score = score;
         }
      }

      return others.map(tree -> new ScoredTree(tree, computeChildSimilarity(base, tree)))
                   .reduce((tree1, tree2) -> tree1.score > tree2.score ? tree1 : tree2)
            .tree;
   }

   private static <T> int computeChildSimilarity(Tree<T> tree1, Tree<T> tree2)
   {
      Set<Tree<T>> otherChildrenSet = tree2.children().toSet();
      return tree1.children().count(otherChildrenSet::contains);
   }
}
