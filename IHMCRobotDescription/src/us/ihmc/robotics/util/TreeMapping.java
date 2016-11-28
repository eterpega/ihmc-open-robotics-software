package us.ihmc.robotics.util;

import us.ihmc.robotics.util.TreeInterface.TreeNodeMapper;

import java.util.*;

/**
 * Maintains a bidirectional mapping between two trees where
 * the target tree is mutable.
 */
public class TreeMapping<T extends TreeInterface<T>, U>
{
   private final Map<T, U> mapping = new HashMap<>();
   private final Map<U, T> reverseMapping = new HashMap<>();
   private final TreeNodeMapper<T, U> mapper;

   public TreeMapping(TreeNodeMapper<T, U> mapper)
   {
      this.mapper = (node, children) ->
      {
         U mapped = mapper.mapNode(node, children);
         mapping.put(node, mapped);
         reverseMapping.put(mapped, node);
         return mapped;
      };
   }

   public U mapNewNode(T node)
   {
      return TreeInterface.map(node, mapper);
   }

   public void removeMapping(T node)
   {
      TreeInterface.flatten(node).forEach(childNode -> {
         get(childNode).ifPresent(reverseMapping::remove);
         mapping.remove(childNode);
      });
   }

   public void replaceSingleNode(T oldNode, T newNode)
   {
      get(oldNode).ifPresent(oldNodeMapped -> {
         mapping.remove(oldNode);
         mapping.put(newNode, oldNodeMapped);
         reverseMapping.put(oldNodeMapped, newNode);
      });
   }

   public Optional<U> get(T node)
   {
      return Optional.ofNullable(mapping.get(node));
   }

   public Optional<T> getReverse(U node)
   {
      return Optional.ofNullable(reverseMapping.get(node));
   }
}
