# Modified

If a parent is modified then at least one of its children are modified.

## Set modified

Sets the node, its parent (and grandparent and so on), and all its children to modified.

## Reset modified

Reset modified from the node and all its children and propagate this information upwards.

# Access Specific Node

## Depth Order

In UFOMap, the root node is a the maximum depth and the leaf nodes are a depth $0$. The reason for this choice is multifold. First, it is easier to estimate the maximum resolution that is needed or feasible for a certain scenario than it is to estimate the size of the environment a priori. Second, it makes it easier to set default depth value in the API (i.e., `fun(depth = 0)`) as most of the time one wants to access the leaf nodes. Third, when not accessing leaf nodes, one wants to access nodes `d` levels above leaf level, and not `d` levels under root level. In this case, one simply has to use `d` instead of something like `map.depthLevels() - d`.

## Euclidean

### x, y, z, depth

### Point, depth

Same as x, y, z, depth but x, y, z bundled together:

```c++
struct Point 
{
  coord_t x{};
  coord_t y{};
  coord_t z{};

  // Functions ...
};
```

## Octree

### Key

Four components:

* x, y, z
  * Each corresponds to the index in a dimension of the Octree.
  * 32 bit unsigned integer.
* depth
  * The depth of the Octree the Key refers to.
  * 8 bit unsigned integer.

Example showing how to traverse an Octree to retrieve the Index of a node corresponding to a Key.

``` c++
Key key = ...;
pos_t node = 0; // The root is always at 0
for (depth_t d = rootDepth() - 1; key.depth < d; --d) {
  offset_t child = ((key.x >> d) & 0b1) | (((key.y >> d) & 0b1) << 1) | (((key.z >> d) & 0b1) << 2);
  node = children[node][child];
}
offset_t child = ((key.x >> key.depth) & 0b1) | (((key.y >> key.depth) & 0b1) << 1) | (((key.z >> key.depth) & 0b1) << 2);
return Index(node, child);
```

### Code

Same as Key but baked into a single 64 bit unsigned integer. The 5 least significant bits are for the depth.  In the 59 most significant bits the x, y, z components of the corresponding Key are interleaved. Bits shown below, with least significant to the left:
$[\neg d_0\ \neg d_1\ \neg d_2\ \neg d_3\ \neg d_4\ x_0\ y_0\ z_0\ x_1\ y_1\ z_1\ ...\ x_{17}\ y_{17}\ z_{17}\ x_{18}\ y_{18}\ z_{18}\ x_{19}\ y_{19}]$

The depth bits are stored in the least significant bits to ensure that codes for close parts of the tree are close when the codes are ordered. For efficient traversal of ordered codes, the depth bits are also negated ($\neg$). Meaning codes corresponding to nodes along the same branch of the tree will be ordered based on how close they are to the root. So parents come before children in ascending order. This is the same as depth first traversal order.

We make use of all bits to allow for a 21 level tree (root will be level $20$, leaf level $0$). In a tree with $21$ levels, the root node will only have 4 direct children, as there are not enough bits for the $z_{19}$ bit. This means that the x and y dimensions will be double that of the z dimension. In this case, level 19 is moved such that the four direct children of the root node is centered at $z=0$.

A three bit pair $x_d\ y_d\ z_d$ gives a number $[0..7]$ which corresponds to the child index when traversing from depth $d+1$ to depth $d$.

Having depth as least significant bits also makes it possible to easily get the root's children indices by just shifting, even for a $21$ level Octree.

Example showing how to traverse an Octree to retrieve the Index of a node corresponding to a Code.

``` c++
code_t code = ...;               // Fill in some code
depth_t depth = ~code & 0b11111; // Get the depth of the code
code >>= 5;                      // Remove the depth part of the code
pos_t node = 0;                // The root is always at 0
for (depth_t d = rootDepth() - 1; depth < d; --d) {
  offset_t child = (code >> 3*d) & 0b111;
  node = children[node][child];
}
offset_t child = (code >> 3*depth) & 0b111;
return Index(node, child);
```

With a leaf size of $0.01\ m$, the map is $x=y=0.01*2^{21}\ m \approx 21\ km$, $z=0.01*2^{20}\ m \approx 10\ km$, or $4611.69\ km^3$ large.

### Node

Consists of a Code and the `pos` of the Index.

## Container

Direct access, similar to vector's `operator[]`.

### Index

Contains a 32 bit unsigned position and a 8 bit offset.

Makes it possible to have a maximum of $8 \cdot 2^{32} = 34\ 359\ 738\ 368 \approx 34$ billion nodes in the tree.

Only use if you know what you are doing.

Safe to use if only for accessing data. Not safe if modifying, as these only alter the node and all children. Not the parents as the other. Also, it does not set the modified state.