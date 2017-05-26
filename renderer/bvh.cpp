//===- bvh.cpp -------------------------------------------------*- C++ --*-===//
// Copyright 2017  Warren Hunt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//===----------------------------------------------------------------------===//

#include "build.h"
#include <algorithm>
#include <cstdio>
#include <functional>

//#define VERBOSE

namespace {
struct BoxEvaluate {
    template <size_t SIZE> void operator()(const BuilderBoxN<SIZE>* node) {
        totalDepth = maxDepth = numNodes = numBoxes = 0;
        Traverse(node, 1);
        printf("num node%zus = %d\n", SIZE, numNodes);
        printf("num boxes = %d\n", numBoxes);
        printf("max depth = %d\n", maxDepth);
        printf("average depth = %f\n", totalDepth / (double)numNodes);
    }

    void operator()(const Box4Node* node) {
        totalDepth = maxDepth = numNodes = numBoxes = 0;
        Traverse(node, 1);
        printf("num node4s = %d\n", numNodes);
        printf("num boxes = %d\n", numBoxes);
        printf("max depth = %d\n", maxDepth);
        printf("average depth = %f\n", totalDepth / (double)numNodes);
    }

  private:
    template <size_t SIZE> void Traverse(const BuilderBoxN<SIZE>* node, unsigned depth) {
        totalDepth += depth;
        if (maxDepth < depth) maxDepth = depth;
        numNodes++;
        for (size_t i = 0; i < SIZE; i++) {
            if (node->IsLeaf(i))
                numBoxes += node->Size(i);
            else
                Traverse(node->Child(i), depth + 1);
        }
    }

    void Traverse(const Box4Node* node, unsigned depth) {
        totalDepth += depth;
        if (maxDepth < depth) maxDepth = depth;
        numNodes++;
        for (size_t i = 0; i < 4; i++) {
            if (node->leafMask & (1 << i))
                numBoxes += (node->data[1 + i] - node->data[i]) / sizeof(Shape);
            else
                Traverse(node->Child(i), depth + 1);
        }
    }

    unsigned totalDepth;
    unsigned maxDepth;
    unsigned numNodes;
    unsigned numBoxes;
};

float SurfaceArea(const Box& a) { return (a.data[2] + a.data[0]) * (a.data[3] + a.data[1]); };

unsigned BuildNode2(BuilderBox& out, BuilderBox2* node, float* temp, const Box** boxes, size_t index, size_t size) {
    // Initialize the box.
    out.index = (unsigned)index;
    out.size = (unsigned)size;
    out.childOffset = 0;
    if (size < 2) {
        assert(size == 1);
        out.box = **boxes;
        return 0;
    }

    // Define the sorting lambda.
    auto SortBoxes = [](const Box** boxes, size_t size, size_t axis) {
        std::sort(boxes, boxes + size, [axis](const Box*& a, const Box*& b) {
            return a->data[axis + 2] - a->data[axis] < b->data[axis + 2] - b->data[axis];
        });
    };

    // Scan the axes.
    size_t split = 0;
    float bestCost = floatInfinity();
    size_t bestAxis = 0;
    for (size_t axis = 0; axis <= 1; axis++) {
        SortBoxes(boxes, size, axis);

        // Scan in reverse to accumulate the right side cost into temp.
        Box box = Box::Empty();
        for (size_t i = 1; i < size; i++) {
            box |= *boxes[size - i];
            temp[size - i] = SurfaceArea(box) * (float)(int)i;
        }

        // Scan forward and evaluate SAH cost.
        box = Box::Empty();
        for (size_t i = 1; i < size; i++) {
            box |= *boxes[i - 1];
            auto cost = temp[i] + SurfaceArea(box) * (float)(int)i;
            if (bestCost <= cost) continue;
            bestCost = cost;
            split = i;
            bestAxis = axis;
        }
    }

    // Sort by bestAxis if we're not already sorted by it.
    if (bestAxis != 1) SortBoxes(boxes, size, bestAxis);

    // Build build the children and ourself.
    unsigned numNewNodes = 1;

    // Build the left sub-tree.
    if (auto count = BuildNode2(node->data[0], node + numNewNodes, temp, boxes, index, split)) {
        node->data[0].childOffset = numNewNodes;
        numNewNodes += count;
    }

    // Build the right sub-tree.
    if (auto count =
            BuildNode2(node->data[1], node + numNewNodes, temp + split, boxes + split, index + split, size - split)) {
        node->data[1].childOffset = numNewNodes;
        numNewNodes += count;
    }

    // Write the union of the boxes to your parent.
    out.box = node->data[0].box | node->data[1].box;

    // Check to see if we are better off not returning any nodes and return.
    return SurfaceArea(out.box) * (float)(int)(size - 2) < bestCost ? 0u : numNewNodes;
}

size_t BuildNode4(BuilderBox* parentCache, BuilderBox4*& outStream, const BuilderBox2* input) {
    BuilderBox cache[6];
    size_t cacheSize = 0;

    // Process the node's children.
    for (size_t i = 0; i < 2; i++)
        if (input->IsLeaf(i))
            cache[cacheSize++] = input->data[i];
        else
            cacheSize += BuildNode4(cache + cacheSize, outStream, input->Child(i));

    // If our cache is too small to make a node with, push it to our parent.
    if (cacheSize < 4) {
        for (size_t i = 0; i < cacheSize; i++) parentCache[i] = cache[i];
        return cacheSize;
    }

    // Find the grouping of 4 boxes with minimum cost.
    size_t j0Best = 4;
    size_t j1Best = 5;
    if (cacheSize > 4) {
        auto minCost = floatInfinity();
        if (cacheSize == 5) {
            for (size_t j0 = 0; j0 < 5; j0++) {
                Box box = Box::Empty();
                for (size_t i = 0; i < 5; i++)
                    if (i != j0) box |= cache[i].box;
                if (minCost > SurfaceArea(box)) {
                    minCost = SurfaceArea(box);
                    j0Best = j0;
                }
            }
        } else {
            for (size_t j1 = 1; j1 < 6; j1++) {
                for (size_t j0 = 0; j0 < j1; j0++) {
                    Box box = Box::Empty();
                    for (size_t i = 0; i < 6; i++)
                        if (i != j0 && i != j1) box |= cache[i].box;
                    if (minCost > SurfaceArea(box)) {
                        minCost = SurfaceArea(box);
                        j0Best = j0;
                        j1Best = j1;
                    }
                }
            }
        }
    }

    // Allocate a new node off of the stream.
    auto out = --outStream;

    // Allocate and initialize the parent box for the node we're about to create.
    auto parent = parentCache++;
    parent->box = Box::Empty();
    parent->index = parent->size;
    parent->childOffset = (unsigned)(out - (BuilderBox4*)nullptr);

    // Collect the 4 boxes that will make our new node.
    size_t cacheIndices[4];
    size_t* nextIndex = cacheIndices;
    size_t numLeaves = 0;
    for (size_t isChild = 0; isChild <= 1; isChild++)
        for (size_t i = 0; i < cacheSize; i++) {
            // Push leaves on the first pass and non-leaves on the second.
            if ((cache[i].childOffset == 0) != (isChild == 0)) continue;

            // Spill the unused portion of our cache to our parent.
            if (i == j0Best || i == j1Best) {
                *parentCache++ = cache[i];
                continue;
            }

            // Update the parent box with this cache entry's bounds.
            parent->box |= cache[i].box;

            if (cache[i].childOffset)
                // If the cache entry has an address compute the offset to it correctly.
                cache[i].childOffset -= (unsigned)(out - (BuilderBox4*)nullptr);
            else
                numLeaves++;

            // Push This cache entry into the output.
            *nextIndex++ = i;
        }

    constexpr unsigned char permTable[12][4] = {
        {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 3, 1}, {1, 0, 2, 3}, {1, 0, 3, 2}, {2, 0, 1, 3},
        {0, 2, 1, 3}, {0, 3, 1, 2}, {0, 3, 2, 1}, {1, 2, 0, 3}, {1, 3, 0, 2}, {2, 1, 0, 3},
    };

    constexpr unsigned char permLeafOkay[12] = {
        0xf, 0x7, 0x3, 0xd, 0x5, 0x9, 0xb, 0x3, 0x3, 0x9, 0x1, 0x9,
    };

    // Find the best perm and flips for this node.
    unsigned char bestPerm = 0;
    float bestCost = floatInfinity();
    for (unsigned char perm = 0; perm < 12; perm++) {
        if (!(permLeafOkay[perm] & (1 << (numLeaves & 0x3)))) continue;
        float cost = 0;
        for (unsigned i = 0; i < 3; i++)
            cost += SurfaceArea(cache[cacheIndices[permTable[perm][i + 0]]].box |
                                cache[cacheIndices[permTable[perm][i + 1]]].box);
        if (bestCost <= cost) continue;
        bestCost = cost;
        bestPerm = perm;
    }
    for (size_t i = 0; i < 4; i++) out->data[i] = cache[cacheIndices[permTable[bestPerm][i]]];

    // Return the number of nodes we pushed onto our parent's cache.
    return cacheSize - 3;
}

struct BuilderBox4Walker {
    static void Transcribe(Box4Node* nodes,
                           Shape* shapes,
                           const BuilderBox4* input,
                           const Shape* source,
                           const unsigned* indices,
                           unsigned firstShape) {
        BuilderBox4Walker walker(nodes, shapes, source, indices, firstShape);
        walker.TranscribeNode(input);

#ifdef VERBOSE
        printf("nodes copied : %zu\n", walker.nextNode - nodes);
        printf("shapes copied : %u\n", walker.index);
        BoxEvaluate()(nodes);
#endif
    }

    BuilderBox4Walker(Box4Node* nodes, Shape* shapes, const Shape* source, const unsigned* indices, unsigned firstShape)
        : nextNode(nodes), shapes(shapes), source(source), indices(indices), index(firstShape) {}

    void TranscribeNode(const BuilderBox4* input) {
        // Allocate and initialize a new node.
        auto node = nextNode++;
        node->leafMask = 0;
        node->data[0] = index * (unsigned)sizeof(Shape);

        for (size_t i = 0; i < 4; i++) {
            // Copy box data.
            node->min_x[i] = -input->data[i].box.data[0];
            node->min_y[i] = -input->data[i].box.data[1];
            node->max_x[i] = input->data[i].box.data[2];
            node->max_y[i] = input->data[i].box.data[3];

            // Copy shapes or recurse.
            if (input->IsLeaf(i)) {
                node->leafMask |= (1 << i);
                auto src = indices + input->Index(i);
                for (auto e = src + input->Size(i); src < e; src++) shapes[index++] = source[*src];
                node->data[1 + i] = index * (unsigned)sizeof(Shape);
            } else {
                node->data[1 + i] = (unsigned)((char*)nextNode - (char*)node);
                TranscribeNode(input->Child(i));
            }
        }
    }

    const Shape* source;
    const unsigned* indices;
    Box4Node* nextNode;
    Shape* shapes;
    unsigned index;
};

void Refit(BuilderBox2* node, const unsigned* indices, const Shape* shapes) {
    for (size_t i = 0; i < 2; i++) {
        if (node->IsLeaf(i)) {
            for (auto j = node->Index(i), e = node->Index(i) + node->Size(i); j < e; j++)
                node->data[i].box |= shapes[indices[j]].Bound();
        } else {
            node->data[i].box = Box::Empty();
            Refit(node->Child(i), indices, shapes);
            node->data[i].box = node->Child(i)->data[0].box | node->Child(i)->data[1].box;
        }
    }
}

void Refit(BuilderBox4* node, const unsigned* indices, const Shape* shapes) {
    for (size_t i = 0; i < 4; i++) {
        node->data[i].box = Box::Empty();
        if (node->IsLeaf(i)) {
            for (auto j = node->Index(i), e = node->Index(i) + node->Size(i); j < e; j++)
                node->data[i].box |= shapes[indices[j]].Bound();
        } else {
            Refit(node->Child(i), indices, shapes);
            node->data[i].box = node->Child(i)->data[0].box | node->Child(i)->data[1].box |
                                node->Child(i)->data[2].box | node->Child(i)->data[3].box;
        }
    }
}

void Refit(Box4Node* node, const Shape* shapes) {
    for (size_t i = 0; i < 4; i++) {
        Box box = Box::Empty();
        if (node->IsLeaf(i)) {
            for (auto shape = node->ShapesAt(shapes, i), e = node->ShapesAt(shapes, i + 1); shape < e; shape++)
                box |= shape->Bound();
        } else {
            Refit(node->Child(i), shapes);
            for (size_t j = 0; j < 4; j++)
                box |= Box(node->Child(i)->min_x[j], node->Child(i)->min_y[j], node->Child(i)->max_x[j],
                           node->Child(i)->max_y[j]);
        }
        node->min_x[i] = -box.data[0];
        node->min_y[i] = -box.data[1];
        node->max_x[i] = box.data[2];
        node->max_y[i] = box.data[3];
    }
}

float TestCloseness(const Shape* shapes, size_t numShapes) {
    float area = 0;
    for (size_t i = 1; i < numShapes; i++) area += SurfaceArea(shapes[i - 1].Bound() | shapes[i].Bound());
    return area;
}
} // namespace {

BuilderMesh::BuilderMesh(const Shape* shapes, size_t numShapes) : source(shapes), indices(numShapes) {
    if (!shapes || numShapes == 0) return;

    auto ClearBuilderBox = [](BuilderBox& builderBox) {
        builderBox.box = Box::Empty();
        builderBox.childOffset = 0;
        builderBox.index = 0;
        builderBox.size = 0;
    };

    auto InitializeFromBox = [=](const BuilderBox& builderBox) {
        node4s = Array<BuilderBox4>(1);
        for (size_t i = 0; i < 3; i++) ClearBuilderBox(node4s[0].data[i]);
        node4s[0].data[3] = builderBox;
        for (size_t i = 0; i < numShapes; i++) indices[i] = (unsigned)i;
#ifdef VERBOSE
        BoxEvaluate()(node4s.data);
#endif
    };

    // Root node, output from the recursive build algorithms (not used other than as a sink).
    BuilderBox root;

    if (numShapes == 1) {
        ClearBuilderBox(root);
        for (size_t i = 0; i < numShapes; i++) root.box |= shapes[i].Bound();
        root.size = (unsigned)numShapes;
        InitializeFromBox(root);
        return;
    }

    // Allocate a conservative number of nodes for node2s array.
    auto node2s = Array<BuilderBox2>(numShapes - 1);
    size_t numNode2s = 0;

    // Build the node2s array.
    {
        auto ptrs = Array<const Box*>(numShapes);
        auto temp = Array<float>(numShapes);
        auto boxes = Array<Box>(numShapes);

        // Initialize the boxes and pointers.
        for (size_t i = 0; i < numShapes; i++) {
            boxes[i] = shapes[i].Bound();
            ptrs[i] = boxes.data + i;
        }

        // Recursively build the node2 array.
        numNode2s = BuildNode2(root, node2s.data, temp.data, ptrs.data, 0, numShapes);
#ifdef VERBOSE
        printf("root = %f, %f .. %f, %f\n", -root.box.data[0], -root.box.data[1], root.box.data[2], root.box.data[3]);
#endif
        // Create the index array
        for (size_t i = 0; i < numShapes; i++) indices[i] = (unsigned)(ptrs[i] - boxes.data);

#ifdef VERBOSE
        BoxEvaluate()(node2s.data);
#endif
    }

    // Check to see if we didn't actually build a node-2.
    if (!numNode2s) {
        InitializeFromBox(root);
        return;
    }

    // If we don't have enough node2s to make a full node4, build a custom node4.
    if (numNode2s < 4) {
        node4s = Array<BuilderBox4>(1);
        auto next = node4s[0].data + 4;
        for (auto node = node2s.data + numNode2s - 1; node >= node2s.data; node--)
            for (size_t i = 1; i < 2; i--) // note: underflow
                if (node->IsLeaf(i)) *(--next) = node->data[i];
        while (next != node4s[0].data) ClearBuilderBox(*--next);
#ifdef VERBOSE
        BoxEvaluate()(node4s.data);
#endif
        return;
    }

    // Prepare the node2s array by collapsing leaves until we have 3n + 1 of them.
    auto numNode4s = 0u;
    for (auto numLeafBoxes = (unsigned)numNode2s + 1;; numLeafBoxes--) {
        if (numLeafBoxes % 3 == 1) {
            numNode4s = numLeafBoxes / 3;
            break;
        }

        // Initialize our lowest cost node.
        auto leastCost = floatInfinity();
        BuilderBox2* leastCostNode = nullptr;
        unsigned* leastCostOffset = nullptr;

        // Loop through the nodes backwards (children will be traversed before parents).
        for (auto node = node2s.data + numNode2s - 1; node >= node2s.data; node--) {
            // Check to see if this is a dead node.
            if (node->data[0].size + node->data[1].size == 0) continue;

            for (size_t i = 0; i < 2; i++)
                if (!node->IsLeaf(i) && node->Child(i) == leastCostNode) leastCostOffset = &node->data[i].childOffset;

            // Check to see if we're not a leaf.
            if (node->data[0].childOffset | node->data[1].childOffset) continue;

            // Determine if this is our least cost node.
            auto cost = SurfaceArea(node->data[0].box | node->data[1].box) * (node->data[0].size + node->data[1].size);
            if (leastCost > cost) {
                leastCost = cost;
                leastCostNode = node;
            }
        }

        // Nuke the least cost box and continue.
        *leastCostOffset = leastCostNode->data[0].size = leastCostNode->data[1].size = 0;
#ifdef VERBOSE
        BoxEvaluate()(node2s.data);
#endif
    };

    // Build the node4s array.
    node4s = Array<BuilderBox4>(numNode4s);
    auto node4Stream = node4s.data + numNode4s;
    BuildNode4(&root, node4Stream, node2s.data);
#ifdef VERBOSE
    BoxEvaluate()(node4s.data);
#endif
}

size_t BuilderMesh::TranscribeBVH(Box4Node* nodes, Shape* shapes, unsigned firstShape) const {
    if (!nodes || !node4s.size) return node4s.size;

    BuilderBox4Walker::Transcribe(nodes, shapes, node4s.data, source, indices.data, firstShape);
#ifdef VERBOSE
    printf("chained cost = %g -> %g\n", TestCloseness(source, indices.size), TestCloseness(shapes, indices.size));
#endif
    return node4s.size;
}

#if 0
    for (size_t i = 1; i < 66; i++) {
        auto numNodes = (i + 1) / 3;
        auto extra = i == 1 ? 3 : 2 - (i + 1) % 3;
        auto leaves = i + extra;

        auto power4 = (size_t)1;
        while (power4 * 4 <= leaves) power4 *= 4;
        auto spilled = leaves - power4;
        auto rightOuterNodes = spilled / 3;
        auto rightInnerNodes = rightOuterNodes / 4;
        auto right = rightOuterNodes * 4;

        auto split = leaves % 4;
        auto splitNodes = (size_t)(split ? 1 : 0);

        auto left = leaves - split - right;
        auto leftNodes = left / 4;
        auto innerNodes = numNodes - leftNodes - splitNodes - rightInnerNodes - rightOuterNodes;
        printf("%2zu = %2zu + %zu = %2zu + %2zu = %2zu + %zu + %2zu : %2zu = %zu + %zu + %zu + %zu + %zu\n", leaves, i,
               extra, power4, spilled, left, split, right, numNodes, innerNodes, leftNodes, splitNodes, rightInnerNodes,
               rightOuterNodes);
    }
#endif

size_t QuickBuild(Box4Node* nodes, const Mesh* scene, const Box4NodeRef* input, size_t numRefs) {
    if (!numRefs) return 0;
    auto numNodes = (numRefs + 1) / 3;
    auto extra = 2 - (numRefs + 1) % 3;
    if (numRefs == 1) {
        numNodes = 1;
        extra = 3;
    }

    if (!nodes) return numNodes;

    auto outNodeID = numNodes;
    auto nextNodeID = numNodes;
    auto nextRefID = numRefs;

    auto AllocateOutNode = [&](unsigned mask) -> Box4Node& {
        auto& out = nodes[--outNodeID];
        out.leafMask = mask;
        return out;
    };

    auto InitOutBox = [](Box4Node& out, size_t i, const Box& box) {
        out.min_x[i] = -box.data[0];
        out.min_y[i] = -box.data[1];
        out.max_x[i] = box.data[2];
        out.max_y[i] = box.data[3];
    };

    auto MakeEmptyBox = [&](Box4Node& out, size_t i) {
        InitOutBox(out, i, Box::Empty());
        out.data[i] = out.data[i + 1];
    };

    auto MakeLeafBox = [&](Box4Node& out, size_t i) {
        auto& ref = input[--nextRefID];
        InitOutBox(out, i, scene[ref.meshIndex].nodes[ref.nodeIndex].Bound(invert(ref.objectFromParent)));
        out.data[i] = out.data[i + 1] - (unsigned)sizeof(Box4NodeRef);
    };

    auto MakeNodeBox = [&](Box4Node& out, size_t i) {
        auto& node = nodes[--nextNodeID];
        InitOutBox(out, i, node.Bound());
        out.data[i + 1] = (unsigned)((char*)&node - (char*)&out);
    };

    auto MakeOuterNode = [&]() {
        auto& out = AllocateOutNode(0xf);
        out.data[4] = (unsigned)(nextRefID * sizeof(Box4NodeRef));
        size_t i = 3;
        for (; extra; i--, extra--) MakeEmptyBox(out, i);
        for (; i < 4; i--) MakeLeafBox(out, i);
    };

    auto MakeInnerNode = [&]() {
        auto& out = AllocateOutNode(0);
        out.data[0] = 0;
        for (size_t i = 3; i < 4; i--) MakeNodeBox(out, i);
    };

    auto MakeSplitNode = [&](size_t split) {
        auto& out = AllocateOutNode((1u << (unsigned)split) - 1);
        out.data[split] = (unsigned)(nextRefID * sizeof(Box4NodeRef));
        for (size_t i = 3; i >= split; i--) MakeNodeBox(out, i);
        for (size_t i = split - 1; i < 4; i--) MakeLeafBox(out, i);
    };

    auto leaves = numRefs + extra;
    auto power4 = (size_t)1;
    while (power4 * 4 <= leaves) power4 *= 4;
    auto spilled = leaves - power4;
    auto rightOuterNodes = spilled / 3;
    auto rightInnerNodes = rightOuterNodes / 4;
    auto right = rightOuterNodes * 4;

    auto split = leaves % 4;
    auto splitNodes = (size_t)(split ? 1 : 0);

    auto left = leaves - split - right;
    auto leftOuterNodes = left / 4;
    auto innerNodes = numNodes - leftOuterNodes - splitNodes - rightInnerNodes - rightOuterNodes;

    //printf("%2zu = %2zu + %zu = %2zu + %2zu = %2zu + %zu + %2zu : %2zu = %zu + %zu + %zu + %zu + %zu\n", leaves,
    //       numRefs, extra, power4, spilled, left, split, right, numNodes, innerNodes, leftOuterNodes, splitNodes,
    //       rightInnerNodes, rightOuterNodes);

    for (; rightOuterNodes; rightOuterNodes--) MakeOuterNode();
    for (; rightInnerNodes; rightInnerNodes--) MakeInnerNode();
    if (splitNodes) MakeSplitNode(split);
    for (; leftOuterNodes; leftOuterNodes--) MakeOuterNode();
    for (; innerNodes; innerNodes--) MakeInnerNode();

    return numNodes;
}
