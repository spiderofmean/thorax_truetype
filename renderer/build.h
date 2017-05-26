#pragma once
#include "types.h"

struct BuilderBox {
    Box box;
    unsigned childOffset;
    unsigned index;
    unsigned size;
    unsigned PADDING;
};

template <size_t SIZE> struct BuilderBoxN {
    bool IsLeaf(size_t i) const { return data[i].childOffset == 0; }
    unsigned Index(size_t i) const { return data[i].index; }
    unsigned Size(size_t i) const { return data[i].size; }
    const BuilderBoxN* Child(size_t i) const { return this + data[i].childOffset; }
    void SetHead(unsigned head) { data[0].PADDING = head; }
    void SetTail(unsigned tail) { data[1].PADDING = tail; }
    void SetPerm(unsigned perm) { data[2].PADDING = perm; }
    void SetFlip(unsigned flip) { data[3].PADDING = flip; }
    unsigned GetHead() const { return data[0].PADDING; }
    unsigned GetTail() const { return data[1].PADDING; }
    unsigned GetPerm() const { return data[2].PADDING; }
    unsigned GetFlip() const { return data[3].PADDING; }
    BuilderBoxN* Child(size_t i) { return this + data[i].childOffset; }
    BuilderBox data[SIZE];
};

typedef BuilderBoxN<2> BuilderBox2;
typedef BuilderBoxN<4> BuilderBox4;

struct BuilderMesh {
    BuilderMesh() = default;
    BuilderMesh(const Shape* shapes, size_t numShapes);
    size_t TranscribeBVH(Box4Node* nodes, Shape* shapes, unsigned firstShape) const;

private:
    Array<BuilderBox4> node4s;
    const Shape* source;
    Array<unsigned> indices;
};

size_t QuickBuild(Box4Node* nodes, const Mesh* meshes, const Box4NodeRef* input, size_t numRefs);
