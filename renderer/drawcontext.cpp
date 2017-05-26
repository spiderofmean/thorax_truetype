#include "build.h"
#include "drawcontext.h"
#include <utility>
#include <cassert>
#include <cmath>
#include <cstdio>

#define DEBUG_NOINLINE
//#define DEBUG_NOINLINE __declspec(noinline)

struct UTMatrix {
    UTMatrix(float x_x, const Vector& y, const Point& w = Point(0, 0)) : x_x(x_x), y(y), w(w) {}
    Vector operator*(const Vector& a) const { return Vector(x_x * a.x + y.x * a.y, y.y * a.y); }
    Point operator*(const Point& a) const { return Point(x_x * a.x + y.x * a.y + w.x, y.y * a.y + w.y); }
    UTMatrix operator*(const UTMatrix& a) const { return UTMatrix(x_x * a.x_x, (*this) * a.y, (*this) * a.w); }
    float x_x;
    Vector y;
    Point w;
};

static inline UTMatrix invert(const UTMatrix& a) {
    auto scale = 1.0f / (a.x_x * a.y.y);
    auto x_x = a.y.y * scale;
    auto y = Vector(-a.y.x, a.x_x) * scale;
    auto w = Point(y.x * -a.w.y - x_x * a.w.x, y.y * -a.w.y);
    return UTMatrix(x_x, y, w);
}

static inline UTMatrix ComputeViewFromSample(Vector x, Vector y) {
    if (std::abs(y.x) > std::abs(x.x)) std::swap(x, y);
    auto xy = x.y;
    auto yy = y.y;
    // TODO: still buggy for smaller distortions
    if (std::abs(xy) * 30 < std::abs(x.x)) xy = 0; // Add dead-zone for highly anisotripic patterns.
    // TODO: guarantee that out.y.y > 0
    auto scale = 1 / std::sqrt(xy * xy + yy * yy);
    xy *= scale;
    yy *= scale;
    return UTMatrix(x.x * yy - y.x * xy, x * xy + y * yy);
}

//==============================================================================
// ShapeBuilderImpl
//==============================================================================

struct ShapeBuilderImpl final : ShapeBuilder {
    struct EdgeEqn {
        EdgeEqn(short2 vert, short2 next) : a((float)(next.x - vert.x) / (next.y - vert.y)), b(vert.x - a * vert.y) {}
        float XofY(float Y) const { return a * Y + b; }

        float a;
        float b;
    };

    struct Event {
        Event() = default;
        Event(unsigned short segmentID, bool isBegin, bool isUp, const short2& vert)
            : segmentID(segmentID), isBegin(isBegin), isUp(isUp), vert(vert) {}
        bool operator<(const Event& a) const { return vert.y < a.vert.y; }
        unsigned short segmentID;
        bool isBegin;
        bool isUp;
        short2 vert;
    };

    struct Edge {
        Edge() = default;
        Edge(float x, unsigned short segmentID, unsigned short slabID) : segmentID(segmentID), slabID(slabID), x(x) {}
        bool operator<(const Edge& a) const { return x < a.x; }
        float x;
        unsigned short segmentID;
        unsigned short slabID;
    };

    struct Slab {
        Slab() = default;
        Slab(float y_min, unsigned short upSegmentID, unsigned short dnSegmentID)
            : y_min(y_min), upSegmentID(upSegmentID), dnSegmentID(dnSegmentID) {}
        float y_min;
        unsigned short upSegmentID;
        unsigned short dnSegmentID;
    };

    void Clear(size_t reserve) override;
    size_t GenerateShapes(const Segment* segments,
                          const size_t* contourSizes,
                          size_t numCountours,
                          Shape* shapes) override;

    void ProcessSegment(short2 p0, short2 p1, short2 p2);

    void PushTrapezoid(const Slab& slab, float y_max) {
        auto& up = eqns[slab.upSegmentID];
        auto& dn = eqns[slab.dnSegmentID];
        auto y0 = slab.y_min;
        auto y1 = y_max;
        auto x0 = up.XofY(y0);
        auto x1 = up.XofY(y1);
        auto x2 = dn.XofY(y1);
        auto x3 = dn.XofY(y0);
        if (shapes) shapes[numShapes] = Shape(x0, x1, x2, x3, y0, y1);
        numShapes++;
    }

    DynamicArray<EdgeEqn> eqns;
    DynamicArray<Event> events;
    DynamicArray<Edge> upEdges;
    DynamicArray<Edge> dnEdges;
    DynamicArray<Slab> slabs;
    Shape* shapes;
    size_t numShapes;
};

void ShapeBuilderImpl::Clear(size_t reserve) {
    eqns = DynamicArray<EdgeEqn>(reserve);
    events = DynamicArray<Event>(reserve * 2);
    upEdges = DynamicArray<Edge>(reserve);
    dnEdges = DynamicArray<Edge>(reserve);
    slabs = DynamicArray<Slab>(reserve * 2);
}

void ShapeBuilderImpl::ProcessSegment(short2 p0, short2 p1, short2 p2) {
    // Create the curves and generate the trapezoid event list.
    if (p0.y == p2.y) return; // Ignore flat horizontal segments.

    auto v1_x = p1.x - p0.x;
    auto v1_y = p1.y - p0.y;
    auto v2_x = p2.x - p0.x;
    auto v2_y = p2.y - p0.y;
    // If we have a curved segment, push it.
    if (v2_x * v1_y != v1_x * v2_y) {
        if (shapes)
            shapes[numShapes] =
                Shape(Point(p0.x, p0.y), Vector((float)v1_x, (float)v1_y), Vector((float)v2_x, (float)v2_y));
        numShapes++;
    }

    auto segmentID = (unsigned short)eqns.size;
    if (p0.y < p2.y) {
        events.Push(segmentID, true, true, p0);
        events.Push(segmentID, false, true, p2);
    } else {
        events.Push(segmentID, false, false, p0);
        events.Push(segmentID, true, false, p2);
    }
    eqns.Push(p0, p2);
}

static void InsertEdge(DynamicArray<ShapeBuilderImpl::Edge>& edges,
                       float x,
                       unsigned short segmentID,
                       unsigned short slabID) {
    size_t i = edges.size;
    while (i && edges[i - 1].x < x) i--;
    for (auto j = edges.size++; j > i; j--) edges[j] = edges[j - 1];
    edges[i] = ShapeBuilderImpl::Edge(x, segmentID, slabID);
};

static unsigned short DeleteEdge(DynamicArray<ShapeBuilderImpl::Edge>& edges, unsigned short segmentID) {
    size_t i = 0;
    while (i < edges.size && edges[i].segmentID != segmentID) i++;
    auto slabID = edges[i].slabID;
    for (auto e = edges.size - 1; i < e; i++) edges[i] = edges[i + 1];
    edges.size--;
    return slabID;
};

size_t ShapeBuilderImpl::GenerateShapes(const Segment* segments,
                                        const size_t* contourSizes,
                                        size_t numCountours,
                                        Shape* out) {
    // Set up the outputs.
    shapes = out;
    numShapes = 0;

    // Reset the event and equations lists and repopulate them.  Process contour also
    events.Resize(0);
    eqns.Resize(0);
    for (auto pSize = contourSizes, end = contourSizes + numCountours; pSize < end; pSize++) {
        if (*pSize <= 1) continue;
        for (size_t i = 0, e = *pSize - 1; i < e; i++)
            ProcessSegment(segments[i].vert, segments[i].knot, segments[i + 1].vert);
        ProcessSegment(segments[*pSize - 1].vert, segments[*pSize - 1].knot, segments[0].vert);
        segments += *pSize;
    }
    if (!events.size) return numShapes;

    // Shell Sort the event list.
    static constexpr size_t gapSequence[] = {57, 23, 10, 4, 1};
    for (auto gap : gapSequence)
        for (auto i = gap, e = events.size; i < e; ++i)
            for (auto j = i - gap; j < e && events[j + gap] < events[j]; j -= gap)
                std::swap(events[j], events[j + gap]);

    // Walk the event list and create the trapezoid list.
    upEdges.Resize(0);
    dnEdges.Resize(0);
    slabs.Resize(0);
    auto event = events.begin();
    do {
        auto y = event->vert.y;
        for (auto& edge : upEdges) edge.x = eqns[edge.segmentID].XofY(y);
        for (auto& edge : dnEdges) edge.x = eqns[edge.segmentID].XofY(y);

        // Iterate over all of the events that occur at this y.
        auto baseSlabID = slabs.size;
        do {
            if (event->isUp) {
                if (event->isBegin) {
                    auto slabID = slabs.size;
                    slabs.Push(y, event->segmentID, (unsigned short)0);
                    InsertEdge(upEdges, event->vert.x, event->segmentID, (unsigned short)slabID);
                } else {
                    auto slabID = DeleteEdge(upEdges, event->segmentID);
                    PushTrapezoid(slabs[slabID], y);
                }
            } else {
                if (event->isBegin)
                    InsertEdge(dnEdges, event->vert.x, event->segmentID, (unsigned short)-1);
                else
                    DeleteEdge(dnEdges, event->segmentID);
            }
            ++event;
        } while (event != events.end() && event->vert.y == y);

        // Re-pair edges and adopt orphaned edges.
        auto numPairs = upEdges.size;
        for (size_t i = 0; i < numPairs; i++) {
            if (upEdges[i].slabID == dnEdges[i].slabID) continue;
            if (upEdges[i].slabID < baseSlabID) {
                PushTrapezoid(slabs[upEdges[i].slabID], y); // We split an old slab.
                auto slabID = slabs.size;
                slabs.Push(y, upEdges[i].segmentID, dnEdges[i].segmentID);
                upEdges[i].slabID = dnEdges[i].slabID = (unsigned short)slabID;
            } else {
                dnEdges[i].slabID = upEdges[i].slabID;
                slabs[dnEdges[i].slabID].dnSegmentID = dnEdges[i].segmentID;
            }
        }
        assert(numPairs == dnEdges.size);
    } while (event != events.end());
    return numShapes;
}

//==============================================================================
// TileImpl
//==============================================================================

static const size_t STAMP_WIDTH = 4;
static const size_t STAMP_HEIGHT = 2;
static const size_t TILE_WIDTH = 128;
static const size_t TILE_HEIGHT = 64;
static const ptrdiff_t TILE_STRIDE = TILE_WIDTH + STAMP_WIDTH;
static const size_t TILE_TOTAL_HEIGHT = TILE_HEIGHT + STAMP_HEIGHT - 1;
static const size_t TOTAL_TILE_SIZE = TILE_STRIDE * TILE_TOTAL_HEIGHT;

struct Tile {
    Tile(int x, int y, ptrdiff_t renderTargetStride, const Box& viewport, int tileWidth, int tileHeight)
        : pixelport(viewport & Box((float)x, (float)y, (float)x + (float)tileWidth, (float)y + (float)tileHeight)),
          tileAddressOffset(-(y * TILE_STRIDE + x)),
          renderTargetOffset(y * renderTargetStride + x),
          tileWidth(tileWidth),
          tileHeight(tileHeight) {}
    Box pixelport;
    int tileWidth;
    int tileHeight;
    ptrdiff_t tileAddressOffset;
    ptrdiff_t renderTargetOffset;
};

struct ThreadState {
    float8 red;
    float8 green;
    float8 blue;
    short* coverage;
    const Tile* tile;
};

struct PackedMatrix2x3 {
    PackedMatrix2x3(float x_x, float y_x, float w_x, float x_y, float y_y, float w_y)
        : xxyyww00(x_x, x_y, y_x, y_y, w_x, w_y, 0, 0) {}
    explicit PackedMatrix2x3(const Matrix2x3& a) {
        static const __declspec(align(32)) int MASK[] = { -1, -1, -1, -1, -1, -1, 0, 0 };
        xxyyww00 = float8::LoadU(&a.x.x) & *(const float8*)MASK;
    }
    explicit PackedMatrix2x3(float8 xxyyww00) : xxyyww00(xxyyww00) {}
    PackedMatrix2x3 operator*(const PackedMatrix2x3& a) const {
        auto v0v2 = shuffle4<0, 0>(xxyyww00) * shuffle<0, 0, 1, 1>(a.xxyyww00);
        auto v1v3 = shuffle4<0, 0>(xxyyww00) * shuffle<2, 2, 3, 3>(a.xxyyww00);
        return PackedMatrix2x3(shuffle<0, 1, 0, 1>(v0v2, v1v3) + shuffle<2, 3, 2, 3>(v0v2, v1v3) +
            shuffle4<8, 1>(xxyyww00));
    }
    void ComputeBounds(float8& mins, float8& maxs) const {
        auto v0123 = shuffle2<2, 2, 2, 2>(xxyyww00) + shuffle2<3, 0, 3, 0>(xxyyww00) + shuffle2<3, 3, 1, 1>(xxyyww00);
        auto v01 = v0123.v0123();
        auto v23 = v0123.v4567();
        auto vmin = min(v01, v23);
        vmin = min(vmin, shuffle<2, 3, 0, 1>(vmin));
        mins = float8(shuffle<0, 0, 0, 0>(vmin), shuffle<1, 1, 1, 1>(vmin));
        auto vmax = max(v01, v23);
        vmax = max(vmax, shuffle<2, 3, 0, 1>(vmax));
        maxs = float8(shuffle<0, 0, 0, 0>(vmax), shuffle<1, 1, 1, 1>(vmax));
    }
    float8 xxyyww00;
};

// Effectively a Matrix2x3 that can apply to 2 points and 2 vectors simultaneiously (matches the format of Shape)
struct ShapeTransform {
    ShapeTransform() = default;
    explicit ShapeTransform(const UTMatrix& a)
        : x(_mm256_castsi256_ps(_mm256_srli_epi64(_mm256_castps_si256(_mm256_broadcast_ss(&a.x_x)), 32))),
        y(_mm256_castpd_ps(_mm256_broadcast_sd((double*)&a.y.x))),
        w(_mm256_castsi256_ps(_mm256_srli_si256(_mm256_castpd_si256(_mm256_broadcast_sd((double*)&a.w.x)), 8))) {}
    explicit ShapeTransform(const Matrix2x3& a)
        : x(_mm256_castpd_ps(_mm256_broadcast_sd((double*)&a.x.x))),
        y(_mm256_castpd_ps(_mm256_broadcast_sd((double*)&a.y.x))),
        w(_mm256_castsi256_ps(_mm256_srli_si256(_mm256_castpd_si256(_mm256_broadcast_sd((double*)&a.w.x)), 8))) {}
    Shape operator*(const Shape& a) {
        return Shape(madd(shuffle<0, 0, 2, 2>(a.p0v1p2v3), x, madd(shuffle<1, 1, 3, 3>(a.p0v1p2v3), y, w)));
    }
    float8 x;
    float8 y;
    float8 w;
};

//==============================================================================
// DrawContextImpl
//==============================================================================

struct DrawContextImpl final : DrawContext {
    size_t SetRenderTarget(unsigned* renderTarget, int width, int height, size_t stride) override;
    void SetViewport(float lower_x, float lower_y, float upper_x, float upper_y) override;
    void SetFilterKernel(float x0, float y0, float x1, float y1) override;
    void Draw(const Mesh* scene, const Matrix2x3& worldFromScene) override;

    DrawContextImpl(unsigned numThreads);
    bool DrawTrapezoid(short* out, Shape trapezoid, const Box& pixelport);
    bool DrawCurve(short* out, Shape curve, const Box& pixelport);
    void Trace(const ThreadState& state, const Mesh* scene, const Box4Node* node, PackedMatrix2x3 objectFromWorld, PackedMatrix2x3 tileBounds);
    void Trace(const ThreadState& state, const Mesh* scene, const Box4Node* node, PackedMatrix2x3 objectFromWorld, PackedMatrix2x3 tileBounds, float8 mins, float8 maxs);
    void DrawTile(const ThreadState& state, const Mesh* scene, PackedMatrix2x3 layoutFromScreen);

    // RenderTarget related values.
    struct {
        unsigned* data;
        ptrdiff_t stride;
        int width;
        int height;
    } renderTarget;

    // Viewport related values.
    ShapeTransform screenFromWorld;
    Matrix2x3 worldFromScreen;

    // Filter kernel related values.
    ShapeTransform unitFromPixel;
    Box sampleBounds;

    // Local tile storage used during drawing.
    Array<short> coverageBuffer;
    Array<ThreadState> threadStates;
    Array<Tile> tiles;
};

DrawContextImpl::DrawContextImpl(unsigned numThreads) {
    coverageBuffer = Array<short>(TOTAL_TILE_SIZE * numThreads, 32);
    threadStates = Array<ThreadState>(numThreads);
    for (size_t i = 0; i < numThreads; i++)
        threadStates[i].coverage = coverageBuffer + TOTAL_TILE_SIZE * i;
}

DrawContext* DrawContext::Create(unsigned numThreads) {
    auto p = (DrawContextImpl*)_aligned_malloc(sizeof(DrawContextImpl), __alignof(DrawContextImpl));
    new (p) DrawContextImpl(numThreads);
    return p;
}

void DrawContext::Destroy(DrawContext*& context) {
    if (!context) return;
    _aligned_free(context);
    context = nullptr;
}

size_t DrawContextImpl::SetRenderTarget(unsigned* data, int width, int height, size_t stride) {
    // TODO: validate inputs.
    // Set up render-target.
    renderTarget.data = data + (height - 1) * stride;
    renderTarget.width = width;
    renderTarget.height = height;
    renderTarget.stride = -(ptrdiff_t)stride;

    // Initialize the tiles.
    auto viewport = Box(0, 0, (float)width, (float)height);
    auto numTiles = ((width + TILE_WIDTH - 1) / TILE_WIDTH) * ((height + TILE_HEIGHT - 1) / TILE_HEIGHT);
    tiles = Array<Tile>(numTiles);
    auto nextTile = tiles.data;
    for (int y = 0; y < height; y += (int)TILE_HEIGHT) {
        auto tileHeight = height - y < TILE_HEIGHT ? height - y : (int)TILE_HEIGHT;
        for (int x = 0; x < width; x += (int)TILE_WIDTH) {
            auto tileWidth = width - x < TILE_WIDTH ? width - x : (int)TILE_WIDTH;
            *nextTile++ = Tile(x, y, renderTarget.stride, viewport, tileWidth, tileHeight);
        }
    }
    return numTiles;
}

void DrawContextImpl::SetViewport(float lower_x, float lower_y, float upper_x, float upper_y) {
    // TODO: validate inputs
    auto xform =
        UTMatrix((upper_x - lower_x) / (float)renderTarget.width,
                 Vector(0, (upper_y - lower_y) / (float)renderTarget.height), Point(lower_x, lower_y));
    worldFromScreen = Matrix2x3(xform.x_x, xform.y.x, xform.w.x, 0, xform.y.y, xform.w.y);
    screenFromWorld = ShapeTransform(invert(xform));
}

void DrawContextImpl::SetFilterKernel(float x0, float y0, float x1, float y1) {
    // The spaces used during rendering:
    // World: origin is world origin, pixel spacing application defined
    // Screen : origin is viewport.lower, pixel spacing is 1
    // Pixel : pixel spacing is 1, the current pixel center is (0.5, 0.5)
    // View : same as pixel but the current pixel center is (0, 0)
    // Sample : pixel is at the origin, integration window is (-0.5, -0.5) -- (0.5, 0.5)
    // Unit: pixel integration window is (0, 0) -- (1, 1)

    // TODO: validate inputs
    // Compute unitFromPixel
    auto pixelFromView = UTMatrix(1, Vector(0, 1), Point(0.5f, 0.5f));
    auto viewFromSample = ComputeViewFromSample(Vector(x0, y0), Vector(x1, y1));
    auto sampleFromUnit = UTMatrix(1, Vector(0, 1), Point(-0.5f, -0.5f));
    auto pixelFromUnit = pixelFromView * viewFromSample * sampleFromUnit;
    unitFromPixel = ShapeTransform(invert(pixelFromUnit));

    // Compute sampleBounds
    auto v = viewFromSample;
    auto sampleDelta = Vector(std::abs(v.x_x) + std::abs(v.y.x), std::abs(v.y.y)) * 0.5f;
    sampleBounds = Box(-sampleDelta.x, -sampleDelta.y, sampleDelta.x, sampleDelta.y);
}

//==============================================================================
// Wide data structures for drawing.
//==============================================================================

struct Range8 {
    Range8() = default;
    Range8(float8 lower, float8 upper) : lower(lower), upper(upper) {}
    static Range8 Make(float8 a, float8 b) { return Range8(min(a, b), max(a, b)); }
    float8 lower, upper;
};

__forceinline static Range8 Cubic(const Range8& a) {
    auto cubic = [](float8 x) { return nmadd(x, float8(2), float8(3)) * (x * x); };
    return Range8(cubic(a.lower), cubic(a.upper));
}

__forceinline static Range8 Clamp(const Range8& x, const Range8& bounds = Range8(float8::Zero(), float8(1))) {
    return Range8(min(max(x.lower, bounds.lower), bounds.upper), min(max(x.upper, bounds.lower), bounds.upper));
}

struct LinearEqn8 {
    LinearEqn8(float8 a, float8 b, float8 i_a, const Range8& solve) : a(a), b(b), i_a(i_a), solve(solve) {}
    DEBUG_NOINLINE LinearEqn8(float A, float B) {
        a = float8(A);
        b = float8(B);
        // TODO: fix the case where A == 0
        i_a = float8(-1.0f / A);
        auto i_b = i_a * b;
        solve = Range8::Make(i_b, i_b - i_a);
    }
    Range8 operator()(const Range8& t) const { return Range8(madd(a, t.lower, b), madd(a, t.upper, b)); }
    __forceinline LinearEqn8 Apply(float8 x) const {
        return LinearEqn8(a, b + x, i_a, Range8(madd(i_a, x, solve.lower), madd(i_a, x, solve.upper)));
    }
    Range8 Solve() const { return solve; }
    float8 a, b;
    float8 i_a;
    Range8 solve;
};

struct QuadraticEqn8 {
    QuadraticEqn8(float8 a, float8 b, float8 c, float8 i_a, float8 i_b, const Range8& solve)
        : a(a), b(b), c(c), i_a(i_a), i_b(i_b), solve(solve) {}
    DEBUG_NOINLINE QuadraticEqn8(float A, float B, float C) {
        a = float8(A);
        b = float8(B);
        c = float8(C);
        // TODO: fix the case where A == 0
        i_a = float8(-1.0f / A);
        // TODO: write comment about the B == 0 branch.
        i_b = B == 0 ? float8::SignMask() : float8(0.5f * B) * i_a;
        auto i_c = madd(i_b, i_b, i_a * float8(C));
        solve = Range8::Make(i_c, i_c - i_a);
    }
    float8 operator()(float8 t) const { return madd(madd(a, t, b), t, c); }
    Range8 operator()(const Range8& t) const { return Range8(operator()(t.lower), operator()(t.upper)); }
    float8 SolveApex() const { return i_b; }
    Range8 SolveDelta() const {
        return Range8(sqrt(max(float8::Zero(), solve.lower)), sqrt(max(float8::Zero(), solve.upper)));
    }
    __forceinline QuadraticEqn8 Apply(float8 x) const {
        return QuadraticEqn8(a, b, c + x, i_a, i_b, Range8(madd(i_a, x, solve.lower), madd(i_a, x, solve.upper)));
    }
    float8 a, b, c;
    float8 i_a, i_b;
    Range8 solve;
};

struct TrapezoidEqns8 {
    explicit TrapezoidEqns8(const Shape& a)
        : y0(a.p0v1p2v3[3], a.p0v1p2v3[1]),
          x0(a.p0v1p2v3[2], a.p0v1p2v3[0]),
          x1(-a.p0v1p2v3[6], a.p0v1p2v3[4] + a.p0v1p2v3[6]) {}
    LinearEqn8 y0, x0, x1;
};

struct CurveEqns8 {
    explicit CurveEqns8(const Shape& a)
        : x0(-a.p0v1p2v3[6], a.p0v1p2v3[0] + a.p0v1p2v3[6]),
          y0(-a.p0v1p2v3[7], a.p0v1p2v3[1] + a.p0v1p2v3[7]),
          x1(a.p0v1p2v3[6] - 2 * a.p0v1p2v3[2], 2 * a.p0v1p2v3[2], a.p0v1p2v3[0]),
          y1(a.p0v1p2v3[7] - 2 * a.p0v1p2v3[3], 2 * a.p0v1p2v3[3], a.p0v1p2v3[1]) {}
    LinearEqn8 x0, y0;
    QuadraticEqn8 x1, y1;
};

//==============================================================================
// Drawing
//==============================================================================

__forceinline static bool SetupDraw(const Box& box,
                                    int& i_steps,
                                    int& j_steps,
                                    ptrdiff_t& p_offset,
                                    ptrdiff_t& p_stride,
                                    float8& x_start,
                                    float8& y_start) {
    auto origin = round_up(box.data);
    auto rect = int4(origin);
    auto mask = int4::True(rect.data); // create a mask of all 1s
    auto delta = rect + shuffle<2, 3, 0, 1>(rect);
    if (movemask(mask + delta)) return false;
    x_start = float8(origin[0]) - float8(0, 1, 2, 3, 0, 1, 2, 3);
    y_start = float8(origin[1]) - float8(0, 0, 0, 0, 1, 1, 1, 1);
    delta = delta + int4(STAMP_WIDTH - 1, STAMP_HEIGHT - 1, 0, 0);
    i_steps = delta[0] / STAMP_WIDTH;
    j_steps = delta[1] / STAMP_HEIGHT;
    p_offset = -rect[1] * TILE_STRIDE - rect[0];
    p_stride = STAMP_HEIGHT * TILE_STRIDE + STAMP_WIDTH - STAMP_WIDTH * i_steps;
    return true;
}

__forceinline static void WriteAVX(short* out, size_t stride, float8 value) {
    auto x = int8(float8(4096.0f) * value);
    auto y = short8::Pack(x);
    y = y + short8(*(long long*)out, *(long long*)(out + stride));
    *(long long*)out = y.v0123();
    *(long long*)(out + stride) = y.v4567();
}

DEBUG_NOINLINE float8 Integrate(const TrapezoidEqns8& eqns, float8 offset_x, float8 offset_y) {
    auto y0 = eqns.y0.Apply(offset_y);
    auto ty0 = Clamp(y0.Solve());
    auto yy0 = Cubic(y0(ty0));

    float8 area;
    {
        auto x0 = eqns.x0.Apply(offset_x);
        auto tx0 = Clamp(x0.Solve(), ty0);
        auto yx0 = Cubic(y0(tx0));
        auto yx0_mid = yx0.lower + yx0.upper;
        auto xx0 = Cubic(Clamp(x0(tx0)));
        area = xx0.lower * msub(float8(0.5f), yx0_mid, yy0.lower);
        area = madd(xx0.upper, nmadd(float8(0.5f), yx0_mid, yy0.upper), area);
    }
    {
        auto x1 = eqns.x1.Apply(offset_x);
        auto tx1 = Clamp(x1.Solve(), ty0);
        auto yx1 = Cubic(y0(tx1));
        auto yx1_mid = yx1.lower + yx1.upper;
        auto xx1 = Cubic(Clamp(x1(tx1)));
        area = nmadd(xx1.lower, msub(float8(0.5f), yx1_mid, yy0.lower), area);
        area = nmadd(xx1.upper, nmadd(float8(0.5f), yx1_mid, yy0.upper), area);
    }
    return area;
}

bool DrawContextImpl::DrawTrapezoid(short* out, Shape trapezoid, const Box& pixelport) {
    static const float8 x_step = float8(-(float)STAMP_WIDTH);
    static const float8 y_step = float8(-(float)STAMP_HEIGHT);
    int i_steps, j_steps;
    ptrdiff_t offset, stride;
    float8 x_start, y_start;

    trapezoid = screenFromWorld * trapezoid;
    auto box = trapezoid.Bound() + sampleBounds & pixelport;
    if (!SetupDraw(box, i_steps, j_steps, offset, stride, x_start, y_start)) return false;
    trapezoid = unitFromPixel * trapezoid; //< note that pixelFromScreen has been factored into the loop.
    TrapezoidEqns8 eqns(trapezoid);

    auto unitFromPixel_x_x = float8(unitFromPixel.x[0]);
    auto unitFromPixel_y_x = float8(unitFromPixel.y[0]);
    auto unitFromPixel_y_y = float8(unitFromPixel.y[1]);

    auto pixel_y = y_start;
    out += offset;
    auto j = j_steps;
    goto J_ENTRY;
    do {
        pixel_y += y_step;
        out += stride;
    J_ENTRY:
        auto unit_x = unitFromPixel_y_x * pixel_y;
        auto unit_y = unitFromPixel_y_y * pixel_y;
        auto pixel_x = x_start;
        auto i = i_steps;
        goto I_ENTRY;
        do {
            pixel_x += x_step;
            out += STAMP_WIDTH;
        I_ENTRY:
            auto value = Integrate(eqns, madd(unitFromPixel_x_x, pixel_x, unit_x), unit_y);
            WriteAVX(out, TILE_STRIDE, value);
        } while (--i);
    } while (--j);
    return true;
}

DEBUG_NOINLINE float8 Integrate(const CurveEqns8& eqns, float8 offset_x, float8 offset_y) {
    auto y0 = eqns.y0.Apply(offset_y);
    auto ty0 = Clamp(y0.Solve());
    auto yy0 = Cubic(y0(ty0));

    float8 area;
    {
        auto x0 = eqns.x0.Apply(offset_x);
        auto tx0 = Clamp(x0.Solve(), ty0);
        auto yx0 = Cubic(y0(tx0));
        auto yx0_mid = yx0.lower + yx0.upper;
        auto xx0 = Cubic(Clamp(x0(tx0)));
        area = xx0.lower * msub(float8(0.5f), yx0_mid, yy0.lower);
        area = madd(xx0.upper, nmadd(float8(0.5f), yx0_mid, yy0.upper), area);
    }
    {
        auto y1 = eqns.y1.Apply(offset_y);
        auto ty1_apex = y1.SolveApex();
        auto ty1_delta = y1.SolveDelta();
        auto ty1_apex_sign = ty1_apex & float8::SignMask();
        auto ty1 = Clamp(Range8(ty1_apex - (ty1_apex_sign | blend(ty1_delta.upper, ty1_delta.lower, ty1_apex_sign)),
                                ty1_apex - (ty1_apex_sign | blend(ty1_delta.lower, ty1_delta.upper, ty1_apex_sign))));

        auto x1 = eqns.x1.Apply(offset_x);
        auto tx1_apex = x1.SolveApex();
        auto tx1_delta = x1.SolveDelta();
        auto tx1_lower = Clamp(Range8(tx1_apex - tx1_delta.upper, tx1_apex - tx1_delta.lower), ty1);
        auto tx1_upper = Clamp(Range8(tx1_apex + tx1_delta.lower, tx1_apex + tx1_delta.upper), ty1);

        auto yx1_lower = Cubic(y1(tx1_lower));
        auto yx1_lower_mid = yx1_lower.lower + yx1_lower.upper;
        auto xx1_lower = Cubic(Clamp(x1(tx1_lower)));
        area = madd(xx1_lower.lower, msub(float8(0.5f), yx1_lower_mid, yy0.upper), area);
        area = madd(xx1_lower.upper, nmadd(float8(0.5f), yx1_lower_mid, yx1_lower.upper), area);

        auto yx1_upper = Cubic(y1(tx1_upper));
        auto yx1_upper_mid = yx1_upper.lower + yx1_upper.upper;
        auto xx1_upper = Cubic(Clamp(x1(tx1_upper)));
        area = madd(xx1_upper.lower, msub(float8(0.5f), yx1_upper_mid, yx1_lower.upper), area);
        area = madd(xx1_upper.upper, nmadd(float8(0.5f), yx1_upper_mid, yy0.lower), area);
    }
    return area;
}

bool DrawContextImpl::DrawCurve(short* out, Shape curve, const Box& pixelport) {
    static const float8 x_step = float8(-(float)STAMP_WIDTH);
    static const float8 y_step = float8(-(float)STAMP_HEIGHT);
    int i_steps, j_steps;
    ptrdiff_t offset, stride;
    float8 x_start, y_start;

    curve = screenFromWorld * curve;
    auto box = curve.Bound() + sampleBounds & pixelport;
    if (!SetupDraw(box, i_steps, j_steps, offset, stride, x_start, y_start)) return false;
    curve = unitFromPixel * curve; //< note that pixelFromScreen has been factored into the loop.
    CurveEqns8 eqns(curve);

    auto unitFromPixel_x_x = float8(unitFromPixel.x[0]);
    auto unitFromPixel_y_x = float8(unitFromPixel.y[0]);
    auto unitFromPixel_y_y = float8(unitFromPixel.y[1]);

    auto pixel_y = y_start;
    out += offset;
    auto j = j_steps;
    goto J_ENTRY;
    do {
        pixel_y += y_step;
        out += stride;
    J_ENTRY:
        auto unit_x = unitFromPixel_y_x * pixel_y;
        auto unit_y = unitFromPixel_y_y * pixel_y;
        auto pixel_x = x_start;
        auto i = i_steps;
        goto I_ENTRY;
        do {
            pixel_x += x_step;
            out += STAMP_WIDTH;
        I_ENTRY:
            auto value = Integrate(eqns, madd(unitFromPixel_x_x, pixel_x, unit_x), unit_y);
            WriteAVX(out, TILE_STRIDE, value);
        } while (--i);
    } while (--j);
    return true;
}

DEBUG_NOINLINE static void ClearCoverageTile(const short* data) {
    auto count = TILE_STRIDE * TILE_HEIGHT;
    auto p = (char*)(data + count);
    auto i = 0 - sizeof(short) * count;
    auto zero = float8::Zero();
    do {
        *(float8*)(p + i) = zero;
    } while (i += sizeof(float8));
}

DEBUG_NOINLINE static void ResolveCoverageTileAVX(
    unsigned* out, ptrdiff_t stride, const short* local, int width, int height, float8 r, float8 g, float8 b) {
    stride -= width;
    // TODO: fix me to use masked writes and support non-multiples of 16.
    do {
        size_t i = width / 16;
        do {
            auto x = abs(short16::loadu(local));
            x = min(srli<4>(x - srli<7>(x)), short16(0xff));
            auto lo = x.ZeroExtendUnpackLo();
            auto hi = x.ZeroExtendUnpackHi();
            // TODO: remove AVX instructions by using int8
            auto loR = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(lo.data)) * r).data);
            auto loG = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(lo.data)) * g).data);
            auto loB = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(lo.data)) * b).data);
            lo = _mm256_or_si256(loR, _mm256_or_si256(_mm256_slli_epi32(loG, 8), _mm256_slli_epi32(loB, 16)));
            auto hiR = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(hi.data)) * r).data);
            auto hiG = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(hi.data)) * g).data);
            auto hiB = _mm256_cvtps_epi32((float8(_mm256_cvtepi32_ps(hi.data)) * b).data);
            hi = _mm256_or_si256(hiR, _mm256_or_si256(_mm256_slli_epi32(hiG, 8), _mm256_slli_epi32(hiB, 16)));
            unpack4lo(lo, hi).storeu((int*)out);
            unpack4hi(lo, hi).storeu((int*)out + 8);
            local += 16;
            out += 16;
        } while (--i);
        local += TILE_STRIDE - width;
        out += stride;
    } while (--height);
}

DEBUG_NOINLINE static void ResolveCoverageTile(
    unsigned* out, ptrdiff_t stride, const short* local, int width, int height) {
    // TODO: remove me after fixing the AVX version to handle boarder regions
    for (size_t j = 0; j < height; j++)
        for (size_t i = 0; i < width; i++) {
            int x = local[j * TILE_STRIDE + i];
            x = abs(x);
            x = (x - (x >> 7)) >> 4;
            if (x > 0xff) x = 0xff;
            out[j * stride + i] = 0x10101 * x;
        }
}

__forceinline void DrawContextImpl::Trace(const ThreadState& state,
                                          const Mesh* scene,
                                          const Box4Node* node,
                                          PackedMatrix2x3 objectFromWorld,
                                          PackedMatrix2x3 tileBounds) {
    float8 mins, maxs;
    tileBounds.ComputeBounds(mins, maxs);
    Trace(state, scene, node, objectFromWorld, tileBounds, mins, maxs);
}

void DrawContextImpl::Trace(const ThreadState& state,
                            const Mesh* scene,
                            const Box4Node* node,
                            PackedMatrix2x3 objectFromWorld,
                            PackedMatrix2x3 tileBounds,
                            float8 mins,
                            float8 maxs) {
    auto mask = movemask((mins <= float8::Load(node->max_x)) & (float8::Load(node->min_x) <= maxs));
    mask = mask & (mask >> 4);

    for (unsigned i = 0; i < 4; i++) {
        // If we miss the box, continue.
        if (!(mask & (1 << i))) continue;

        // If the box is not a leaf traverse it.
        if (!node->IsLeaf(i)) {
            Trace(state, scene, node->Child(i), objectFromWorld, tileBounds, mins, maxs);
            continue;
        }

        // The box contains leaf geometry or references to other BVHs.
        auto& tile = *state.tile;
        auto worldFromObject = ShapeTransform(invert(*(const Matrix2x3*)&objectFromWorld));

        // Iterate through the leaf geometry/BVH references.
        for (auto p = node->ShapesAt(scene->shapes, i), e = node->ShapesAt(scene->shapes, i + 1); p < e; p++) {
            // If we hit a reference, update our transform and continue traversal.
            if (p->IsBox4NodeRef()) {
                auto ref = node->Box4NodeRefAt(scene->shapes, i);
                auto childFromObject = PackedMatrix2x3(ref->objectFromParent);
                Trace(state, scene + ref->meshIndex, scene[ref->meshIndex].nodes + ref->nodeIndex,
                      childFromObject * objectFromWorld, childFromObject * tileBounds);
                continue;
            }

            // The leaf geometry is a shape, draw it.
            auto shape = worldFromObject * *p;
            if (p->IsTrapazoid())
                DrawTrapezoid(state.coverage + tile.tileAddressOffset, shape, tile.pixelport);
            else
                DrawCurve(state.coverage + tile.tileAddressOffset, shape, tile.pixelport);
        }
    }
}

void DrawContextImpl::DrawTile(const ThreadState& state,
                               const Mesh* scene,
                               PackedMatrix2x3 layoutFromScreen) {
    auto coverage = state.coverage;
    auto& tile = *state.tile;

    // Compute the bounds of the beam to be traversed.
    auto pixelport = tile.pixelport + sampleBounds;
    auto tileBounds = PackedMatrix2x3(worldFromScreen) * PackedMatrix2x3(layoutFromScreen) *
        PackedMatrix2x3(pixelport.data[0] + pixelport.data[2], 0, -pixelport.data[0],
            0, pixelport.data[1] + pixelport.data[3], -pixelport.data[1]);

    ClearCoverageTile(coverage);
    Trace(state, scene, scene->nodes, layoutFromScreen, tileBounds);

    if (tile.tileWidth % 16)
        ResolveCoverageTile(renderTarget.data + tile.renderTargetOffset, renderTarget.stride, coverage,
                            tile.tileWidth, tile.tileHeight);
    else
        ResolveCoverageTileAVX(renderTarget.data + tile.renderTargetOffset, renderTarget.stride, coverage,
                               tile.tileWidth, tile.tileHeight, state.red, state.green, state.blue);
}

void DrawContextImpl::Draw(const Mesh* scene, const Matrix2x3& screenFromLayout) {
    if (!scene) return;
    auto layoutFromScreen = PackedMatrix2x3(invert(screenFromLayout));
    auto& state = threadStates[0];
    state.red = float8(0.4f);
    state.green = float8(1.0f);
    state.blue = float8(0.6f);
    for (size_t i = 0; i < tiles.size; i++) {
        state.tile = tiles + i;
        DrawTile(state, scene, layoutFromScreen);
    }
}

//==============================================================================
// FontRenderInfo
//==============================================================================

bool FontRenderInfo::Initialize(const Font* font) {
    Clear();
    FontInfo fontInfo;
    if (!fontInfo.Initialize(font)) return false;
    if (!fontInfo.maximumProfile->numGlyphs) return false;
    if (!fontInfo.maximumProfile->maxComponentElements) return false;

    // Load font metrics.
    emsPerUnit = 1.0f / (int)fontInfo.fontHeader->unitsPerEm;
    ascent = (float)(int)fontInfo.metricsHeader->ascent;
    descent = (float)(int)fontInfo.metricsHeader->descent;
    lineGap = (float)(int)fontInfo.metricsHeader->lineGap;
    caretSlopeRise = (float)(int)fontInfo.metricsHeader->caretSlopeRise;
    caretSlopeRun = (float)(int)fontInfo.metricsHeader->caretSlopeRun;
    caretOffset = (float)(int)fontInfo.metricsHeader->caretOffset;
    lineSpacing = ascent - descent + lineGap;

    size_t numGlyphs = fontInfo.maximumProfile->numGlyphs;

    // Calculate the codes for the sorted glyphs.
    struct GlyphCacheEntry {
        GlyphCacheEntry() = default;
        GlyphCacheEntry(const GlyphData* glyphData, int index) : glyphData(glyphData), index(index) {}
        const GlyphData* glyphData;
        int index;
    };
    DynamicArray<GlyphCacheEntry> cache(numGlyphs);
    size_t numSimpleGlyphs = 0;
    size_t numCompoundGlyphs = 0;
    for (unsigned i = 0, e = (unsigned)numGlyphs; i < e; i++) {
        auto glyphData = fontInfo.GetGlyphDataByIndex(i);
        if (glyphData && glyphData->numberOfContours < 0)
            cache.Push(glyphData, ~(int)numCompoundGlyphs++);
        else
            cache.Push(glyphData, (int)numSimpleGlyphs++);
    }

    // Create the codepointIndex table.
    codepointIndex = new int[0x10000];
    for (size_t i = 0; i < 0x10000; i++)
        codepointIndex[i] = cache[fontInfo.GetGlyphIndexByCodepoint((unsigned)i)].index;

    // Shape building buffers.
    DynamicArray<GlyphPoint> glyphPoints = DynamicArray<GlyphPoint>(fontInfo.maximumProfile->maxPoints);
    DynamicArray<Segment> segments = DynamicArray<Segment>(fontInfo.maximumProfile->maxPoints * 3);
    DynamicArray<size_t> contourSizes = DynamicArray<size_t>(fontInfo.maximumProfile->maxContours);
    ShapeBuilderImpl builder;
    builder.Clear(fontInfo.maximumProfile->maxPoints * 3);

    auto ProcessGlyph = [&glyphPoints, &segments, &contourSizes, &builder](const GlyphData* glyphData, Shape* shapes) {
        // Define a helper lambda for processing each contour.
        auto ProcessContour = [&segments, &contourSizes](const GlyphPoint* first, size_t numPoints) {
            auto GetValue = [](const GlyphPoint* a) { return short2{a->value_x, a->value_y}; };
            auto Average = [](const GlyphPoint* a, const GlyphPoint* b) {
                return short2{(short)((a->value_x + b->value_x) / 2), (short)((a->value_y + b->value_y) / 2)};
            };

            auto firstSegmentIndex = segments.size;
            auto last = first + numPoints - 1;
            short2 temp; // An on-curve point we haven't yet used in the output.
            if (first->IsOnCurve())
                temp = short2{first->value_x, first->value_y};
            else if (!last->IsOnCurve())
                segments.Push(Average(last, first), GetValue(first));
            for (auto point = first + 1; point <= last; point++) {
                if (point->IsOnCurve()) {
                    if (point[-1].IsOnCurve()) segments.Push(temp, temp);
                    temp = GetValue(point);
                } else {
                    if (!point[-1].IsOnCurve()) temp = Average(point - 1, point);
                    segments.Push(temp, GetValue(point));
                }
            }
            if (last->IsOnCurve()) segments.Push(temp, first->IsOnCurve() ? temp : GetValue(first));

            // Calculate the number of segments and push them onto the contour sizes.
            auto numSegments = segments.size - firstSegmentIndex;
            contourSizes.Push(numSegments);
        };

        // Decode the points into the glyphPoints buffer.
        if (!glyphData->DecodeGlyphPoints(glyphPoints.data)) return (size_t)0;

        // Process the contours to generate segments and contour sizes.
        segments.Resize(0);
        contourSizes.Resize(0);
        ProcessContour(glyphPoints.data, (size_t)glyphData->endPtsOfContours[0] + 1);
        for (size_t i = 1, e = (size_t)(short)glyphData->numberOfContours; i < e; ++i)
            ProcessContour(glyphPoints.data + glyphData->endPtsOfContours[i - 1] + 1,
                           glyphData->endPtsOfContours[i] - glyphData->endPtsOfContours[i - 1]);

        return builder.GenerateShapes(segments.data, contourSizes.data, contourSizes.size, shapes);
    };

    // Count all of the shapes and the compound elements.
    // Allocate the glyph info data structures.
    simpleGlyphInfos = Array<SimpleGlyphInfo>(numSimpleGlyphs + 1);
    compoundGlyphInfos = Array<CompoundGlyphInfo>(numCompoundGlyphs + 1);
    unsigned totalShapes = 0;
    unsigned totalCompoundElements = 0;
    for (size_t i = 0; i < numGlyphs; i++) {
        auto index = cache[i].index;
        auto glyphData = cache[i].glyphData;
        auto advanceWidth = (float)(int)fontInfo.GetHorizontalMetricByIndex((unsigned)i).advanceWidth;
        if (index >= 0) {
            auto count = glyphData ? (unsigned)ProcessGlyph(glyphData, nullptr) : 0;
            simpleGlyphInfos[index].advanceWidth = advanceWidth;
            simpleGlyphInfos[index].firstShape = totalShapes;
            totalShapes += count;
        } else {
            auto count = glyphData->DecodeCompoundGlyph(nullptr);
            compoundGlyphInfos[~index].advanceWidth = advanceWidth;
            compoundGlyphInfos[~index].firstElement = totalCompoundElements;
            totalCompoundElements += count;
        }
    }
    simpleGlyphInfos[numSimpleGlyphs].firstShape = totalShapes;
    compoundGlyphInfos[numCompoundGlyphs].firstElement = totalCompoundElements;

    // Allocate storage
    if (!totalShapes) return Clear(), false;

    // Allocate memory based on the number of elements.
    compoundElements = Array<CompoundElement>(totalCompoundElements);
    Array<Shape> tempShapes(totalShapes);
    Array<GlyphReference> references(fontInfo.maximumProfile->maxComponentElements);
    Array<BuilderMesh*> builders(numSimpleGlyphs);

    // Iterate through the glyphs a second time and fill out the now-allocated glyph data.
    auto totalNodeCount = 0u;
    for (size_t i = 0; i < numGlyphs; i++) {
        auto index = cache[i].index;
        auto glyphData = cache[i].glyphData;
        if (index >= 0) {
            auto& glyphInfo = simpleGlyphInfos[index];
            glyphInfo.entryNode = totalNodeCount;
            if (glyphData) {
                auto firstShape = tempShapes + (size_t)glyphInfo.firstShape;
                auto numShapes = ProcessGlyph(glyphData, firstShape);
                builders[index] = new BuilderMesh(firstShape, numShapes);
                totalNodeCount += (unsigned)builders[index]->TranscribeBVH(nullptr, nullptr, 0);
            } else
                builders[index] = nullptr;
        } else {
            // Decode the compound glyph and initialize the element list.
            auto numCompounds = glyphData->DecodeCompoundGlyph(references.data);
            auto elements = compoundElements + (size_t)compoundGlyphInfos[~index].firstElement;
            for (size_t j = 0; j < numCompounds; j++) {
                auto& ref = references[j];
                elements[j].glyphID = cache[ref.glyphIndex].index;
                elements[j].transform = Matrix2x3(ref.x_x, ref.y_x, ref.offset_x, ref.x_y, ref.y_y, ref.offset_y);
            }
        }
    }

    // Allocate final shape and node array and build the BVHs.
    nodes = Array<Box4Node>(totalNodeCount);
    shapes = Array<Shape>(totalShapes);
    for (size_t i = 0; i < numSimpleGlyphs; i++) {
        if (!builders[i]) continue;
        auto glyphInfo = simpleGlyphInfos + i;
        builders[i]->TranscribeBVH(nodes + (size_t)glyphInfo->entryNode, shapes, glyphInfo->firstShape);
        delete builders[i];
    }

    return true;
}

void FontRenderInfo::Clear() {
    codepointIndex = Array<int>();
    simpleGlyphInfos = Array<SimpleGlyphInfo>();
    compoundGlyphInfos = Array<CompoundGlyphInfo>();
    compoundElements = Array<CompoundElement>();
    shapes = Array<Shape>();
}

size_t FontRenderInfo::LayoutGlyphs(Box4NodeRef* glyphRefs,
                                    int meshIndex,
                                    const unsigned* codepoints,
                                    size_t numCodepoints) {
    if (!glyphRefs) {
        // Count the drawn glyphs.
        size_t drawnGlyphCount = 0;
        for (auto j = 0; j < numCodepoints; j++) {
            if (codepoints[j] == '\n') continue;
            auto index = codepointIndex[codepoints[j]];
            if (index >= 0) {
                auto glyphInfo = simpleGlyphInfos + (size_t)index;
                if (glyphInfo->entryNode != glyphInfo[1].entryNode) drawnGlyphCount++;
                continue;
            }
            auto compoundInfo = compoundGlyphInfos + (size_t)~index;
            for (auto i = compoundInfo->firstElement, e = compoundInfo[1].firstElement; i < e; i++) {
                auto glyphInfo = simpleGlyphInfos + (size_t)compoundElements[i].glyphID;
                if (glyphInfo->entryNode != glyphInfo[1].entryNode) drawnGlyphCount++;
            }
        }
        return drawnGlyphCount;
    }

    auto nextGlyphRef = glyphRefs;
    float pen_x = 0, pen_y = 0;
    for (auto j = 0; j < numCodepoints; j++) {
        // Handle newline.
        if (codepoints[j] == '\n') {
            pen_x = 0;
            pen_y -= lineSpacing;
            continue;
        }
        auto index = codepointIndex[codepoints[j]];
        if (index >= 0) {
            // Layout a simple glyph.
            auto glyphInfo = simpleGlyphInfos + (size_t)index;
            if (glyphInfo->entryNode != glyphInfo[1].entryNode) {
                nextGlyphRef->objectFromParent = invert(Matrix2x3(1, 0, pen_x, 0, 1, pen_y));
                nextGlyphRef->meshIndex = meshIndex;
                nextGlyphRef->nodeIndex = glyphInfo->entryNode;
                nextGlyphRef++;
            }
            pen_x += glyphInfo->advanceWidth;
            continue;
        }
        auto compoundInfo = compoundGlyphInfos + (size_t)~index;
        // Layout a compound glyph.
        for (auto i = compoundInfo->firstElement, e = compoundInfo[1].firstElement; i < e; i++) {
            auto& element = compoundElements[i];
            auto glyphInfo = simpleGlyphInfos + (size_t)element.glyphID;
            if (glyphInfo->entryNode != glyphInfo[1].entryNode) {
                nextGlyphRef->objectFromParent = invert(Matrix2x3(1, 0, pen_x, 0, 1, pen_y) * element.transform);
                nextGlyphRef->meshIndex = meshIndex;
                nextGlyphRef->nodeIndex = glyphInfo->entryNode;
                nextGlyphRef++;
            }
        }
        pen_x += compoundInfo->advanceWidth;
    }
    return nextGlyphRef - glyphRefs;
}
