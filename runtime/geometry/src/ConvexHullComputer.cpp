#include "PCH.h"

#include "ConvexHullComputer.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>

namespace Radion::Geometry
{

namespace
{

constexpr f32 kInfinity = std::numeric_limits<f32>::infinity();

Math::Vec3 vecMin(const Math::Vec3& a, const Math::Vec3& b)
{
    return Math::Vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

Math::Vec3 vecMax(const Math::Vec3& a, const Math::Vec3& b)
{
    return Math::Vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

int vecMinAxis(const Math::Vec3& v)
{
    return (v.x < v.y) ? ((v.x < v.z) ? 0 : 2) : ((v.y < v.z) ? 1 : 2);
}

int vecMaxAxis(const Math::Vec3& v)
{
    return (v.x < v.y) ? ((v.y < v.z) ? 2 : 1) : ((v.x < v.z) ? 2 : 0);
}

class HullInternal
{
public:
    class Point64
    {
    public:
        std::int64_t x;
        std::int64_t y;
        std::int64_t z;

        Point64(std::int64_t x, std::int64_t y, std::int64_t z) : x(x), y(y), z(z) {}

        bool isZero() const { return (x == 0) && (y == 0) && (z == 0); }

        std::int64_t dot(const Point64& b) const { return x * b.x + y * b.y + z * b.z; }
    };

    class Point32
    {
    public:
        std::int32_t x;
        std::int32_t y;
        std::int32_t z;
        int index;

        Point32() = default;

        Point32(std::int32_t x, std::int32_t y, std::int32_t z) : x(x), y(y), z(z), index(-1) {}

        bool operator==(const Point32& b) const { return (x == b.x) && (y == b.y) && (z == b.z); }

        bool operator!=(const Point32& b) const { return (x != b.x) || (y != b.y) || (z != b.z); }

        bool isZero() const { return (x == 0) && (y == 0) && (z == 0); }

        Point64 cross(const Point32& b) const
        {
            return Point64(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
        }

        Point64 cross(const Point64& b) const
        {
            return Point64(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
        }

        std::int64_t dot(const Point32& b) const { return x * b.x + y * b.y + z * b.z; }

        std::int64_t dot(const Point64& b) const { return x * b.x + y * b.y + z * b.z; }

        Point32 operator+(const Point32& b) const { return Point32(x + b.x, y + b.y, z + b.z); }

        Point32 operator-(const Point32& b) const { return Point32(x - b.x, y - b.y, z - b.z); }
    };

    class Int128
    {
    public:
        std::uint64_t low;
        std::uint64_t high;

        Int128() = default;

        Int128(std::uint64_t low, std::uint64_t high) : low(low), high(high) {}

        Int128(std::uint64_t low) : low(low), high(0) {}

        Int128(std::int64_t value) : low((std::uint64_t)value), high((value >= 0) ? 0 : (std::uint64_t)-1LL) {}

        static Int128 mul(std::int64_t a, std::int64_t b);

        static Int128 mul(std::uint64_t a, std::uint64_t b);

        Int128 operator-() const { return Int128((std::uint64_t) - (std::int64_t)low, ~high + (low == 0)); }

        Int128 operator+(const Int128& b) const
        {
            std::uint64_t lo = low + b.low;
            return Int128(lo, high + b.high + (lo < low));
        }

        Int128 operator-(const Int128& b) const { return *this + -b; }

        Int128& operator+=(const Int128& b)
        {
            std::uint64_t lo = low + b.low;
            if (lo < low)
                ++high;
            low = lo;
            high += b.high;
            return *this;
        }

        Int128& operator++()
        {
            if (++low == 0)
                ++high;
            return *this;
        }

        Int128 operator*(std::int64_t b) const;

        f32 toScalar() const
        {
            return ((std::int64_t)high >= 0) ? f32(high) * (f32(0x100000000LL) * f32(0x100000000LL)) + f32(low)
                                              : -(-*this).toScalar();
        }

        int getSign() const { return ((std::int64_t)high < 0) ? -1 : (high || low) ? 1 : 0; }

        bool operator<(const Int128& b) const { return (high < b.high) || ((high == b.high) && (low < b.low)); }

        int ucmp(const Int128& b) const
        {
            if (high < b.high)
                return -1;
            if (high > b.high)
                return 1;
            if (low < b.low)
                return -1;
            if (low > b.low)
                return 1;
            return 0;
        }
    };

    class Rational64
    {
    private:
        std::uint64_t m_numerator;
        std::uint64_t m_denominator;
        int sign;

    public:
        Rational64(std::int64_t numerator, std::int64_t denominator)
        {
            if (numerator > 0)
            {
                sign = 1;
                m_numerator = (std::uint64_t)numerator;
            }
            else if (numerator < 0)
            {
                sign = -1;
                m_numerator = (std::uint64_t)-numerator;
            }
            else
            {
                sign = 0;
                m_numerator = 0;
            }
            if (denominator > 0)
            {
                m_denominator = (std::uint64_t)denominator;
            }
            else if (denominator < 0)
            {
                sign = -sign;
                m_denominator = (std::uint64_t)-denominator;
            }
            else
            {
                m_denominator = 0;
            }
        }

        bool isNegativeInfinity() const { return (sign < 0) && (m_denominator == 0); }

        bool isNaN() const { return (sign == 0) && (m_denominator == 0); }

        int compare(const Rational64& b) const;

        f32 toScalar() const { return f32(sign) * ((m_denominator == 0) ? kInfinity : (f32)m_numerator / (f32)m_denominator); }
    };

    class Rational128
    {
    private:
        Int128 numerator;
        Int128 denominator;
        int sign;
        bool isInt64;

    public:
        Rational128(std::int64_t value)
        {
            if (value > 0)
            {
                sign = 1;
                this->numerator = value;
            }
            else if (value < 0)
            {
                sign = -1;
                this->numerator = -value;
            }
            else
            {
                sign = 0;
                this->numerator = (std::uint64_t)0;
            }
            this->denominator = (std::uint64_t)1;
            isInt64 = true;
        }

        Rational128(const Int128& numerator, const Int128& denominator)
        {
            sign = numerator.getSign();
            if (sign >= 0)
            {
                this->numerator = numerator;
            }
            else
            {
                this->numerator = -numerator;
            }
            int dsign = denominator.getSign();
            if (dsign >= 0)
            {
                this->denominator = denominator;
            }
            else
            {
                sign = -sign;
                this->denominator = -denominator;
            }
            isInt64 = false;
        }

        int compare(const Rational128& b) const;

        int compare(std::int64_t b) const;

        f32 toScalar() const { return f32(sign) * ((denominator.getSign() == 0) ? kInfinity : numerator.toScalar() / denominator.toScalar()); }
    };

    class PointR128
    {
    public:
        Int128 x;
        Int128 y;
        Int128 z;
        Int128 denominator;

        PointR128() = default;

        PointR128(Int128 x, Int128 y, Int128 z, Int128 denominator) : x(x), y(y), z(z), denominator(denominator) {}

        f32 xvalue() const { return x.toScalar() / denominator.toScalar(); }
        f32 yvalue() const { return y.toScalar() / denominator.toScalar(); }
        f32 zvalue() const { return z.toScalar() / denominator.toScalar(); }
    };

    class Edge;
    class Face;

    class Vertex
    {
    public:
        Vertex* next;
        Vertex* prev;
        Edge* edges;
        Face* firstNearbyFace;
        Face* lastNearbyFace;
        PointR128 point128;
        Point32 point;
        int copy;

        Vertex() : next(nullptr), prev(nullptr), edges(nullptr), firstNearbyFace(nullptr), lastNearbyFace(nullptr), copy(-1) {}

        Point32 operator-(const Vertex& b) const { return point - b.point; }

        Rational128 dot(const Point64& b) const
        {
            return (point.index >= 0) ? Rational128(point.dot(b))
                                       : Rational128(point128.x * b.x + point128.y * b.y + point128.z * b.z, point128.denominator);
        }

        f32 xvalue() const { return (point.index >= 0) ? f32(point.x) : point128.xvalue(); }
        f32 yvalue() const { return (point.index >= 0) ? f32(point.y) : point128.yvalue(); }
        f32 zvalue() const { return (point.index >= 0) ? f32(point.z) : point128.zvalue(); }

        void receiveNearbyFaces(Vertex* src)
        {
            if (lastNearbyFace)
            {
                lastNearbyFace->nextWithSameNearbyVertex = src->firstNearbyFace;
            }
            else
            {
                firstNearbyFace = src->firstNearbyFace;
            }
            if (src->lastNearbyFace)
            {
                lastNearbyFace = src->lastNearbyFace;
            }
            for (Face* f = src->firstNearbyFace; f; f = f->nextWithSameNearbyVertex)
            {
                assert(f->nearbyVertex == src);
                f->nearbyVertex = this;
            }
            src->firstNearbyFace = nullptr;
            src->lastNearbyFace = nullptr;
        }
    };

    class Edge
    {
    public:
        Edge* next;
        Edge* prev;
        Edge* reverse;
        Vertex* target;
        Face* face;
        int copy;

        ~Edge()
        {
            next = nullptr;
            prev = nullptr;
            reverse = nullptr;
            target = nullptr;
            face = nullptr;
        }

        void link(Edge* n)
        {
            assert(reverse->target == n->reverse->target);
            next = n;
            n->prev = this;
        }
    };

    class Face
    {
    public:
        Face* next;
        Vertex* nearbyVertex;
        Face* nextWithSameNearbyVertex;
        Point32 origin;
        Point32 dir0;
        Point32 dir1;

        Face() : next(nullptr), nearbyVertex(nullptr), nextWithSameNearbyVertex(nullptr) {}

        void init(Vertex* a, Vertex* b, Vertex* c)
        {
            nearbyVertex = a;
            origin = a->point;
            dir0 = *b - *a;
            dir1 = *c - *a;
            if (a->lastNearbyFace)
            {
                a->lastNearbyFace->nextWithSameNearbyVertex = this;
            }
            else
            {
                a->firstNearbyFace = this;
            }
            a->lastNearbyFace = this;
        }

        Point64 getNormal() { return dir0.cross(dir1); }
    };

    template <typename UWord, typename UHWord>
    class DMul
    {
    private:
        static std::uint32_t high(std::uint64_t value) { return (std::uint32_t)(value >> 32); }
        static std::uint32_t low(std::uint64_t value) { return (std::uint32_t)value; }
        static std::uint64_t mul(std::uint32_t a, std::uint32_t b) { return (std::uint64_t)a * (std::uint64_t)b; }
        static void shlHalf(std::uint64_t& value) { value <<= 32; }

        static std::uint64_t high(Int128 value) { return value.high; }
        static std::uint64_t low(Int128 value) { return value.low; }
        static Int128 mul(std::uint64_t a, std::uint64_t b) { return Int128::mul(a, b); }
        static void shlHalf(Int128& value)
        {
            value.high = value.low;
            value.low = 0;
        }

    public:
        static void mul(UWord a, UWord b, UWord& resLow, UWord& resHigh)
        {
            UWord p00 = mul(low(a), low(b));
            UWord p01 = mul(low(a), high(b));
            UWord p10 = mul(high(a), low(b));
            UWord p11 = mul(high(a), high(b));
            UWord p0110 = UWord(low(p01)) + UWord(low(p10));
            p11 += high(p01);
            p11 += high(p10);
            p11 += high(p0110);
            shlHalf(p0110);
            p00 += p0110;
            if (p00 < p0110)
            {
                ++p11;
            }
            resLow = p00;
            resHigh = p11;
        }
    };

private:
    class IntermediateHull
    {
    public:
        Vertex* minXy;
        Vertex* maxXy;
        Vertex* minYx;
        Vertex* maxYx;

        IntermediateHull() : minXy(nullptr), maxXy(nullptr), minYx(nullptr), maxYx(nullptr) {}
    };

    enum Orientation
    {
        NONE,
        CLOCKWISE,
        COUNTER_CLOCKWISE
    };

    template <typename T>
    class PoolArray
    {
    private:
        T* array;
        int size;

    public:
        PoolArray<T>* next;

        PoolArray(int size) : size(size), next(nullptr)
        {
            array = static_cast<T*>(::operator new(sizeof(T) * (size_t)size, std::align_val_t(16)));
        }

        ~PoolArray()
        {
            ::operator delete(array, std::align_val_t(16));
        }

        T* init()
        {
            T* o = array;
            for (int i = 0; i < size; i++, o++)
            {
                o->next = (i + 1 < size) ? o + 1 : nullptr;
            }
            return array;
        }
    };

    template <typename T>
    class Pool
    {
    private:
        PoolArray<T>* arrays;
        PoolArray<T>* nextArray;
        T* freeObjects;
        int arraySize;

    public:
        Pool() : arrays(nullptr), nextArray(nullptr), freeObjects(nullptr), arraySize(256) {}

        ~Pool()
        {
            while (arrays)
            {
                PoolArray<T>* p = arrays;
                arrays = p->next;
                p->~PoolArray<T>();
                ::operator delete(p, std::align_val_t(16));
            }
        }

        void reset()
        {
            nextArray = arrays;
            freeObjects = nullptr;
        }

        void setArraySize(int arraySize) { this->arraySize = arraySize; }

        T* newObject()
        {
            T* o = freeObjects;
            if (!o)
            {
                PoolArray<T>* p = nextArray;
                if (p)
                {
                    nextArray = p->next;
                }
                else
                {
                    p = new (::operator new(sizeof(PoolArray<T>), std::align_val_t(16))) PoolArray<T>(arraySize);
                    p->next = arrays;
                    arrays = p;
                }
                o = p->init();
            }
            freeObjects = o->next;
            return new (o) T();
        }

        void freeObject(T* object)
        {
            object->~T();
            object->next = freeObjects;
            freeObjects = object;
        }
    };

    Math::Vec3 scaling;
    Math::Vec3 center;
    Pool<Vertex> vertexPool;
    Pool<Edge> edgePool;
    Pool<Face> facePool;
    std::vector<Vertex*> originalVertices;
    int mergeStamp;
    int minAxis;
    int medAxis;
    int maxAxis;
    int usedEdgePairs;
    int maxUsedEdgePairs;

    static Orientation getOrientation(const Edge* prev, const Edge* next, const Point32& s, const Point32& t);
    Edge* findMaxAngle(bool ccw, const Vertex* start, const Point32& s, const Point64& rxs, const Point64& sxrxs, Rational64& minCot);
    void findEdgeForCoplanarFaces(Vertex* c0, Vertex* c1, Edge*& e0, Edge*& e1, Vertex* stop0, Vertex* stop1);

    Edge* newEdgePair(Vertex* from, Vertex* to);

    void removeEdgePair(Edge* edge)
    {
        Edge* n = edge->next;
        Edge* r = edge->reverse;

        assert(edge->target && r->target);

        if (n != edge)
        {
            n->prev = edge->prev;
            edge->prev->next = n;
            r->target->edges = n;
        }
        else
        {
            r->target->edges = nullptr;
        }

        n = r->next;

        if (n != r)
        {
            n->prev = r->prev;
            r->prev->next = n;
            edge->target->edges = n;
        }
        else
        {
            edge->target->edges = nullptr;
        }

        edgePool.freeObject(edge);
        edgePool.freeObject(r);
        usedEdgePairs--;
    }

    void computeInternal(int start, int end, IntermediateHull& result);

    bool mergeProjection(IntermediateHull& h0, IntermediateHull& h1, Vertex*& c0, Vertex*& c1);

    void merge(IntermediateHull& h0, IntermediateHull& h1);

    Math::Vec3 toVec3(const Point32& v);

    Math::Vec3 getNormal(Face* face);

    bool shiftFace(Face* face, f32 amount, std::vector<Vertex*> stack);

public:
    Vertex* vertexList;

    void compute(const void* coords, int stride, int count);

    Math::Vec3 getCoordinates(const Vertex* v);

    f32 shrink(f32 amount, f32 clampAmount);
};

HullInternal::Int128 HullInternal::Int128::operator*(std::int64_t b) const
{
    bool negative = (std::int64_t)high < 0;
    Int128 a = negative ? -*this : *this;
    if (b < 0)
    {
        negative = !negative;
        b = -b;
    }
    Int128 result = mul(a.low, (std::uint64_t)b);
    result.high += a.high * (std::uint64_t)b;
    return negative ? -result : result;
}

HullInternal::Int128 HullInternal::Int128::mul(std::int64_t a, std::int64_t b)
{
    Int128 result;
    bool negative = a < 0;
    if (negative)
    {
        a = -a;
    }
    if (b < 0)
    {
        negative = !negative;
        b = -b;
    }
    DMul<std::uint64_t, std::uint32_t>::mul((std::uint64_t)a, (std::uint64_t)b, result.low, result.high);
    return negative ? -result : result;
}

HullInternal::Int128 HullInternal::Int128::mul(std::uint64_t a, std::uint64_t b)
{
    Int128 result;
    DMul<std::uint64_t, std::uint32_t>::mul(a, b, result.low, result.high);
    return result;
}

int HullInternal::Rational64::compare(const Rational64& b) const
{
    if (sign != b.sign)
    {
        return sign - b.sign;
    }
    else if (sign == 0)
    {
        return 0;
    }

    return sign * Int128::mul(m_numerator, b.m_denominator).ucmp(Int128::mul(m_denominator, b.m_numerator));
}

int HullInternal::Rational128::compare(const Rational128& b) const
{
    if (sign != b.sign)
    {
        return sign - b.sign;
    }
    else if (sign == 0)
    {
        return 0;
    }
    if (isInt64)
    {
        return -b.compare(sign * (std::int64_t)numerator.low);
    }

    Int128 nbdLow, nbdHigh, dbnLow, dbnHigh;
    DMul<Int128, std::uint64_t>::mul(numerator, b.denominator, nbdLow, nbdHigh);
    DMul<Int128, std::uint64_t>::mul(denominator, b.numerator, dbnLow, dbnHigh);

    int cmp = nbdHigh.ucmp(dbnHigh);
    if (cmp)
    {
        return cmp * sign;
    }
    return nbdLow.ucmp(dbnLow) * sign;
}

int HullInternal::Rational128::compare(std::int64_t b) const
{
    if (isInt64)
    {
        std::int64_t a = sign * (std::int64_t)numerator.low;
        return (a > b) ? 1 : (a < b) ? -1 : 0;
    }
    if (b > 0)
    {
        if (sign <= 0)
        {
            return -1;
        }
    }
    else if (b < 0)
    {
        if (sign >= 0)
        {
            return 1;
        }
        b = -b;
    }
    else
    {
        return sign;
    }

    return numerator.ucmp(denominator * b) * sign;
}

HullInternal::Edge* HullInternal::newEdgePair(Vertex* from, Vertex* to)
{
    assert(from && to);
    Edge* e = edgePool.newObject();
    Edge* r = edgePool.newObject();
    e->reverse = r;
    r->reverse = e;
    e->copy = mergeStamp;
    r->copy = mergeStamp;
    e->target = to;
    r->target = from;
    e->face = nullptr;
    r->face = nullptr;
    usedEdgePairs++;
    if (usedEdgePairs > maxUsedEdgePairs)
    {
        maxUsedEdgePairs = usedEdgePairs;
    }
    return e;
}

bool HullInternal::mergeProjection(IntermediateHull& h0, IntermediateHull& h1, Vertex*& c0, Vertex*& c1)
{
    Vertex* v0 = h0.maxYx;
    Vertex* v1 = h1.minYx;
    if ((v0->point.x == v1->point.x) && (v0->point.y == v1->point.y))
    {
        assert(v0->point.z < v1->point.z);
        Vertex* v1p = v1->prev;
        if (v1p == v1)
        {
            c0 = v0;
            if (v1->edges)
            {
                assert(v1->edges->next == v1->edges);
                v1 = v1->edges->target;
                assert(v1->edges->next == v1->edges);
            }
            c1 = v1;
            return false;
        }
        Vertex* v1n = v1->next;
        v1p->next = v1n;
        v1n->prev = v1p;
        if (v1 == h1.minXy)
        {
            if ((v1n->point.x < v1p->point.x) || ((v1n->point.x == v1p->point.x) && (v1n->point.y < v1p->point.y)))
            {
                h1.minXy = v1n;
            }
            else
            {
                h1.minXy = v1p;
            }
        }
        if (v1 == h1.maxXy)
        {
            if ((v1n->point.x > v1p->point.x) || ((v1n->point.x == v1p->point.x) && (v1n->point.y > v1p->point.y)))
            {
                h1.maxXy = v1n;
            }
            else
            {
                h1.maxXy = v1p;
            }
        }
    }

    v0 = h0.maxXy;
    v1 = h1.maxXy;
    Vertex* v00 = nullptr;
    Vertex* v10 = nullptr;
    std::int32_t sign = 1;

    for (int side = 0; side <= 1; side++)
    {
        std::int32_t dx = (v1->point.x - v0->point.x) * sign;
        if (dx > 0)
        {
            while (true)
            {
                std::int32_t dy = v1->point.y - v0->point.y;

                Vertex* w0 = side ? v0->next : v0->prev;
                if (w0 != v0)
                {
                    std::int32_t dx0 = (w0->point.x - v0->point.x) * sign;
                    std::int32_t dy0 = w0->point.y - v0->point.y;
                    if ((dy0 <= 0) && ((dx0 == 0) || ((dx0 < 0) && (dy0 * dx <= dy * dx0))))
                    {
                        v0 = w0;
                        dx = (v1->point.x - v0->point.x) * sign;
                        continue;
                    }
                }

                Vertex* w1 = side ? v1->next : v1->prev;
                if (w1 != v1)
                {
                    std::int32_t dx1 = (w1->point.x - v1->point.x) * sign;
                    std::int32_t dy1 = w1->point.y - v1->point.y;
                    std::int32_t dxn = (w1->point.x - v0->point.x) * sign;
                    if ((dxn > 0) && (dy1 < 0) && ((dx1 == 0) || ((dx1 < 0) && (dy1 * dx < dy * dx1))))
                    {
                        v1 = w1;
                        dx = dxn;
                        continue;
                    }
                }

                break;
            }
        }
        else if (dx < 0)
        {
            while (true)
            {
                std::int32_t dy = v1->point.y - v0->point.y;

                Vertex* w1 = side ? v1->prev : v1->next;
                if (w1 != v1)
                {
                    std::int32_t dx1 = (w1->point.x - v1->point.x) * sign;
                    std::int32_t dy1 = w1->point.y - v1->point.y;
                    if ((dy1 >= 0) && ((dx1 == 0) || ((dx1 < 0) && (dy1 * dx <= dy * dx1))))
                    {
                        v1 = w1;
                        dx = (v1->point.x - v0->point.x) * sign;
                        continue;
                    }
                }

                Vertex* w0 = side ? v0->prev : v0->next;
                if (w0 != v0)
                {
                    std::int32_t dx0 = (w0->point.x - v0->point.x) * sign;
                    std::int32_t dy0 = w0->point.y - v0->point.y;
                    std::int32_t dxn = (v1->point.x - w0->point.x) * sign;
                    if ((dxn < 0) && (dy0 > 0) && ((dx0 == 0) || ((dx0 < 0) && (dy0 * dx < dy * dx0))))
                    {
                        v0 = w0;
                        dx = dxn;
                        continue;
                    }
                }

                break;
            }
        }
        else
        {
            std::int32_t x = v0->point.x;
            std::int32_t y0 = v0->point.y;
            Vertex* w0 = v0;
            Vertex* t;
            while (((t = side ? w0->next : w0->prev) != v0) && (t->point.x == x) && (t->point.y <= y0))
            {
                w0 = t;
                y0 = t->point.y;
            }
            v0 = w0;

            std::int32_t y1 = v1->point.y;
            Vertex* w1 = v1;
            while (((t = side ? w1->prev : w1->next) != v1) && (t->point.x == x) && (t->point.y >= y1))
            {
                w1 = t;
                y1 = t->point.y;
            }
            v1 = w1;
        }

        if (side == 0)
        {
            v00 = v0;
            v10 = v1;

            v0 = h0.minXy;
            v1 = h1.minXy;
            sign = -1;
        }
    }

    v0->prev = v1;
    v1->next = v0;

    v00->next = v10;
    v10->prev = v00;

    if (h1.minXy->point.x < h0.minXy->point.x)
    {
        h0.minXy = h1.minXy;
    }
    if (h1.maxXy->point.x >= h0.maxXy->point.x)
    {
        h0.maxXy = h1.maxXy;
    }

    h0.maxYx = h1.maxYx;

    c0 = v00;
    c1 = v10;

    return true;
}

void HullInternal::computeInternal(int start, int end, IntermediateHull& result)
{
    int n = end - start;
    switch (n)
    {
        case 0:
            result.minXy = nullptr;
            result.maxXy = nullptr;
            result.minYx = nullptr;
            result.maxYx = nullptr;
            return;
        case 2:
        {
            Vertex* v = originalVertices[start];
            Vertex* w = v + 1;
            if (v->point != w->point)
            {
                std::int32_t dx = v->point.x - w->point.x;
                std::int32_t dy = v->point.y - w->point.y;

                if ((dx == 0) && (dy == 0))
                {
                    if (v->point.z > w->point.z)
                    {
                        Vertex* t = w;
                        w = v;
                        v = t;
                    }
                    assert(v->point.z < w->point.z);
                    v->next = v;
                    v->prev = v;
                    result.minXy = v;
                    result.maxXy = v;
                    result.minYx = v;
                    result.maxYx = v;
                }
                else
                {
                    v->next = w;
                    v->prev = w;
                    w->next = v;
                    w->prev = v;

                    if ((dx < 0) || ((dx == 0) && (dy < 0)))
                    {
                        result.minXy = v;
                        result.maxXy = w;
                    }
                    else
                    {
                        result.minXy = w;
                        result.maxXy = v;
                    }

                    if ((dy < 0) || ((dy == 0) && (dx < 0)))
                    {
                        result.minYx = v;
                        result.maxYx = w;
                    }
                    else
                    {
                        result.minYx = w;
                        result.maxYx = v;
                    }
                }

                Edge* e = newEdgePair(v, w);
                e->link(e);
                v->edges = e;

                e = e->reverse;
                e->link(e);
                w->edges = e;

                return;
            }
            {
                Vertex* v2 = originalVertices[start];
                v2->edges = nullptr;
                v2->next = v2;
                v2->prev = v2;

                result.minXy = v2;
                result.maxXy = v2;
                result.minYx = v2;
                result.maxYx = v2;
            }

            return;
        }

        case 1:
        {
            Vertex* v = originalVertices[start];
            v->edges = nullptr;
            v->next = v;
            v->prev = v;

            result.minXy = v;
            result.maxXy = v;
            result.minYx = v;
            result.maxYx = v;

            return;
        }
    }

    int split0 = start + n / 2;
    Point32 p = originalVertices[split0 - 1]->point;
    int split1 = split0;
    while ((split1 < end) && (originalVertices[split1]->point == p))
    {
        split1++;
    }
    computeInternal(start, split0, result);
    IntermediateHull hull1;
    computeInternal(split1, end, hull1);
    merge(result, hull1);
}

HullInternal::Orientation HullInternal::getOrientation(const Edge* prev, const Edge* next, const Point32& s, const Point32& t)
{
    assert(prev->reverse->target == next->reverse->target);
    if (prev->next == next)
    {
        if (prev->prev == next)
        {
            Point64 n = t.cross(s);
            Point64 m = (*prev->target - *next->reverse->target).cross(*next->target - *next->reverse->target);
            assert(!m.isZero());
            std::int64_t dot = n.dot(m);
            assert(dot != 0);
            return (dot > 0) ? COUNTER_CLOCKWISE : CLOCKWISE;
        }
        return COUNTER_CLOCKWISE;
    }
    else if (prev->prev == next)
    {
        return CLOCKWISE;
    }
    else
    {
        return NONE;
    }
}

HullInternal::Edge* HullInternal::findMaxAngle(bool ccw, const Vertex* start, const Point32& s, const Point64& rxs, const Point64& sxrxs, Rational64& minCot)
{
    Edge* minEdge = nullptr;

    Edge* e = start->edges;
    if (e)
    {
        do
        {
            if (e->copy > mergeStamp)
            {
                Point32 t = *e->target - *start;
                Rational64 cot(t.dot(sxrxs), t.dot(rxs));
                if (cot.isNaN())
                {
                    assert(ccw ? (t.dot(s) < 0) : (t.dot(s) > 0));
                }
                else
                {
                    int cmp;
                    if (minEdge == nullptr)
                    {
                        minCot = cot;
                        minEdge = e;
                    }
                    else if ((cmp = cot.compare(minCot)) < 0)
                    {
                        minCot = cot;
                        minEdge = e;
                    }
                    else if ((cmp == 0) && (ccw == (getOrientation(minEdge, e, s, t) == COUNTER_CLOCKWISE)))
                    {
                        minEdge = e;
                    }
                }
            }
            e = e->next;
        } while (e != start->edges);
    }
    return minEdge;
}

void HullInternal::findEdgeForCoplanarFaces(Vertex* c0, Vertex* c1, Edge*& e0, Edge*& e1, Vertex* stop0, Vertex* stop1)
{
    Edge* start0 = e0;
    Edge* start1 = e1;
    Point32 et0 = start0 ? start0->target->point : c0->point;
    Point32 et1 = start1 ? start1->target->point : c1->point;
    Point32 s = c1->point - c0->point;
    Point64 normal = ((start0 ? start0 : start1)->target->point - c0->point).cross(s);
    std::int64_t dist = c0->point.dot(normal);
    assert(!start1 || (start1->target->point.dot(normal) == dist));
    Point64 perp = s.cross(normal);
    assert(!perp.isZero());

    std::int64_t maxDot0 = et0.dot(perp);
    if (e0)
    {
        while (e0->target != stop0)
        {
            Edge* e = e0->reverse->prev;
            if (e->target->point.dot(normal) < dist)
            {
                break;
            }
            assert(e->target->point.dot(normal) == dist);
            if (e->copy == mergeStamp)
            {
                break;
            }
            std::int64_t dot = e->target->point.dot(perp);
            if (dot <= maxDot0)
            {
                break;
            }
            maxDot0 = dot;
            e0 = e;
            et0 = e->target->point;
        }
    }

    std::int64_t maxDot1 = et1.dot(perp);
    if (e1)
    {
        while (e1->target != stop1)
        {
            Edge* e = e1->reverse->next;
            if (e->target->point.dot(normal) < dist)
            {
                break;
            }
            assert(e->target->point.dot(normal) == dist);
            if (e->copy == mergeStamp)
            {
                break;
            }
            std::int64_t dot = e->target->point.dot(perp);
            if (dot <= maxDot1)
            {
                break;
            }
            maxDot1 = dot;
            e1 = e;
            et1 = e->target->point;
        }
    }

    std::int64_t dx = maxDot1 - maxDot0;
    if (dx > 0)
    {
        while (true)
        {
            std::int64_t dy = (et1 - et0).dot(s);

            if (e0 && (e0->target != stop0))
            {
                Edge* f0 = e0->next->reverse;
                if (f0->copy > mergeStamp)
                {
                    std::int64_t dx0 = (f0->target->point - et0).dot(perp);
                    std::int64_t dy0 = (f0->target->point - et0).dot(s);
                    if ((dx0 == 0) ? (dy0 < 0) : ((dx0 < 0) && (Rational64(dy0, dx0).compare(Rational64(dy, dx)) >= 0)))
                    {
                        et0 = f0->target->point;
                        dx = (et1 - et0).dot(perp);
                        e0 = (e0 == start0) ? nullptr : f0;
                        continue;
                    }
                }
            }

            if (e1 && (e1->target != stop1))
            {
                Edge* f1 = e1->reverse->next;
                if (f1->copy > mergeStamp)
                {
                    Point32 d1 = f1->target->point - et1;
                    if (d1.dot(normal) == 0)
                    {
                        std::int64_t dx1 = d1.dot(perp);
                        std::int64_t dy1 = d1.dot(s);
                        std::int64_t dxn = (f1->target->point - et0).dot(perp);
                        if ((dxn > 0) && ((dx1 == 0) ? (dy1 < 0) : ((dx1 < 0) && (Rational64(dy1, dx1).compare(Rational64(dy, dx)) > 0))))
                        {
                            e1 = f1;
                            et1 = e1->target->point;
                            dx = dxn;
                            continue;
                        }
                    }
                    else
                    {
                        assert((e1 == start1) && (d1.dot(normal) < 0));
                    }
                }
            }

            break;
        }
    }
    else if (dx < 0)
    {
        while (true)
        {
            std::int64_t dy = (et1 - et0).dot(s);

            if (e1 && (e1->target != stop1))
            {
                Edge* f1 = e1->prev->reverse;
                if (f1->copy > mergeStamp)
                {
                    std::int64_t dx1 = (f1->target->point - et1).dot(perp);
                    std::int64_t dy1 = (f1->target->point - et1).dot(s);
                    if ((dx1 == 0) ? (dy1 > 0) : ((dx1 < 0) && (Rational64(dy1, dx1).compare(Rational64(dy, dx)) <= 0)))
                    {
                        et1 = f1->target->point;
                        dx = (et1 - et0).dot(perp);
                        e1 = (e1 == start1) ? nullptr : f1;
                        continue;
                    }
                }
            }

            if (e0 && (e0->target != stop0))
            {
                Edge* f0 = e0->reverse->prev;
                if (f0->copy > mergeStamp)
                {
                    Point32 d0 = f0->target->point - et0;
                    if (d0.dot(normal) == 0)
                    {
                        std::int64_t dx0 = d0.dot(perp);
                        std::int64_t dy0 = d0.dot(s);
                        std::int64_t dxn = (et1 - f0->target->point).dot(perp);
                        if ((dxn < 0) && ((dx0 == 0) ? (dy0 > 0) : ((dx0 < 0) && (Rational64(dy0, dx0).compare(Rational64(dy, dx)) < 0))))
                        {
                            e0 = f0;
                            et0 = e0->target->point;
                            dx = dxn;
                            continue;
                        }
                    }
                    else
                    {
                        assert((e0 == start0) && (d0.dot(normal) < 0));
                    }
                }
            }

            break;
        }
    }
}

void HullInternal::merge(IntermediateHull& h0, IntermediateHull& h1)
{
    if (!h1.maxXy)
    {
        return;
    }
    if (!h0.maxXy)
    {
        h0 = h1;
        return;
    }

    mergeStamp--;

    Vertex* c0 = nullptr;
    Edge* toPrev0 = nullptr;
    Edge* firstNew0 = nullptr;
    Edge* pendingHead0 = nullptr;
    Edge* pendingTail0 = nullptr;
    Vertex* c1 = nullptr;
    Edge* toPrev1 = nullptr;
    Edge* firstNew1 = nullptr;
    Edge* pendingHead1 = nullptr;
    Edge* pendingTail1 = nullptr;
    Point32 prevPoint;

    if (mergeProjection(h0, h1, c0, c1))
    {
        Point32 s = *c1 - *c0;
        Point64 normal = Point32(0, 0, -1).cross(s);
        Point64 t = s.cross(normal);
        assert(!t.isZero());

        Edge* e = c0->edges;
        Edge* start0 = nullptr;
        if (e)
        {
            do
            {
                std::int64_t dot = (*e->target - *c0).dot(normal);
                assert(dot <= 0);
                if ((dot == 0) && ((*e->target - *c0).dot(t) > 0))
                {
                    if (!start0 || (getOrientation(start0, e, s, Point32(0, 0, -1)) == CLOCKWISE))
                    {
                        start0 = e;
                    }
                }
                e = e->next;
            } while (e != c0->edges);
        }

        e = c1->edges;
        Edge* start1 = nullptr;
        if (e)
        {
            do
            {
                std::int64_t dot = (*e->target - *c1).dot(normal);
                assert(dot <= 0);
                if ((dot == 0) && ((*e->target - *c1).dot(t) > 0))
                {
                    if (!start1 || (getOrientation(start1, e, s, Point32(0, 0, -1)) == COUNTER_CLOCKWISE))
                    {
                        start1 = e;
                    }
                }
                e = e->next;
            } while (e != c1->edges);
        }

        if (start0 || start1)
        {
            findEdgeForCoplanarFaces(c0, c1, start0, start1, nullptr, nullptr);
            if (start0)
            {
                c0 = start0->target;
            }
            if (start1)
            {
                c1 = start1->target;
            }
        }

        prevPoint = c1->point;
        prevPoint.z++;
    }
    else
    {
        prevPoint = c1->point;
        prevPoint.x++;
    }

    Vertex* first0 = c0;
    Vertex* first1 = c1;
    bool firstRun = true;

    while (true)
    {
        Point32 s = *c1 - *c0;
        Point32 r = prevPoint - c0->point;
        Point64 rxs = r.cross(s);
        Point64 sxrxs = s.cross(rxs);

        Rational64 minCot0(0, 0);
        Edge* min0 = findMaxAngle(false, c0, s, rxs, sxrxs, minCot0);
        Rational64 minCot1(0, 0);
        Edge* min1 = findMaxAngle(true, c1, s, rxs, sxrxs, minCot1);
        if (!min0 && !min1)
        {
            Edge* e = newEdgePair(c0, c1);
            e->link(e);
            c0->edges = e;

            e = e->reverse;
            e->link(e);
            c1->edges = e;
            return;
        }
        else
        {
            int cmp = !min0 ? 1 : !min1 ? -1 : minCot0.compare(minCot1);
            if (firstRun || ((cmp >= 0) ? !minCot1.isNegativeInfinity() : !minCot0.isNegativeInfinity()))
            {
                Edge* e = newEdgePair(c0, c1);
                if (pendingTail0)
                {
                    pendingTail0->prev = e;
                }
                else
                {
                    pendingHead0 = e;
                }
                e->next = pendingTail0;
                pendingTail0 = e;

                e = e->reverse;
                if (pendingTail1)
                {
                    pendingTail1->next = e;
                }
                else
                {
                    pendingHead1 = e;
                }
                e->prev = pendingTail1;
                pendingTail1 = e;
            }

            Edge* e0 = min0;
            Edge* e1 = min1;

            if (cmp == 0)
            {
                findEdgeForCoplanarFaces(c0, c1, e0, e1, nullptr, nullptr);
            }

            if ((cmp >= 0) && e1)
            {
                if (toPrev1)
                {
                    for (Edge *e = toPrev1->next, *n = nullptr; e != min1; e = n)
                    {
                        n = e->next;
                        removeEdgePair(e);
                    }
                }

                if (pendingTail1)
                {
                    if (toPrev1)
                    {
                        toPrev1->link(pendingHead1);
                    }
                    else
                    {
                        min1->prev->link(pendingHead1);
                        firstNew1 = pendingHead1;
                    }
                    pendingTail1->link(min1);
                    pendingHead1 = nullptr;
                    pendingTail1 = nullptr;
                }
                else if (!toPrev1)
                {
                    firstNew1 = min1;
                }

                prevPoint = c1->point;
                c1 = e1->target;
                toPrev1 = e1->reverse;
            }

            if ((cmp <= 0) && e0)
            {
                if (toPrev0)
                {
                    for (Edge *e = toPrev0->prev, *n = nullptr; e != min0; e = n)
                    {
                        n = e->prev;
                        removeEdgePair(e);
                    }
                }

                if (pendingTail0)
                {
                    if (toPrev0)
                    {
                        pendingHead0->link(toPrev0);
                    }
                    else
                    {
                        pendingHead0->link(min0->next);
                        firstNew0 = pendingHead0;
                    }
                    min0->link(pendingTail0);
                    pendingHead0 = nullptr;
                    pendingTail0 = nullptr;
                }
                else if (!toPrev0)
                {
                    firstNew0 = min0;
                }

                prevPoint = c0->point;
                c0 = e0->target;
                toPrev0 = e0->reverse;
            }
        }

        if ((c0 == first0) && (c1 == first1))
        {
            if (toPrev0 == nullptr)
            {
                pendingHead0->link(pendingTail0);
                c0->edges = pendingTail0;
            }
            else
            {
                for (Edge *e = toPrev0->prev, *n = nullptr; e != firstNew0; e = n)
                {
                    n = e->prev;
                    removeEdgePair(e);
                }
                if (pendingTail0)
                {
                    pendingHead0->link(toPrev0);
                    firstNew0->link(pendingTail0);
                }
            }

            if (toPrev1 == nullptr)
            {
                pendingTail1->link(pendingHead1);
                c1->edges = pendingTail1;
            }
            else
            {
                for (Edge *e = toPrev1->next, *n = nullptr; e != firstNew1; e = n)
                {
                    n = e->next;
                    removeEdgePair(e);
                }
                if (pendingTail1)
                {
                    toPrev1->link(pendingHead1);
                    pendingTail1->link(firstNew1);
                }
            }

            return;
        }

        firstRun = false;
    }
}

class PointCompare
{
public:
    bool operator()(const HullInternal::Point32& p, const HullInternal::Point32& q) const
    {
        return (p.y < q.y) || ((p.y == q.y) && ((p.x < q.x) || ((p.x == q.x) && (p.z < q.z))));
    }
};

void HullInternal::compute(const void* coords, int stride, int count)
{
    Math::Vec3 minPoint(1e30f, 1e30f, 1e30f);
    Math::Vec3 maxPoint(-1e30f, -1e30f, -1e30f);
    const char* ptr = (const char*)coords;
    for (int i = 0; i < count; i++)
    {
        const float* v = (const float*)ptr;
        Math::Vec3 p(v[0], v[1], v[2]);
        ptr += stride;
        minPoint = vecMin(minPoint, p);
        maxPoint = vecMax(maxPoint, p);
    }

    Math::Vec3 s = maxPoint - minPoint;
    maxAxis = vecMaxAxis(s);
    minAxis = vecMinAxis(s);
    if (minAxis == maxAxis)
    {
        minAxis = (maxAxis + 1) % 3;
    }
    medAxis = 3 - maxAxis - minAxis;

    s /= f32(10216);
    if (((medAxis + 1) % 3) != maxAxis)
    {
        s *= -1.0f;
    }
    scaling = s;

    if (s[0] != 0)
    {
        s[0] = 1.0f / s[0];
    }
    if (s[1] != 0)
    {
        s[1] = 1.0f / s[1];
    }
    if (s[2] != 0)
    {
        s[2] = 1.0f / s[2];
    }

    center = (minPoint + maxPoint) * 0.5f;

    std::vector<Point32> points;
    points.resize(count);
    ptr = (const char*)coords;
    for (int i = 0; i < count; i++)
    {
        const float* v = (const float*)ptr;
        Math::Vec3 p(v[0], v[1], v[2]);
        ptr += stride;
        p = (p - center) * s;
        points[i].x = (std::int32_t)p[medAxis];
        points[i].y = (std::int32_t)p[maxAxis];
        points[i].z = (std::int32_t)p[minAxis];
        points[i].index = i;
    }
    std::sort(points.begin(), points.end(), PointCompare());

    vertexPool.reset();
    vertexPool.setArraySize(count);
    originalVertices.resize(count);
    for (int i = 0; i < count; i++)
    {
        Vertex* v = vertexPool.newObject();
        v->edges = nullptr;
        v->point = points[i];
        v->copy = -1;
        originalVertices[i] = v;
    }

    points.clear();

    edgePool.reset();
    edgePool.setArraySize(6 * count);

    usedEdgePairs = 0;
    maxUsedEdgePairs = 0;

    mergeStamp = -3;

    IntermediateHull hull;
    computeInternal(0, count, hull);
    vertexList = hull.minXy;
}

Math::Vec3 HullInternal::toVec3(const Point32& v)
{
    Math::Vec3 p;
    p[medAxis] = f32(v.x);
    p[maxAxis] = f32(v.y);
    p[minAxis] = f32(v.z);
    return p * scaling;
}

Math::Vec3 HullInternal::getNormal(Face* face)
{
    return glm::normalize(glm::cross(toVec3(face->dir0), toVec3(face->dir1)));
}

Math::Vec3 HullInternal::getCoordinates(const Vertex* v)
{
    Math::Vec3 p;
    p[medAxis] = v->xvalue();
    p[maxAxis] = v->yvalue();
    p[minAxis] = v->zvalue();
    return p * scaling + center;
}

f32 HullInternal::shrink(f32 amount, f32 clampAmount)
{
    if (!vertexList)
    {
        return 0;
    }
    int stamp = --mergeStamp;
    std::vector<Vertex*> stack;
    vertexList->copy = stamp;
    stack.push_back(vertexList);
    std::vector<Face*> faces;

    Point32 ref = vertexList->point;
    Int128 hullCenterX(0, 0);
    Int128 hullCenterY(0, 0);
    Int128 hullCenterZ(0, 0);
    Int128 volume(0, 0);

    while (stack.size() > 0)
    {
        Vertex* v = stack[stack.size() - 1];
        stack.pop_back();
        Edge* e = v->edges;
        if (e)
        {
            do
            {
                if (e->target->copy != stamp)
                {
                    e->target->copy = stamp;
                    stack.push_back(e->target);
                }
                if (e->copy != stamp)
                {
                    Face* face = facePool.newObject();
                    face->init(e->target, e->reverse->prev->target, v);
                    faces.push_back(face);
                    Edge* f = e;

                    Vertex* a = nullptr;
                    Vertex* b = nullptr;
                    do
                    {
                        if (a && b)
                        {
                            std::int64_t vol = (v->point - ref).dot((a->point - ref).cross(b->point - ref));
                            assert(vol >= 0);
                            Point32 c = v->point + a->point + b->point + ref;
                            hullCenterX += vol * c.x;
                            hullCenterY += vol * c.y;
                            hullCenterZ += vol * c.z;
                            volume += vol;
                        }

                        assert(f->copy != stamp);
                        f->copy = stamp;
                        f->face = face;

                        a = b;
                        b = f->target;

                        f = f->reverse->prev;
                    } while (f != e);
                }
                e = e->next;
            } while (e != v->edges);
        }
    }

    if (volume.getSign() <= 0)
    {
        return 0;
    }

    Math::Vec3 hullCenter;
    hullCenter[medAxis] = hullCenterX.toScalar();
    hullCenter[maxAxis] = hullCenterY.toScalar();
    hullCenter[minAxis] = hullCenterZ.toScalar();
    hullCenter /= 4.0f * volume.toScalar();
    hullCenter *= scaling;

    int faceCount = (int)faces.size();

    if (clampAmount > 0)
    {
        f32 minDist = kInfinity;
        for (int i = 0; i < faceCount; i++)
        {
            Math::Vec3 normal = getNormal(faces[i]);
            f32 dist = glm::dot(normal, toVec3(faces[i]->origin) - hullCenter);
            if (dist < minDist)
            {
                minDist = dist;
            }
        }

        if (minDist <= 0)
        {
            return 0;
        }

        amount = std::min(amount, minDist * clampAmount);
    }

    unsigned int seed = 243703;
    for (int i = 0; i < faceCount; i++, seed = 1664525 * seed + 1013904223)
    {
        std::swap(faces[i], faces[seed % faceCount]);
    }

    for (int i = 0; i < faceCount; i++)
    {
        if (!shiftFace(faces[i], amount, stack))
        {
            return -amount;
        }
    }

    return amount;
}

bool HullInternal::shiftFace(Face* face, f32 amount, std::vector<Vertex*> stack)
{
    Math::Vec3 origShift = getNormal(face) * -amount;
    if (scaling[0] != 0)
    {
        origShift[0] /= scaling[0];
    }
    if (scaling[1] != 0)
    {
        origShift[1] /= scaling[1];
    }
    if (scaling[2] != 0)
    {
        origShift[2] /= scaling[2];
    }
    Point32 shift((std::int32_t)origShift[medAxis], (std::int32_t)origShift[maxAxis], (std::int32_t)origShift[minAxis]);
    if (shift.isZero())
    {
        return true;
    }
    Point64 normal = face->getNormal();
    std::int64_t origDot = face->origin.dot(normal);
    Point32 shiftedOrigin = face->origin + shift;
    std::int64_t shiftedDot = shiftedOrigin.dot(normal);
    assert(shiftedDot <= origDot);
    if (shiftedDot >= origDot)
    {
        return false;
    }

    Edge* intersection = nullptr;

    Edge* startEdge = face->nearbyVertex->edges;
    Rational128 optDot = face->nearbyVertex->dot(normal);
    int cmp = optDot.compare(shiftedDot);
    if (cmp >= 0)
    {
        Edge* e = startEdge;
        do
        {
            Rational128 dot = e->target->dot(normal);
            assert(dot.compare(origDot) <= 0);
            if (dot.compare(optDot) < 0)
            {
                int c = dot.compare(shiftedDot);
                optDot = dot;
                e = e->reverse;
                startEdge = e;
                if (c < 0)
                {
                    intersection = e;
                    break;
                }
                cmp = c;
            }
            e = e->prev;
        } while (e != startEdge);

        if (!intersection)
        {
            return false;
        }
    }
    else
    {
        Edge* e = startEdge;
        do
        {
            Rational128 dot = e->target->dot(normal);
            assert(dot.compare(origDot) <= 0);
            if (dot.compare(optDot) > 0)
            {
                cmp = dot.compare(shiftedDot);
                if (cmp >= 0)
                {
                    intersection = e;
                    break;
                }
                optDot = dot;
                e = e->reverse;
                startEdge = e;
            }
            e = e->prev;
        } while (e != startEdge);

        if (!intersection)
        {
            return true;
        }
    }

    if (cmp == 0)
    {
        Edge* e = intersection->reverse->next;
        while (e->target->dot(normal).compare(shiftedDot) <= 0)
        {
            e = e->next;
            if (e == intersection->reverse)
            {
                return true;
            }
        }
    }

    Edge* firstIntersection = nullptr;
    Edge* faceEdge = nullptr;
    Edge* firstFaceEdge = nullptr;

    while (true)
    {
        if (cmp == 0)
        {
            Edge* e = intersection->reverse->next;
            startEdge = e;
            while (true)
            {
                if (e->target->dot(normal).compare(shiftedDot) >= 0)
                {
                    break;
                }
                intersection = e->reverse;
                e = e->next;
                if (e == startEdge)
                {
                    return true;
                }
            }
        }

        if (!firstIntersection)
        {
            firstIntersection = intersection;
        }
        else if (intersection == firstIntersection)
        {
            break;
        }

        int prevCmp = cmp;
        Edge* prevIntersection = intersection;
        Edge* prevFaceEdge = faceEdge;

        Edge* e = intersection->reverse;
        while (true)
        {
            e = e->reverse->prev;
            assert(e != intersection->reverse);
            cmp = e->target->dot(normal).compare(shiftedDot);
            if (cmp >= 0)
            {
                intersection = e;
                break;
            }
        }

        if (cmp > 0)
        {
            Vertex* removed = intersection->target;
            e = intersection->reverse;
            if (e->prev == e)
            {
                removed->edges = nullptr;
            }
            else
            {
                removed->edges = e->prev;
                e->prev->link(e->next);
                e->link(e);
            }

            Point64 n0 = intersection->face->getNormal();
            Point64 n1 = intersection->reverse->face->getNormal();
            std::int64_t m00 = face->dir0.dot(n0);
            std::int64_t m01 = face->dir1.dot(n0);
            std::int64_t m10 = face->dir0.dot(n1);
            std::int64_t m11 = face->dir1.dot(n1);
            std::int64_t r0 = (intersection->face->origin - shiftedOrigin).dot(n0);
            std::int64_t r1 = (intersection->reverse->face->origin - shiftedOrigin).dot(n1);
            Int128 det = Int128::mul(m00, m11) - Int128::mul(m01, m10);
            assert(det.getSign() != 0);
            Vertex* v = vertexPool.newObject();
            v->point.index = -1;
            v->copy = -1;
            v->point128 = PointR128(Int128::mul(face->dir0.x * r0, m11) - Int128::mul(face->dir0.x * r1, m01) + Int128::mul(face->dir1.x * r1, m00) - Int128::mul(face->dir1.x * r0, m10) + det * shiftedOrigin.x,
                                     Int128::mul(face->dir0.y * r0, m11) - Int128::mul(face->dir0.y * r1, m01) + Int128::mul(face->dir1.y * r1, m00) - Int128::mul(face->dir1.y * r0, m10) + det * shiftedOrigin.y,
                                     Int128::mul(face->dir0.z * r0, m11) - Int128::mul(face->dir0.z * r1, m01) + Int128::mul(face->dir1.z * r1, m00) - Int128::mul(face->dir1.z * r0, m10) + det * shiftedOrigin.z,
                                     det);
            v->point.x = (std::int32_t)v->point128.xvalue();
            v->point.y = (std::int32_t)v->point128.yvalue();
            v->point.z = (std::int32_t)v->point128.zvalue();
            intersection->target = v;
            v->edges = e;

            stack.push_back(v);
            stack.push_back(removed);
            stack.push_back(nullptr);
        }

        if (cmp || prevCmp || (prevIntersection->reverse->next->target != intersection->target))
        {
            faceEdge = newEdgePair(prevIntersection->target, intersection->target);
            if (prevCmp == 0)
            {
                faceEdge->link(prevIntersection->reverse->next);
            }
            if ((prevCmp == 0) || prevFaceEdge)
            {
                prevIntersection->reverse->link(faceEdge);
            }
            if (cmp == 0)
            {
                intersection->reverse->prev->link(faceEdge->reverse);
            }
            faceEdge->reverse->link(intersection->reverse);
        }
        else
        {
            faceEdge = prevIntersection->reverse->next;
        }

        if (prevFaceEdge)
        {
            if (prevCmp > 0)
            {
                faceEdge->link(prevFaceEdge->reverse);
            }
            else if (faceEdge != prevFaceEdge->reverse)
            {
                stack.push_back(prevFaceEdge->target);
                while (faceEdge->next != prevFaceEdge->reverse)
                {
                    Vertex* removed = faceEdge->next->target;
                    removeEdgePair(faceEdge->next);
                    stack.push_back(removed);
                }
                stack.push_back(nullptr);
            }
        }
        faceEdge->face = face;
        faceEdge->reverse->face = intersection->face;

        if (!firstFaceEdge)
        {
            firstFaceEdge = faceEdge;
        }
    }

    if (cmp > 0)
    {
        firstFaceEdge->reverse->target = faceEdge->target;
        firstIntersection->reverse->link(firstFaceEdge);
        firstFaceEdge->link(faceEdge->reverse);
    }
    else if (firstFaceEdge != faceEdge->reverse)
    {
        stack.push_back(faceEdge->target);
        while (firstFaceEdge->next != faceEdge->reverse)
        {
            Vertex* removed = firstFaceEdge->next->target;
            removeEdgePair(firstFaceEdge->next);
            stack.push_back(removed);
        }
        stack.push_back(nullptr);
    }

    assert(stack.size() > 0);
    vertexList = stack[0];

    int pos = 0;
    while (pos < (int)stack.size())
    {
        int end = (int)stack.size();
        while (pos < end)
        {
            Vertex* kept = stack[pos++];
            bool deeper = false;
            Vertex* removed;
            while ((removed = stack[pos++]) != nullptr)
            {
                kept->receiveNearbyFaces(removed);
                while (removed->edges)
                {
                    if (!deeper)
                    {
                        deeper = true;
                        stack.push_back(kept);
                    }
                    stack.push_back(removed->edges->target);
                    removeEdgePair(removed->edges);
                }
            }
            if (deeper)
            {
                stack.push_back(nullptr);
            }
        }
    }

    stack.resize(0);
    face->origin = shiftedOrigin;

    return true;
}

int getVertexCopy(HullInternal::Vertex* vertex, std::vector<HullInternal::Vertex*>& vertices)
{
    int index = vertex->copy;
    if (index < 0)
    {
        index = (int)vertices.size();
        vertex->copy = index;
        vertices.push_back(vertex);
    }
    return index;
}

} // namespace

int ConvexHullComputer::Edge::getSourceVertex() const
{
    return (this + reverse)->targetVertex;
}

int ConvexHullComputer::Edge::getTargetVertex() const
{
    return targetVertex;
}

const ConvexHullComputer::Edge* ConvexHullComputer::Edge::getNextEdgeOfVertex() const
{
    return this + next;
}

const ConvexHullComputer::Edge* ConvexHullComputer::Edge::getNextEdgeOfFace() const
{
    return (this + reverse)->getNextEdgeOfVertex();
}

const ConvexHullComputer::Edge* ConvexHullComputer::Edge::getReverseEdge() const
{
    return this + reverse;
}

f32 ConvexHullComputer::compute(const float* coords, int stride, int count, f32 shrink, f32 shrinkClamp)
{
    if (count <= 0)
    {
        vertices.clear();
        edges.clear();
        faces.clear();
        return 0;
    }

    HullInternal hull;
    hull.compute(coords, stride, count);

    f32 shift = 0;
    if ((shrink > 0) && ((shift = hull.shrink(shrink, shrinkClamp)) < 0))
    {
        vertices.clear();
        edges.clear();
        faces.clear();
        return shift;
    }

    vertices.resize(0);
    edges.resize(0);
    faces.resize(0);

    std::vector<HullInternal::Vertex*> oldVertices;
    getVertexCopy(hull.vertexList, oldVertices);
    int copied = 0;
    while (copied < (int)oldVertices.size())
    {
        HullInternal::Vertex* v = oldVertices[copied];
        vertices.push_back(hull.getCoordinates(v));
        HullInternal::Edge* firstEdge = v->edges;
        if (firstEdge)
        {
            int firstCopy = -1;
            int prevCopy = -1;
            HullInternal::Edge* e = firstEdge;
            do
            {
                if (e->copy < 0)
                {
                    int s = (int)edges.size();
                    edges.push_back(Edge());
                    edges.push_back(Edge());
                    Edge* c = &edges[s];
                    Edge* r = &edges[s + 1];
                    e->copy = s;
                    e->reverse->copy = s + 1;
                    c->reverse = 1;
                    r->reverse = -1;
                    c->targetVertex = getVertexCopy(e->target, oldVertices);
                    r->targetVertex = copied;
                }
                if (prevCopy >= 0)
                {
                    edges[e->copy].next = prevCopy - e->copy;
                }
                else
                {
                    firstCopy = e->copy;
                }
                prevCopy = e->copy;
                e = e->next;
            } while (e != firstEdge);
            edges[firstCopy].next = prevCopy - firstCopy;
        }
        copied++;
    }

    for (int i = 0; i < copied; i++)
    {
        HullInternal::Vertex* v = oldVertices[i];
        HullInternal::Edge* firstEdge = v->edges;
        if (firstEdge)
        {
            HullInternal::Edge* e = firstEdge;
            do
            {
                if (e->copy >= 0)
                {
                    faces.push_back(e->copy);
                    HullInternal::Edge* f = e;
                    do
                    {
                        f->copy = -1;
                        f = f->reverse->prev;
                    } while (f != e);
                }
                e = e->next;
            } while (e != firstEdge);
        }
    }

    return shift;
}

} // namespace Radion::Geometry
