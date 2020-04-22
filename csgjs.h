#ifndef VXMATHCSGJS_H
#define VXMATHCSGJS_H

// Original CSG.JS library by Evan Wallace (http://madebyevan.com), under the MIT license.
// GitHub: https://github.com/evanw/csg.js/
//
// C++ port by Tomasz Dabrowski (http://28byteslater.com), under the MIT license.
// GitHub: https://github.com/dabroz/csgjscpp-cpp/
//
// Constructive Solid Geometry (CSG) is a modeling technique that uses Boolean
// operations like union and intersection to combine 3D solids. This library
// implements CSG operations on meshes elegantly and concisely using BSP trees,
// and is meant to serve as an easily understandable implementation of the
// algorithm. All edge cases involving overlapping coplanar polygons in both
// solids are correctly handled.
//
// modified by dazza - 200421

#include <vector>

#if !defined(CSGJSCPP_REAL)
#define CSGJSCPP_REAL float
#endif

namespace csgjscpp {

// `CSG.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
// point is on the plane.
const CSGJSCPP_REAL csgjs_EPSILON = 0.0001f;

struct Vector {
    CSGJSCPP_REAL x, y, z;

    Vector() : x(0.0f), y(0.0f), z(0.0f) {
    }
    Vector(CSGJSCPP_REAL x, CSGJSCPP_REAL y, CSGJSCPP_REAL z) : x(x), y(y), z(z) {
    }
};

// Vector implementation

inline Vector operator+(const Vector &a, const Vector &b) {
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z);
}
Vector operator-(const Vector &a, const Vector &b) {
    return Vector(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline Vector operator*(const Vector &a, CSGJSCPP_REAL b) {
    return Vector(a.x * b, a.y * b, a.z * b);
}
inline Vector operator/(const Vector &a, CSGJSCPP_REAL b) {
    return a * ((CSGJSCPP_REAL)1.0 / b);
}
inline CSGJSCPP_REAL dot(const Vector &a, const Vector &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vector lerp(const Vector &a, const Vector &b, CSGJSCPP_REAL v) {
    return a + (b - a) * v;
}
inline Vector negate(const Vector &a) {
    return a * -(CSGJSCPP_REAL)1.0;
}
inline CSGJSCPP_REAL length(const Vector &a) {
    return (CSGJSCPP_REAL)sqrt(dot(a, a));
}
inline Vector unit(const Vector &a) {
    return a / length(a);
}
inline Vector cross(const Vector &a, const Vector &b) {
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Vector operator-(const Vector &a) {
    return Vector(-a.x, -a.y, -a.z);
}

struct Vertex {
    Vector pos;
    Vector normal;
    Vector col;
};

struct Polygon;

// Represents a plane in 3D space.
struct Plane {
    Vector  normal;
    CSGJSCPP_REAL w;

    Plane();
    Plane(const Vector &a, const Vector &b, const Vector &c);
    bool ok() const;
    void flip();
    void splitpolygon(const Polygon &poly, std::vector<Polygon> &coplanarFront,
                      std::vector<Polygon> &coplanarBack, std::vector<Polygon> &front,
                      std::vector<Polygon> &back) const;

    enum Classification { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };
    inline Classification classify(const Vector &p) const {
        CSGJSCPP_REAL       t = dot(normal, p) - this->w;
        Classification c = (t < -csgjs_EPSILON) ? BACK : ((t > csgjs_EPSILON) ? FRONT : COPLANAR);
        return c;
    }
};

// Represents a convex polygon. The vertices used to initialize a polygon must
// be coplanar and form a convex loop. They do not have to be `CSG.Vertex`
// instances but they must behave similarly (duck typing can be used for
// customization).
//
// Each convex polygon has a `shared` property, which is shared between all
// polygons that are clones of each other or were split from the same polygon.
// This can be used to define per-polygon properties (such as surface color).
struct Polygon {
    std::vector<Vertex> vertices;
    Plane               plane;
    void                      flip();

    Polygon();
    Polygon(const std::vector<Vertex> &list);
};

struct Model {
    std::vector<Vertex> vertices;
    std::vector<uint16_t>     indices;
};

// public interface - not super efficient, if you use multiple CSG operations you should
// use BSP trees and convert them into model only once. Another optimization trick is
// replacing model with your own class.

Model csgunion(const Model &a, const Model &b);
Model csgintersection(const Model &a, const Model &b);
Model csgsubtract(const Model &a, const Model &b);

std::vector<Polygon> csgunion(const std::vector<Polygon> &a, const std::vector<Polygon> &b);
std::vector<Polygon> csgintersection(const std::vector<Polygon> &a, const std::vector<Polygon> &b);
std::vector<Polygon> csgsubtract(const std::vector<Polygon> &a, const std::vector<Polygon> &b);

Model modelfrompolygons(const std::vector<Polygon> &polygons);

Model csgsmodel_cube(const Vector &center = {0.0f, 0.0f, 0.0f},
                           const Vector &dim = {1.0f, 1.0f, 1.0f}, const Vector &col = {1.0f, 1.0f, 1.0f});
Model csgsmodel_sphere(const Vector &center = {0.0f, 0.0f, 0.0f}, CSGJSCPP_REAL radius = 1.0f,
                             const Vector &col = {1.0f, 1.0f, 1.0f}, int slices = 16, int stacks = 8);

Model csgsmodel_cylinder(const Vector &s = {0.0f, -1.0f, 0.0f}, const Vector &e = {0.0f, 1.0f, 0.0f},
                               CSGJSCPP_REAL radius = 1.0f, const Vector &col = {1.0f, 1.0f, 1.0f},
                               int slices = 16);

} // namespace csgjscpp

#if defined(CSGJSCPP_IMPLEMENTATION)

/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/* implementation below here */

#include <deque>
#include <algorithm>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace csgjscpp {

// Holds a node in a BSP tree. A BSP tree is built from a collection of polygons
// by picking a polygon to split along. That polygon (and all other coplanar
// polygons) are added directly to that node and the other polygons are added to
// the front and/or back subtrees. This is not a leafy BSP tree since there is
// no distinction between internal and leaf nodes.
struct CSGNode {
    std::vector<Polygon> polygons;
    CSGNode *            front;
    CSGNode *            back;
    Plane                plane;

    CSGNode();
    CSGNode(const std::vector<Polygon> &list);
    ~CSGNode();

    CSGNode *            clone() const;
    void                       clipto(const CSGNode *other);
    void                       invert();
    void                       build(const std::vector<Polygon> &Polygon);
    std::vector<Polygon> clippolygons(const std::vector<Polygon> &list) const;
    std::vector<Polygon> allpolygons() const;
};

// Vertex implementation

// Invert all orientation-specific data (e.g. Vertex normal). Called when the
// orientation of a polygon is flipped.
inline Vertex flip(Vertex v) {
    v.normal = negate(v.normal);
    return v;
}

// Create a new Vertex between this Vertex and `other` by linearly
// interpolating all properties using a parameter of `t`. Subclasses should
// override this to interpolate additional properties.
inline Vertex interpolate(const Vertex &a, const Vertex &b, CSGJSCPP_REAL t) {
    Vertex ret;
    ret.pos = lerp(a.pos, b.pos, t);
    ret.normal = lerp(a.normal, b.normal, t);
    ret.col = lerp(a.col, b.col, t);
    return ret;
}

// Plane implementation

Plane::Plane() : normal(), w(0.0f) {
}

bool Plane::ok() const {
    return length(this->normal) > 0.0f;
}

void Plane::flip() {
    this->normal = negate(this->normal);
    this->w *= -1.0f;
}

Plane::Plane(const Vector &a, const Vector &b, const Vector &c) {
    this->normal = unit(cross(b - a, c - a));
    this->w = dot(this->normal, a);
}

// Split `polygon` by this plane if needed, then put the polygon or polygon
// fragments in the appropriate lists. Coplanar polygons go into either
// `coplanarFront` or `coplanarBack` depending on their orientation with
// respect to this plane. Polygons in front or in back of this plane go into
// either `front` or `back`.
void Plane::splitpolygon(const Polygon &poly, std::vector<Polygon> &coplanarFront,
                               std::vector<Polygon> &coplanarBack, std::vector<Polygon> &front,
                               std::vector<Polygon> &back) const {

    // Classify each point as well as the entire polygon into one of the above
    // four classes.
    int polygonType = 0;
    for (const auto &v : poly.vertices) {
        polygonType |= classify(v.pos);
    }

    // Put the polygon in the correct list, splitting it when necessary.
    switch (polygonType) {
    case COPLANAR: {
        if (dot(this->normal, poly.plane.normal) > 0)
            coplanarFront.push_back(poly);
        else
            coplanarBack.push_back(poly);
        break;
    }
    case FRONT: {
        front.push_back(poly);
        break;
    }
    case BACK: {
        back.push_back(poly);
        break;
    }
    case SPANNING: {
        std::vector<Vertex> f, b;

        for (size_t i = 0; i < poly.vertices.size(); i++) {

            size_t j = (i + 1) % poly.vertices.size();

            const Vertex &vi = poly.vertices[i];
            const Vertex &vj = poly.vertices[j];

            int ti = classify(vi.pos);
            int tj = classify(vj.pos);

            if (ti != BACK)
                f.push_back(vi);
            if (ti != FRONT)
                b.push_back(vi);
            if ((ti | tj) == SPANNING) {
                CSGJSCPP_REAL t = (this->w - dot(this->normal, vi.pos)) / dot(this->normal, vj.pos - vi.pos);
                Vertex  v = interpolate(vi, vj, t);
                f.push_back(v);
                b.push_back(v);
            }
        }
        if (f.size() >= 3)
            front.push_back(Polygon(f));
        if (b.size() >= 3)
            back.push_back(Polygon(b));
        break;
    }
    }
}

// Polygon implementation

void Polygon::flip() {
    std::reverse(vertices.begin(), vertices.end());
    for (size_t i = 0; i < vertices.size(); i++)
        vertices[i].normal = negate(vertices[i].normal);
    plane.flip();
}

Polygon::Polygon() {
}

Polygon::Polygon(const std::vector<Vertex> &list)
    : vertices(list), plane(vertices[0].pos, vertices[1].pos, vertices[2].pos) {
}

// Node implementation

// Return a new CSG solid representing space in either this solid or in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline CSGNode *csg_union(const CSGNode *a1, const CSGNode *b1) {
    CSGNode *a = a1->clone();
    CSGNode *b = b1->clone();
    a->clipto(b);
    b->clipto(a);
    b->invert();
    b->clipto(a);
    b->invert();
    a->build(b->allpolygons());
    CSGNode *ret = new CSGNode(a->allpolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Return a new CSG solid representing space in this solid but not in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline CSGNode *csg_subtract(const CSGNode *a1, const CSGNode *b1) {
    CSGNode *a = a1->clone();
    CSGNode *b = b1->clone();
    a->invert();
    a->clipto(b);
    b->clipto(a);
    b->invert();
    b->clipto(a);
    b->invert();
    a->build(b->allpolygons());
    a->invert();
    CSGNode *ret = new CSGNode(a->allpolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Return a new CSG solid representing space both this solid and in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline CSGNode *csg_intersect(const CSGNode *a1, const CSGNode *b1) {
    CSGNode *a = a1->clone();
    CSGNode *b = b1->clone();
    a->invert();
    b->clipto(a);
    b->invert();
    a->clipto(b);
    b->clipto(a);
    a->build(b->allpolygons());
    a->invert();
    CSGNode *ret = new CSGNode(a->allpolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Convert solid space to empty space and empty space to solid space.
void CSGNode::invert() {
    std::deque<CSGNode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        CSGNode *me = nodes.front();
        nodes.pop_front();

        for (size_t i = 0; i < me->polygons.size(); i++)
            me->polygons[i].flip();
        me->plane.flip();
        std::swap(me->front, me->back);
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }
}

// Recursively remove all polygons in `polygons` that are inside this BSP
// tree.
std::vector<Polygon> CSGNode::clippolygons(const std::vector<Polygon> &ilist) const {
    std::vector<Polygon> result;

    std::deque<std::pair<const CSGNode *const, std::vector<Polygon>>> clips;
    clips.push_back(std::make_pair(this, ilist));
    while (clips.size()) {
        const CSGNode *             me = clips.front().first;
        const std::vector<Polygon> &list = clips.front().second;

        if (!me->plane.ok()) {
            result.insert(result.end(), list.begin(), list.end());
            clips.pop_front();
            continue;
        }

        std::vector<Polygon> list_front, list_back;
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitpolygon(list[i], list_front, list_back, list_front, list_back);

        if (me->front)
            clips.push_back(std::make_pair(me->front, list_front));
        else
            result.insert(result.end(), list_front.begin(), list_front.end());

        if (me->back)
            clips.push_back(std::make_pair(me->back, list_back));

        clips.pop_front();
    }

    return result;
}

// Remove all polygons in this BSP tree that are inside the other BSP tree
// `bsp`.
void CSGNode::clipto(const CSGNode *other) {
    std::deque<CSGNode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        CSGNode *me = nodes.front();
        nodes.pop_front();

        me->polygons = other->clippolygons(me->polygons);
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }
}

// Return a list of all polygons in this BSP tree.
std::vector<Polygon> CSGNode::allpolygons() const {
    std::vector<Polygon> result;

    std::deque<const CSGNode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        const CSGNode *me = nodes.front();
        nodes.pop_front();

        result.insert(result.end(), me->polygons.begin(), me->polygons.end());
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }

    return result;
}

CSGNode *CSGNode::clone() const {
    CSGNode *ret = new CSGNode();

    std::deque<std::pair<const CSGNode *, CSGNode *>> nodes;
    nodes.push_back(std::make_pair(this, ret));
    while (nodes.size()) {
        const CSGNode *original = nodes.front().first;
        CSGNode *      clone = nodes.front().second;
        nodes.pop_front();

        clone->polygons = original->polygons;
        clone->plane = original->plane;
        if (original->front) {
            clone->front = new CSGNode();
            nodes.push_back(std::make_pair(original->front, clone->front));
        }
        if (original->back) {
            clone->back = new CSGNode();
            nodes.push_back(std::make_pair(original->back, clone->back));
        }
    }

    return ret;
}

// Build a BSP tree out of `polygons`. When called on an existing tree, the
// new polygons are filtered down to the bottom of the tree and become new
// nodes there. Each set of polygons is partitioned using the first polygon
// (no heuristic is used to pick a good split).
void CSGNode::build(const std::vector<Polygon> &ilist) {
    if (!ilist.size())
        return;

    std::deque<std::pair<CSGNode *, std::vector<Polygon>>> builds;
    builds.push_back(std::make_pair(this, ilist));

    while (builds.size()) {
        CSGNode *                   me = builds.front().first;
        const std::vector<Polygon> &list = builds.front().second;

        assert(list.size() > 0 && "logic error");

        if (!me->plane.ok())
            me->plane = list[0].plane;
        std::vector<Polygon> list_front, list_back;

        // me->polygons.push_back(list[0]);
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitpolygon(list[i], me->polygons, me->polygons, list_front, list_back);

        if (list_front.size()) {
            if (!me->front)
                me->front = new CSGNode;
            builds.push_back(std::make_pair(me->front, list_front));
        }
        if (list_back.size()) {
            if (!me->back)
                me->back = new CSGNode;
            builds.push_back(std::make_pair(me->back, list_back));
        }

        builds.pop_front();
    }
}

CSGNode::CSGNode() : front(0), back(0) {
}

CSGNode::CSGNode(const std::vector<Polygon> &list) : front(0), back(0) {
    build(list);
}

CSGNode::~CSGNode() {

    std::deque<CSGNode *> nodes_to_delete;
    std::deque<CSGNode *> nodes_to_disassemble;

    nodes_to_disassemble.push_back(this);
    while (nodes_to_disassemble.size()) {
        CSGNode *me = nodes_to_disassemble.front();
        nodes_to_disassemble.pop_front();

        if (me->front) {
            nodes_to_disassemble.push_back(me->front);
            nodes_to_delete.push_back(me->front);
            me->front = NULL;
        }
        if (me->back) {
            nodes_to_disassemble.push_back(me->back);
            nodes_to_delete.push_back(me->back);
            me->back = NULL;
        }
    }

    for (auto it = nodes_to_delete.begin(); it != nodes_to_delete.end(); ++it)
        delete *it;
}

// Public interface implementation

inline std::vector<Polygon> csgjs_modelToPolygons(const Model &model) {
    std::vector<Polygon> list;
    for (size_t i = 0; i < model.indices.size(); i += 3) {
        std::vector<Vertex> triangle;
        for (int j = 0; j < 3; j++) {
            Vertex v = model.vertices[model.indices[i + j]];
            triangle.push_back(v);
        }
        list.push_back(Polygon(triangle));
    }
    return list;
}

Model modelfrompolygons(const std::vector<Polygon> &polygons) {
    Model model;
    uint16_t    p = 0;
    for (size_t i = 0; i < polygons.size(); i++) {
        const Polygon &poly = polygons[i];
        for (size_t j = 2; j < poly.vertices.size(); j++) {
            model.vertices.push_back(poly.vertices[0]);
            model.indices.push_back(p++);
            model.vertices.push_back(poly.vertices[j - 1]);
            model.indices.push_back(p++);
            model.vertices.push_back(poly.vertices[j]);
            model.indices.push_back(p++);
        }
    }
    return model;
}

typedef CSGNode *csg_function(const CSGNode *a1, const CSGNode *b1);

std::vector<Polygon> csgjs_operation(const std::vector<Polygon> &apoly,
                                           const std::vector<Polygon> &bpoly, csg_function fun) {

    CSGNode A(apoly);
    CSGNode B(bpoly);

    CSGNode *AB = fun(&A, &B);
    return AB->allpolygons();
}

inline std::vector<Polygon> csgjs_operation(const Model &a, const Model &b, csg_function fun) {
    return csgjs_operation(csgjs_modelToPolygons(a), csgjs_modelToPolygons(b), fun);
}

Model csgsmodel_cube(const Vector &center, const Vector &dim, const Vector &col) {

    struct Quad {
        int          indices[4];
        Vector normal;
    } quads[] = {{{0, 4, 6, 2}, {-1, 0, 0}}, {{1, 3, 7, 5}, {+1, 0, 0}}, {{0, 1, 5, 4}, {0, -1, 0}},
                 {{2, 6, 7, 3}, {0, +1, 0}}, {{0, 2, 3, 1}, {0, 0, -1}}, {{4, 5, 7, 6}, {0, 0, +1}}};

    std::vector<Polygon> polygons;
    for (const auto &q : quads) {

        std::vector<Vertex> verts;

        for (auto i : q.indices) {
            Vector pos(center.x + dim.x * (2.0f * !!(i & 1) - 1), center.y + dim.y * (2.0f * !!(i & 2) - 1),
                             center.z + dim.z * (2.0f * !!(i & 4) - 1));

            verts.push_back({pos, q.normal, col});
        }
        polygons.push_back(Polygon(verts));
    }

    return modelfrompolygons(polygons);
}

Model csgsmodel_sphere(const Vector &c, CSGJSCPP_REAL r, const Vector &col, int slices, int stacks) {

    std::vector<Polygon> polygons;

    auto mkvertex = [c, r, col](CSGJSCPP_REAL theta, CSGJSCPP_REAL phi) -> Vertex {
        theta *= (CSGJSCPP_REAL)M_PI * 2;
        phi *= (CSGJSCPP_REAL)M_PI;
        Vector dir((CSGJSCPP_REAL)cos(theta) * (CSGJSCPP_REAL)sin(phi), (CSGJSCPP_REAL)cos(phi),
                         (CSGJSCPP_REAL)sin(theta) * (CSGJSCPP_REAL)sin(phi));

        return Vertex{c + (dir * r), dir, col};
    };
    for (CSGJSCPP_REAL i = 0; i < slices; i++) {
        for (CSGJSCPP_REAL j = 0; j < stacks; j++) {

            std::vector<Vertex> vertices;

            vertices.push_back(mkvertex(i / slices, j / stacks));
            if (j > 0) {
                vertices.push_back(mkvertex((i + 1) / slices, j / stacks));
            }
            if (j < stacks - 1) {
                vertices.push_back(mkvertex((i + 1) / slices, (j + 1) / stacks));
            }
            vertices.push_back(mkvertex(i / slices, (j + 1) / stacks));
            polygons.push_back(Polygon(vertices));
        }
    }
    return modelfrompolygons(polygons);
}

Model csgsmodel_cylinder(const Vector &s, const Vector &e, CSGJSCPP_REAL r, const Vector &col,
                               int slices) {

    Vector ray = e - s;

    Vector axisZ = unit(ray);
    bool         isY = fabs(axisZ.y) > 0.5f;
    Vector axisX = unit(cross(Vector(isY, !isY, 0), axisZ));
    Vector axisY = unit(cross(axisX, axisZ));

    Vertex start{s, -axisZ, col};
    Vertex end{e, unit(axisZ), col};

    std::vector<Polygon> polygons;

    auto point = [axisX, axisY, s, r, ray, axisZ, col](CSGJSCPP_REAL stack, CSGJSCPP_REAL slice,
                                                       CSGJSCPP_REAL normalBlend) -> Vertex {
        CSGJSCPP_REAL angle = slice * (CSGJSCPP_REAL)M_PI * 2;
        Vector  out = axisX * (CSGJSCPP_REAL)cos(angle) + axisY * (CSGJSCPP_REAL)sin(angle);
        Vector  pos = s + ray * stack + out * r;
        Vector  normal = out * (1.0f - fabs(normalBlend)) + axisZ * normalBlend;
        return Vertex{pos, normal, col};
    };

    for (CSGJSCPP_REAL i = 0; i < slices; i++) {
        CSGJSCPP_REAL t0 = i / slices;
        CSGJSCPP_REAL t1 = (i + 1) / slices;
        polygons.push_back(Polygon({start, point(0, t0, -1), point(0, t1, -1)}));
        polygons.push_back(Polygon({point(0, t1, 0), point(0, t0, 0), point(1, t0, 0), point(1, t1, 0)}));
        polygons.push_back(Polygon({end, point(1, t1, 1), point(1, t0, 1)}));
    }
    return modelfrompolygons(polygons);
};

Model csgunion(const Model &a, const Model &b) {
    return modelfrompolygons(csgjs_operation(a, b, csg_union));
}

Model csgintersection(const Model &a, const Model &b) {
    return modelfrompolygons(csgjs_operation(a, b, csg_intersect));
}

Model csgsubtract(const Model &a, const Model &b) {
    return modelfrompolygons(csgjs_operation(a, b, csg_subtract));
}

std::vector<Polygon> csgunion(const std::vector<Polygon> &a, const std::vector<Polygon> &b) {
    return csgjs_operation(a, b, csg_union);
}

std::vector<Polygon> csgintersection(const std::vector<Polygon> &a,
                                              const std::vector<Polygon> &b) {
    return csgjs_operation(a, b, csg_intersect);
}

std::vector<Polygon> csgsubtract(const std::vector<Polygon> &a, const std::vector<Polygon> &b) {
    return csgjs_operation(a, b, csg_subtract);
}

} // namespace csgjscpp

#endif

#endif