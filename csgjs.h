#ifndef VXMATHCSGJS_H
#define VXMATHCSGJS_H

// Original CSG.JS library by Evan Wallace (http://madebyevan.com), under the MIT license.
// GitHub: https://github.com/evanw/csg.js/
//
// C++ port by Tomasz Dabrowski (http://28byteslater.com), under the MIT license.
// GitHub: https://github.com/dabroz/csgjs-cpp/
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

struct csgjs_vector {
    float x, y, z;

    csgjs_vector() : x(0.0f), y(0.0f), z(0.0f) {
    }
    csgjs_vector(float x, float y, float z) : x(x), y(y), z(z) {
    }
};

struct csgjs_vector2 {
    float x, y;

    csgjs_vector2() : x(0.0f), y(0.0f) {
    }

    csgjs_vector2(float x, float y) : x(x), y(y) {
    }
};

struct csgjs_vertex {
    csgjs_vector  pos;
    csgjs_vector  normal;
    csgjs_vector2 uv;
};

struct csgjs_polygon;

// Represents a plane in 3D space.
struct csgjs_plane {
    csgjs_vector normal;
    float        w;

    csgjs_plane();
    csgjs_plane(const csgjs_vector &a, const csgjs_vector &b, const csgjs_vector &c);
    bool ok() const;
    void flip();
    void splitPolygon(const csgjs_polygon &polygon, std::vector<csgjs_polygon> &coplanarFront,
                      std::vector<csgjs_polygon> &coplanarBack, std::vector<csgjs_polygon> &front,
                      std::vector<csgjs_polygon> &back) const;
};

// Represents a convex polygon. The vertices used to initialize a polygon must
// be coplanar and form a convex loop. They do not have to be `CSG.Vertex`
// instances but they must behave similarly (duck typing can be used for
// customization).
//
// Each convex polygon has a `shared` property, which is shared between all
// polygons that are clones of each other or were split from the same polygon.
// This can be used to define per-polygon properties (such as surface color).
struct csgjs_polygon {
    std::vector<csgjs_vertex> vertices;
    csgjs_plane               plane;
    void                      flip();

    csgjs_polygon();
    csgjs_polygon(const std::vector<csgjs_vertex> &list);
};

struct csgjs_model {
    std::vector<csgjs_vertex> vertices;
    std::vector<uint16_t>     indices;
};

// public interface - not super efficient, if you use multiple CSG operations you should
// use BSP trees and convert them into model only once. Another optimization trick is
// replacing csgjs_model with your own class.

csgjs_model csgjs_union(const csgjs_model &a, const csgjs_model &b);
csgjs_model csgjs_intersection(const csgjs_model &a, const csgjs_model &b);
csgjs_model csgjs_subtract(const csgjs_model &a, const csgjs_model &b);

std::vector<csgjs_polygon> csgjs_union(const std::vector<csgjs_polygon> &a, const std::vector<csgjs_polygon> &b);
std::vector<csgjs_polygon> csgjs_intersection(const std::vector<csgjs_polygon> &a, const std::vector<csgjs_polygon> &b);
std::vector<csgjs_polygon> csgjs_subtract(const std::vector<csgjs_polygon> &a, const std::vector<csgjs_polygon> &b);

csgjs_model csgjs_modelFromPolygons(const std::vector<csgjs_polygon> &polygons);

csgjs_model csgsmodel_cube(const csgjs_vector &center = {0.0f, 0.0f, 0.0f},
                           const csgjs_vector &dim = {1.0f, 1.0f, 1.0f});
csgjs_model csgsmodel_sphere(const csgjs_vector &center = {0.0f, 0.0f, 0.0f}, float radius = 1.0f, int slices = 16,
                             int stacks = 8);

csgjs_model csgsmodel_cyliner(const csgjs_vector &s = {0.0f, -1.0f, 0.0f}, const csgjs_vector &e = {0.0f, 1.0f, 0.0f},
                              float radius = 1.0f, int slices = 16);

#if defined(CSGJS_IMPLEMENTATION)

/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
/* implementation below here */

#include <list>
#include <algorithm>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>

// `CSG.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
// point is on the plane.
static const float csgjs_EPSILON = 0.0001f;

// Holds a node in a BSP tree. A BSP tree is built from a collection of polygons
// by picking a polygon to split along. That polygon (and all other coplanar
// polygons) are added directly to that node and the other polygons are added to
// the front and/or back subtrees. This is not a leafy BSP tree since there is
// no distinction between internal and leaf nodes.
struct csgjs_csgnode {
    std::vector<csgjs_polygon> polygons;
    csgjs_csgnode *            front;
    csgjs_csgnode *            back;
    csgjs_plane                plane;

    csgjs_csgnode();
    csgjs_csgnode(const std::vector<csgjs_polygon> &list);
    ~csgjs_csgnode();

    csgjs_csgnode *            clone() const;
    void                       clipTo(const csgjs_csgnode *other);
    void                       invert();
    void                       build(const std::vector<csgjs_polygon> &polygon);
    std::vector<csgjs_polygon> clipPolygons(const std::vector<csgjs_polygon> &list) const;
    std::vector<csgjs_polygon> allPolygons() const;
};

// Vector implementation

inline csgjs_vector operator+(const csgjs_vector &a, const csgjs_vector &b) {
    return csgjs_vector(a.x + b.x, a.y + b.y, a.z + b.z);
}
csgjs_vector operator-(const csgjs_vector &a, const csgjs_vector &b) {
    return csgjs_vector(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline csgjs_vector operator*(const csgjs_vector &a, float b) {
    return csgjs_vector(a.x * b, a.y * b, a.z * b);
}
inline csgjs_vector operator/(const csgjs_vector &a, float b) {
    return a * (1.0f / b);
}
inline float dot(const csgjs_vector &a, const csgjs_vector &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline csgjs_vector lerp(const csgjs_vector &a, const csgjs_vector &b, float v) {
    return a + (b - a) * v;
}
inline csgjs_vector2 lerp(const csgjs_vector2 &a, const csgjs_vector2 &b, float v) {
    return csgjs_vector2{a.x + (b.x - a.x) * v, a.y + (b.y - a.y) * v};
}
inline csgjs_vector negate(const csgjs_vector &a) {
    return a * -1.0f;
}
inline float length(const csgjs_vector &a) {
    return sqrtf(dot(a, a));
}
inline csgjs_vector unit(const csgjs_vector &a) {
    return a / length(a);
}
inline csgjs_vector cross(const csgjs_vector &a, const csgjs_vector &b) {
    return csgjs_vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline csgjs_vector operator-(const csgjs_vector &a) {
	return csgjs_vector(-a.x, -a.y, -a.z);
}

// Vertex implementation

// Invert all orientation-specific data (e.g. vertex normal). Called when the
// orientation of a polygon is flipped.
inline csgjs_vertex flip(csgjs_vertex v) {
    v.normal = negate(v.normal);
    return v;
}

// Create a new vertex between this vertex and `other` by linearly
// interpolating all properties using a parameter of `t`. Subclasses should
// override this to interpolate additional properties.
inline csgjs_vertex interpolate(const csgjs_vertex &a, const csgjs_vertex &b, float t) {
    csgjs_vertex ret;
    ret.pos = lerp(a.pos, b.pos, t);
    ret.normal = lerp(a.normal, b.normal, t);
    ret.uv = lerp(a.uv, b.uv, t);
    return ret;
}

// Plane implementation

csgjs_plane::csgjs_plane() : normal(), w(0.0f) {
}

bool csgjs_plane::ok() const {
    return length(this->normal) > 0.0f;
}

void csgjs_plane::flip() {
    this->normal = negate(this->normal);
    this->w *= -1.0f;
}

csgjs_plane::csgjs_plane(const csgjs_vector &a, const csgjs_vector &b, const csgjs_vector &c) {
    this->normal = unit(cross(b - a, c - a));
    this->w = dot(this->normal, a);
}

// Split `polygon` by this plane if needed, then put the polygon or polygon
// fragments in the appropriate lists. Coplanar polygons go into either
// `coplanarFront` or `coplanarBack` depending on their orientation with
// respect to this plane. Polygons in front or in back of this plane go into
// either `front` or `back`.
void csgjs_plane::splitPolygon(const csgjs_polygon &polygon, std::vector<csgjs_polygon> &coplanarFront,
                               std::vector<csgjs_polygon> &coplanarBack, std::vector<csgjs_polygon> &front,
                               std::vector<csgjs_polygon> &back) const {
    enum { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };

    // Classify each point as well as the entire polygon into one of the above
    // four classes.
    int              polygonType = 0;
    std::vector<int> types;
	types.reserve(polygon.vertices.size());
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
        float t = dot(this->normal, polygon.vertices[i].pos) - this->w;
        int   type = (t < -csgjs_EPSILON) ? BACK : ((t > csgjs_EPSILON) ? FRONT : COPLANAR);
        polygonType |= type;
        types.push_back(type);
    }

    // Put the polygon in the correct list, splitting it when necessary.
    switch (polygonType) {
    case COPLANAR: {
        if (dot(this->normal, polygon.plane.normal) > 0)
            coplanarFront.push_back(polygon);
        else
            coplanarBack.push_back(polygon);
        break;
    }
    case FRONT: {
        front.push_back(polygon);
        break;
    }
    case BACK: {
        back.push_back(polygon);
        break;
    }
    case SPANNING: {
        std::vector<csgjs_vertex> f, b;

        for (size_t i = 0; i < polygon.vertices.size(); i++) {
            size_t       j = (i + 1) % polygon.vertices.size();
            int          ti = types[i], tj = types[j];
            csgjs_vertex vi = polygon.vertices[i], vj = polygon.vertices[j];
            if (ti != BACK)
                f.push_back(vi);
            if (ti != FRONT)
                b.push_back(vi);
            if ((ti | tj) == SPANNING) {
                float        t = (this->w - dot(this->normal, vi.pos)) / dot(this->normal, vj.pos - vi.pos);
                csgjs_vertex v = interpolate(vi, vj, t);
                f.push_back(v);
                b.push_back(v);
            }
        }
        if (f.size() >= 3)
            front.push_back(csgjs_polygon(f));
        if (b.size() >= 3)
            back.push_back(csgjs_polygon(b));
        break;
    }
    }
}

// Polygon implementation

void csgjs_polygon::flip() {
    std::reverse(vertices.begin(), vertices.end());
    for (size_t i = 0; i < vertices.size(); i++)
        vertices[i].normal = negate(vertices[i].normal);
    plane.flip();
}

csgjs_polygon::csgjs_polygon() {
}

csgjs_polygon::csgjs_polygon(const std::vector<csgjs_vertex> &list)
    : vertices(list), plane(vertices[0].pos, vertices[1].pos, vertices[2].pos) {
}

// Node implementation

// Return a new CSG solid representing space in either this solid or in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline csgjs_csgnode *csg_union(const csgjs_csgnode *a1, const csgjs_csgnode *b1) {
    csgjs_csgnode *a = a1->clone();
    csgjs_csgnode *b = b1->clone();
    a->clipTo(b);
    b->clipTo(a);
    b->invert();
    b->clipTo(a);
    b->invert();
    a->build(b->allPolygons());
    csgjs_csgnode *ret = new csgjs_csgnode(a->allPolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Return a new CSG solid representing space in this solid but not in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline csgjs_csgnode *csg_subtract(const csgjs_csgnode *a1, const csgjs_csgnode *b1) {
    csgjs_csgnode *a = a1->clone();
    csgjs_csgnode *b = b1->clone();
    a->invert();
    a->clipTo(b);
    b->clipTo(a);
    b->invert();
    b->clipTo(a);
    b->invert();
    a->build(b->allPolygons());
    a->invert();
    csgjs_csgnode *ret = new csgjs_csgnode(a->allPolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Return a new CSG solid representing space both this solid and in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline csgjs_csgnode *csg_intersect(const csgjs_csgnode *a1, const csgjs_csgnode *b1) {
    csgjs_csgnode *a = a1->clone();
    csgjs_csgnode *b = b1->clone();
    a->invert();
    b->clipTo(a);
    b->invert();
    a->clipTo(b);
    b->clipTo(a);
    a->build(b->allPolygons());
    a->invert();
    csgjs_csgnode *ret = new csgjs_csgnode(a->allPolygons());
    delete a;
    a = 0;
    delete b;
    b = 0;
    return ret;
}

// Convert solid space to empty space and empty space to solid space.
void csgjs_csgnode::invert() {
    std::list<csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        csgjs_csgnode *me = nodes.front();
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
std::vector<csgjs_polygon> csgjs_csgnode::clipPolygons(const std::vector<csgjs_polygon> &ilist) const {
    std::vector<csgjs_polygon> result;

    std::list<std::pair<const csgjs_csgnode *const, std::vector<csgjs_polygon>>> clips;
    clips.push_back(std::make_pair(this, ilist));
    while (clips.size()) {
        const csgjs_csgnode *      me = clips.front().first;
        std::vector<csgjs_polygon> list = clips.front().second;
        clips.pop_front();

        if (!me->plane.ok()) {
            result.insert(result.end(), list.begin(), list.end());
            continue;
        }

        std::vector<csgjs_polygon> list_front, list_back;
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitPolygon(list[i], list_front, list_back, list_front, list_back);

        if (me->front)
            clips.push_back(std::make_pair(me->front, list_front));
        else
            result.insert(result.end(), list_front.begin(), list_front.end());

        if (me->back)
            clips.push_back(std::make_pair(me->back, list_back));
    }

    return result;
}

// Remove all polygons in this BSP tree that are inside the other BSP tree
// `bsp`.
void csgjs_csgnode::clipTo(const csgjs_csgnode *other) {
    std::list<csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        csgjs_csgnode *me = nodes.front();
        nodes.pop_front();

        me->polygons = other->clipPolygons(me->polygons);
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }
}

// Return a list of all polygons in this BSP tree.
std::vector<csgjs_polygon> csgjs_csgnode::allPolygons() const {
    std::vector<csgjs_polygon> result;

    std::list<const csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size()) {
        const csgjs_csgnode *me = nodes.front();
        nodes.pop_front();

        result.insert(result.end(), me->polygons.begin(), me->polygons.end());
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }

    return result;
}

csgjs_csgnode *csgjs_csgnode::clone() const {
    csgjs_csgnode *ret = new csgjs_csgnode();

    std::list<std::pair<const csgjs_csgnode *, csgjs_csgnode *>> nodes;
    nodes.push_back(std::make_pair(this, ret));
    while (nodes.size()) {
        const csgjs_csgnode *original = nodes.front().first;
        csgjs_csgnode *      clone = nodes.front().second;
        nodes.pop_front();

        clone->polygons = original->polygons;
        clone->plane = original->plane;
        if (original->front) {
            clone->front = new csgjs_csgnode();
            nodes.push_back(std::make_pair(original->front, clone->front));
        }
        if (original->back) {
            clone->back = new csgjs_csgnode();
            nodes.push_back(std::make_pair(original->back, clone->back));
        }
    }

    return ret;
}

// Build a BSP tree out of `polygons`. When called on an existing tree, the
// new polygons are filtered down to the bottom of the tree and become new
// nodes there. Each set of polygons is partitioned using the first polygon
// (no heuristic is used to pick a good split).
void csgjs_csgnode::build(const std::vector<csgjs_polygon> &ilist) {
    if (!ilist.size())
        return;

    std::list<std::pair<csgjs_csgnode *, std::vector<csgjs_polygon>>> builds;
    builds.push_back(std::make_pair(this, ilist));

    while (builds.size()) {
        csgjs_csgnode *            me = builds.front().first;
        std::vector<csgjs_polygon> list = builds.front().second;
        builds.pop_front();
        assert(list.size() > 0 && "logic error");

        if (!me->plane.ok())
            me->plane = list[0].plane;
        std::vector<csgjs_polygon> list_front, list_back;

        // me->polygons.push_back(list[0]);
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitPolygon(list[i], me->polygons, me->polygons, list_front, list_back);

        if (list_front.size()) {
            if (!me->front)
                me->front = new csgjs_csgnode;
            builds.push_back(std::make_pair(me->front, list_front));
        }
        if (list_back.size()) {
            if (!me->back)
                me->back = new csgjs_csgnode;
            builds.push_back(std::make_pair(me->back, list_back));
        }
    }
}

csgjs_csgnode::csgjs_csgnode() : front(0), back(0) {
}

csgjs_csgnode::csgjs_csgnode(const std::vector<csgjs_polygon> &list) : front(0), back(0) {
    build(list);
}

csgjs_csgnode::~csgjs_csgnode() {
    std::list<csgjs_csgnode *> nodes_to_delete;

    std::list<csgjs_csgnode *> nodes_to_disassemble;
    nodes_to_disassemble.push_back(this);
    while (nodes_to_disassemble.size()) {
        csgjs_csgnode *me = nodes_to_disassemble.front();
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

    for (std::list<csgjs_csgnode *>::iterator it = nodes_to_delete.begin(); it != nodes_to_delete.end(); ++it)
        delete *it;
}

// Public interface implementation

inline std::vector<csgjs_polygon> csgjs_modelToPolygons(const csgjs_model &model) {
    std::vector<csgjs_polygon> list;
    for (size_t i = 0; i < model.indices.size(); i += 3) {
        std::vector<csgjs_vertex> triangle;
        for (int j = 0; j < 3; j++) {
            csgjs_vertex v = model.vertices[model.indices[i + j]];
            triangle.push_back(v);
        }
        list.push_back(csgjs_polygon(triangle));
    }
    return list;
}

csgjs_model csgjs_modelFromPolygons(const std::vector<csgjs_polygon> &polygons) {
    csgjs_model model;
    uint16_t    p = 0;
    for (size_t i = 0; i < polygons.size(); i++) {
        const csgjs_polygon &poly = polygons[i];
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

typedef csgjs_csgnode *csg_function(const csgjs_csgnode *a1, const csgjs_csgnode *b1);

std::vector<csgjs_polygon> csgjs_operation(const std::vector<csgjs_polygon> &apoly,
                                           const std::vector<csgjs_polygon> &bpoly, csg_function fun) {

    csgjs_csgnode A(apoly);
    csgjs_csgnode B(bpoly);

    csgjs_csgnode *AB = fun(&A, &B);
    return AB->allPolygons();
}

inline std::vector<csgjs_polygon> csgjs_operation(const csgjs_model &a, const csgjs_model &b, csg_function fun) {
    return csgjs_operation(csgjs_modelToPolygons(a), csgjs_modelToPolygons(b), fun);
}

csgjs_model csgsmodel_cube(const csgjs_vector &center, const csgjs_vector &dim) {

    struct Quad {
        int          indices[4];
        csgjs_vector normal;
    } quads[] = {{{0, 4, 6, 2}, {-1, 0, 0}}, {{1, 3, 7, 5}, {+1, 0, 0}}, {{0, 1, 5, 4}, {0, -1, 0}},
                 {{2, 6, 7, 3}, {0, +1, 0}}, {{0, 2, 3, 1}, {0, 0, -1}}, {{4, 5, 7, 6}, {0, 0, +1}}};

    std::vector<csgjs_polygon> polygons;
    for (const auto &q : quads) {

        std::vector<csgjs_vertex> verts;

        for (auto i : q.indices) {
            csgjs_vector pos(center.x + dim.x * (2.0f * !!(i & 1) - 1), center.y + dim.y * (2.0f * !!(i & 2) - 1),
                             center.z + dim.z * (2.0f * !!(i & 4) - 1));

            verts.push_back({pos, q.normal, csgjs_vector2{0.0f, 0.0f}});
        }
        polygons.push_back(csgjs_polygon(verts));
    }

    return csgjs_modelFromPolygons(polygons);
}

csgjs_model csgsmodel_sphere(const csgjs_vector &c, float r, int slices, int stacks) {

    std::vector<csgjs_polygon> polygons;

    auto vertex = [c, r](float theta, float phi) -> csgjs_vertex {
        theta *= (float)M_PI * 2;
        phi *= (float)M_PI;
        csgjs_vector dir(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));

        return csgjs_vertex{c + (dir * r), dir, {0.0f, 0.0f}};
    };
    for (float i = 0; i < slices; i++) {
        for (float j = 0; j < stacks; j++) {

            std::vector<csgjs_vertex> vertices;

            vertices.push_back(vertex(i / slices, j / stacks));
            if (j > 0) {
                vertices.push_back(vertex((i + 1) / slices, j / stacks));
            }
            if (j < stacks - 1) {
                vertices.push_back(vertex((i + 1) / slices, (j + 1) / stacks));
            }
            vertices.push_back(vertex(i / slices, (j + 1) / stacks));
            polygons.push_back(csgjs_polygon(vertices));
        }
    }
    return csgjs_modelFromPolygons(polygons);
}

csgjs_model csgsmodel_cyliner(const csgjs_vector &s, const csgjs_vector &e, float r, int slices) {
    
	csgjs_vector      ray = e - s;

	csgjs_vector      axisZ = unit(ray);
	bool isY = fabs(axisZ.y) > 0.5f;
	csgjs_vector      axisX = unit(cross(csgjs_vector(isY, !isY, 0), axisZ));
	csgjs_vector      axisY = unit(cross(axisX, axisZ));

	csgjs_vertex      start{ s, -axisZ };
	csgjs_vertex      end{ e, unit(axisZ) };

	std::vector<csgjs_polygon> polygons;

    auto point = [axisX, axisY, s, r, ray, axisZ](float stack, float slice, float normalBlend) -> csgjs_vertex {
        float angle = slice * (float)M_PI * 2;
		csgjs_vector out = axisX * cosf(angle) + axisY * sinf(angle);
		csgjs_vector pos = s + ray * stack + out * r;
		csgjs_vector normal = out * (1.0f - fabs(normalBlend)) + axisZ * normalBlend;
		return csgjs_vertex{ pos, normal };
	};

    for (float i = 0; i < slices; i++) {
		float t0 = i / slices;
		float t1 = (i + 1) / slices;
		polygons.push_back(csgjs_polygon({ start, point(0, t0, -1), point(0, t1, -1) }));
		polygons.push_back(csgjs_polygon({ point(0, t1, 0), point(0, t0, 0), point(1, t0, 0), point(1, t1, 0) }));
		polygons.push_back(csgjs_polygon({ end, point(1, t1, 1), point(1, t0, 1) }));
    }
    return csgjs_modelFromPolygons(polygons);
};

csgjs_model csgjs_union(const csgjs_model &a, const csgjs_model &b) {
    return csgjs_modelFromPolygons(csgjs_operation(a, b, csg_union));
}

csgjs_model csgjs_intersection(const csgjs_model &a, const csgjs_model &b) {
    return csgjs_modelFromPolygons(csgjs_operation(a, b, csg_intersect));
}

csgjs_model csgjs_subtract(const csgjs_model &a, const csgjs_model &b) {
    return csgjs_modelFromPolygons(csgjs_operation(a, b, csg_subtract));
}

std::vector<csgjs_polygon> csgjs_union(const std::vector<csgjs_polygon> &a, const std::vector<csgjs_polygon> &b) {
    return csgjs_operation(a, b, csg_union);
}

std::vector<csgjs_polygon> csgjs_intersection(const std::vector<csgjs_polygon> &a,
                                              const std::vector<csgjs_polygon> &b) {
    return csgjs_operation(a, b, csg_intersect);
}

std::vector<csgjs_polygon> csgjs_subtract(const std::vector<csgjs_polygon> &a, const std::vector<csgjs_polygon> &b) {
    return csgjs_operation(a, b, csg_subtract);
}

#endif

#endif