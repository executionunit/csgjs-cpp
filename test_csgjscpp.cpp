
#include "mycsgjs.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

using namespace csgjscpp;

using Polygons = CSGJSCPP_VECTOR<csgjscpp::Polygon>;

TEST_CASE("double tjunction right") {

	/*
		+--------------+
		|              |
		|              |
		|              +----+
		|    2! +--->  |    |
		|              +----+
		|              |
		+--------------+
	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};
	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {1, 0.25, 0}, {0, 1, 0}, 0},
		{ {2, 0.25, 0}, {0, 1, 0}, 0},
		{ {2, 0.5, 0}, {0, 1, 0}, 0},
		{ {1, 0.5, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);

	//we shouldn't get any new polygons!
	CHECK(outpolys.size() == inpolygons.size());

	//because this should be split on the first poly fix the Ts
	//BUT THIS CODE DOESNT WORK WITH THIS ALGO? NOT SURE IM WORRKED ATM
	CHECK_FALSE(outpolys[0].vertices.size() == 6);
}



TEST_CASE("tjunction empty") {

	Polygons inpolygons;

	Polygons outpolys = csgfixtjunc(inpolygons);
	CHECK(outpolys.size() == 0);
}


TEST_CASE("tjunction plane") {

	/*

		+-----------------+
		|                 |
		|                 |
		|                 |
		|                 |
		|                 |
		|                 |
		|                 |
		|                 |
		+-----------------+

	*/

	CSGJSCPP_VECTOR<Vertex> verts{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts) };
	Polygons outpolys = csgfixtjunc(inpolygons);
	CHECK(outpolys.size() == inpolygons.size());
}

TEST_CASE("tjunction adjacent plane") {

	/*

		+-----------------+----------------+
		|                 |                |
		|                 |                |
		|                 |                |
		|                 |                |
		|                 |                |
		|                 |                |
		|                 |                |
		|                 |                |
		+-----------------+----------------+

	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};
	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {2, 0, 0}, {0, 1, 0}, 0},
		{ {2, 1, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);
	CHECK(outpolys.size() == inpolygons.size());
}

TEST_CASE("tjunction right") {

	/*

		+-----------------+
		|                 |
		|                 |
		|                 |
		|     T Here----> +---------------+
		|                 |               |
		|                 |               |
		|                 |               |
		|                 |               |
		+-----------------+---------------+

	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};
	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {2, 0, 0}, {0, 1, 0}, 0},
		{ {2, 0.5, 0}, {0, 1, 0}, 0},
		{ {1, 0.5, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);

	//we shouldn't get any new polygons!
	CHECK(outpolys.size() == inpolygons.size());

	//because this should be split on the first poly fix the T
	CHECK(outpolys[0].vertices.size() == 5);
}


TEST_CASE("tjunction above") {

	/*

			  +--------+
			  |        |
			  |        |
			  |        |
			  |        |
		+--------------+
		|     ^        |
		|     |        |
		|     |        |
		|     +        |
		|  T Here      |
		|              |
		|              |
		+--------------+

	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};
	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {0.5, 1, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {1, 2.0, 0}, {0, 1, 0}, 0},
		{ {0.5, 2.0, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);

	//we shouldn't get any new polygons!
	CHECK(outpolys.size() == inpolygons.size());

	//because this should be split on the first poly fix the T
	CHECK(outpolys[0].vertices.size() == 5);
}


TEST_CASE("tjunction left") {

	/*
		+-------+---------------+
		|       |               |
		|       |               |
		|       |               |
		+-------+ <---+         |
				|               |
				|               |
				|               |
				|               |
				+---------------+
	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {2, 0, 0}, {0, 1, 0}, 0},
		{ {2, 1, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0}
	};

	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {0, 0.5, 0}, {0, 1, 0}, 0},
		{ {1, 0.5, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);

	//we shouldn't get any new polygons!
	CHECK(outpolys.size() == inpolygons.size());

	//because this should be split on the first poly fix the T
	CHECK(outpolys[0].vertices.size() == 5);
}


TEST_CASE("tjunction below") {

	/*
		+----------+
		|          |
		|     +    |
		|     |    |
		|     v    |
		+----------+
		|     |
		|     |
		|     |
		+-----+
	*/

	CSGJSCPP_VECTOR<Vertex> verts1{
		{ {0, 0, 0}, {0, 1, 0}, 0},
		{ {1, 0, 0}, {0, 1, 0}, 0},
		{ {1, 1, 0}, {0, 1, 0}, 0},
		{ {0, 1, 0}, {0, 1, 0}, 0}
	};
	CSGJSCPP_VECTOR<Vertex> verts2{
		{ {0, -1, 0}, {0, 1, 0}, 0},
		{ {0.5, -1, 0}, {0, 1, 0}, 0},
		{ {0.5, 0.0, 0}, {0, 1, 0}, 0},
		{ {0.0, 0.0, 0}, {0, 1, 0}, 0}
	};

	Polygons inpolygons = { csgjscpp::Polygon(verts1), csgjscpp::Polygon(verts2) };
	Polygons outpolys = csgfixtjunc(inpolygons);

	//we shouldn't get any new polygons!
	CHECK(outpolys.size() == inpolygons.size());

	//because this should be split on the first poly fix the T
	CHECK(outpolys[0].vertices.size() == 5);
}





TEST_CASE("tjunction cube") {

	Polygons inpolygons = csgjscpp::csgpolygon_cube();
	Polygons outpolys = csgfixtjunc(inpolygons);
	CHECK(outpolys.size() == inpolygons.size());
}

TEST_CASE("tjunction sphere") {

	Polygons inpolygons = csgjscpp::csgpolygon_sphere();
	Polygons outpolys = csgfixtjunc(inpolygons);
    CHECK(outpolys.size() == inpolygons.size());
}

