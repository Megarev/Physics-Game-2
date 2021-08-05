#pragma once
#include "olcPixelGameEngine.h"
#include <random>

constexpr float PI = 3.1415926f;
constexpr float g = 10.0f;

static float Random(float a, float b) {
	std::random_device rd;
	static std::mt19937 m(rd());
	std::uniform_real_distribution<float> dist(a, b);

	return dist(m);
}

struct Debug {
	olc::PixelGameEngine* pge;
} debug;

class PolygonShape {
public:
	std::vector<olc::vf2d> model, vertices;
	std::vector<int> lengths;
	float poly_len_max = 0.0f;

	float angle = 0.0f, angular_velocity = 0.0f, angular_acceleration = 0.0f;
	olc::vf2d position, centroid, velocity, acceleration;
	int n_vertices = 0;

	float mass = 2.0f, inv_mass = 0.0f;
	float I = 0.0f, inv_I = 0.0f;
	float e = 0.1f;
	float sf = 0.6f, df = 0.4f;

	float a = 0.0f, b = 0.0f;

	std::vector<int> triangles;
	olc::Pixel init_color, color;

	bool first_iter = true;
public:
	PolygonShape() {}
	PolygonShape(const olc::vf2d& p, float _angle, int n, float a, float b, float _mass);
	void Initialize();

	void Update(float dt, bool is_gravity = false);
	void ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact);

	void Triangulate();

	void WrapScreen(const olc::vf2d& bounds);

	void Draw(olc::PixelGameEngine* pge, bool is_fill = false, bool is_debug = false);
};

struct Edge { olc::vf2d v1, v2, edge, farthest, poly_pos; };
struct ManifoldPolygonData { olc::vf2d position; std::vector<olc::vf2d> vertices; };

class Manifold {
public:
	PolygonShape* p1 = nullptr, * p2 = nullptr;
	ManifoldPolygonData manifold1, manifold2;

	olc::vf2d normal;
	float depth = 0.0f;

	Edge ref, inc;
	std::vector<olc::vf2d> contacts;

	Edge GetSupport(float dir, int index);
	void GetRefIncEdges();
	bool Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o, std::vector<olc::vf2d>& points);
public:
	Manifold() {}
	Manifold(PolygonShape* _p1, PolygonShape* _p2, const olc::vf2d& _normal, float _depth,
		const ManifoldPolygonData& m1, const ManifoldPolygonData& m2)
		: p1(_p1), p2(_p2), normal(_normal), depth(_depth), manifold1(m1), manifold2(m2) {}

	void GenerateContacts();

	void DynamicResolution();
	void StaticResolution();
};


class Collisions {
private:
	Collisions() {}
public:
	static Collisions& Get() {
		static Collisions c;
		return c;
	}

	bool CircleTest(PolygonShape& p1, PolygonShape& p2);
	bool SAT(PolygonShape& p1, PolygonShape& p2, std::vector<Manifold>& manifolds);
};


class Scene {
private:
	std::vector<PolygonShape> shapes;

	float FPS = 60.0f;
	float inv_FPS = 1.0f / FPS;
	float delay = 0.1f;
	float accumulator = 0.0f;

	olc::vf2d screen_bounds;
public:
	Scene() {}
	Scene(const olc::vf2d& bounds) { screen_bounds = bounds; }

	void Logic(float dt, bool is_wrap = false, bool is_gravity = false);

	void Draw(olc::PixelGameEngine* pge, bool is_fill = false, bool is_debug = false);

	void AddShape(const PolygonShape& s) { shapes.push_back(s); }
	PolygonShape& GetShape(int index) { return shapes[index]; }
	std::vector<PolygonShape>& GetShapes() { return shapes; }
};

// PolygonShape

PolygonShape::PolygonShape(const olc::vf2d& p, float _angle, int n, float _a, float _b, float _mass) 
	: position(p), angle(_angle), n_vertices(n), mass(_mass), a(_a), b(_b) {
	for (int i = 0; i < n_vertices; i++) {
		model.push_back({ cosf(2.0f * PI / n_vertices * i), sinf(2.0f * PI / n_vertices * i) });
		lengths.push_back(Random(a, b));
	}
}

void PolygonShape::Initialize() {

	n_vertices = (int)model.size();

	init_color = olc::Pixel(rand() % 256, rand() % 256, rand() % 256);
	color = init_color;

	float len_min = INFINITY, len_max = -INFINITY;

	for (int i = 0; i < n_vertices; i++) {

		float len = lengths[i];

		len_min = std::fminf(len_min, len);
		len_max = std::fmaxf(len_max, len);
	}

	float len_mean = std::sqrtf(len_min * len_max);
	I = mass * len_mean * len_mean / 12.0f;

	inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
	inv_I = I == 0.0f ? 0.0f : 1.0f / I;

	vertices.resize(n_vertices);
	Update(0.0f);
	Triangulate();
}

void PolygonShape::Update(float dt, bool is_gravity) {

	if (mass > 0.0f) {
		velocity += (acceleration + is_gravity * olc::vf2d{ 0.0f, g }) * dt;
		position += velocity * dt;

		angular_velocity += angular_acceleration * dt;
		angle += angular_velocity * dt;

		acceleration = { 0.0f, 0.0f };
		angular_acceleration = 0.0f;
	}

	float c = cosf(angle), s = sinf(angle);

	for (int i = 0; i < n_vertices; i++) {
		vertices[i] = {
			model[i].x * c + model[i].y * s,
			model[i].x * s - model[i].y * c
		};

		vertices[i] *= lengths[i];

		vertices[i] += position;
	}

	centroid = { 0.0f, 0.0f };
	for (auto& v : vertices) centroid += v;
	centroid /= vertices.size();
	
	if (first_iter) {
		first_iter = false;
		float len_max = -INFINITY;
		for (int i = 0; i < n_vertices; i++) {
			olc::vf2d distance = centroid - vertices[i];
			len_max = std::fmaxf(distance.mag(), len_max);
		}
		poly_len_max = len_max;
	}
}

void PolygonShape::ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact) {
	acceleration += impulse * inv_mass;
	angular_acceleration += contact.cross(impulse) * inv_I;
}

void PolygonShape::Triangulate() {
	triangles.clear();
	std::vector<int> index_list;
	for (int i = 0; i < n_vertices; i++) index_list.push_back(i);

	auto IsContainPointInTriangle = [](const olc::vf2d& a, const olc::vf2d& b, const olc::vf2d& c, const olc::vf2d& p) {
		const olc::vf2d& ab = b - a;
		const olc::vf2d& bc = c - b;
		const olc::vf2d& ca = a - c;

		const olc::vf2d& pa = a - p;
		const olc::vf2d& pb = b - p;
		const olc::vf2d& pc = c - p;

		float c1 = ab.cross(pa);
		float c2 = bc.cross(pb);
		float c3 = ca.cross(pc);

		if (c1 < 0.0f && c2 < 0.0f && c3 < 0.0f) return true;

		return false;
	};

	while (index_list.size() > 3) {
		for (int i = 0; i < index_list.size(); i++) {
			if (index_list.size() == 3) break;
			int a = index_list[(i + index_list.size() - 1) % index_list.size()];
			int b = index_list[i];
			int c = index_list[(i + 1) % index_list.size()];

			const olc::vf2d& ba = vertices[a] - vertices[b];
			const olc::vf2d& bc = vertices[c] - vertices[b];

			if (ba.cross(bc) <= 0.0f) continue;

			bool is_triangulate = true;
			for (int j = 0; j < vertices.size(); j++) {
				if (j == a || j == b || j == c) continue;
				if (IsContainPointInTriangle(vertices[a], vertices[b], vertices[c], vertices[j])) {
					is_triangulate = false;
					break;
				}
			}

			if (is_triangulate) {
				triangles.push_back(a);
				triangles.push_back(b);
				triangles.push_back(c);

				index_list.erase(index_list.begin() + i);
			}
		}
	}

	triangles.push_back(index_list[0]);
	triangles.push_back(index_list[1]);
	triangles.push_back(index_list[2]);
}

void PolygonShape::WrapScreen(const olc::vf2d& bounds) {
	if (position.x < 0.0f) position.x += bounds.x;
	if (position.y < 0.0f) position.y += bounds.y;
	if (position.x > bounds.x) position.x -= bounds.x;
	if (position.y > bounds.y) position.y -= bounds.y;
}


void PolygonShape::Draw(olc::PixelGameEngine* pge, bool is_fill, bool is_debug) {
	if (!is_fill) {
		for (int i = 0; i < n_vertices; i++) {
			int j = (i + 1) % n_vertices;
			pge->DrawLine(vertices[i], vertices[j], color);
		}
	}
	else {
		for (int i = 0; i < triangles.size() / 3; i++) {
			int a = (3 * i + 0);
			int b = (3 * i + 1);
			int c = (3 * i + 2);

			pge->FillTriangle(vertices[triangles[a]], vertices[triangles[b]], vertices[triangles[c]], color);
		}
	}

	if (is_debug) {
		pge->FillCircle(centroid, 2);
		pge->DrawCircle(centroid, poly_len_max);
	}
}

// Manifold

Edge Manifold::GetSupport(float dir, int index) {
	const olc::vf2d& n = normal * dir;
	const std::vector<olc::vf2d>& vertices = !index ? manifold1.vertices : manifold2.vertices;

	float distance = -INFINITY;
	int vertex_index = -1;

	for (int i = 0; i < vertices.size(); i++) {
		float new_distance = n.dot(vertices[i]);
		if (new_distance > distance) {
			distance = new_distance;
			vertex_index = i;
		}
	}

	const olc::vf2d& vertex = vertices[vertex_index];
	const olc::vf2d& prev_vertex = vertices[(vertex_index + 1) % vertices.size()];
	const olc::vf2d& next_vertex = vertices[(vertex_index + vertices.size() - 1) % vertices.size()];

	const olc::vf2d& prev_edge = vertex - prev_vertex;
	const olc::vf2d& next_edge = vertex - next_vertex;

	const olc::vf2d& position = !index ? manifold1.position : manifold2.position;

	if (prev_edge.dot(n) <= next_edge.dot(n)) {
		return Edge{ prev_vertex, vertex, prev_edge, vertex, position };
	}
	else {
		return Edge{ next_vertex, vertex, next_edge, vertex, position };
	}
}

void Manifold::GetRefIncEdges() {
	const Edge& e1 = GetSupport(+1.0f, 0);
	const Edge& e2 = GetSupport(-1.0f, 1);

	if (std::fabsf(e1.edge.dot(normal)) <= std::fabsf(e2.edge.dot(normal))) { ref = e1; inc = e2; }
	else { ref = e2; inc = e1; }
}

bool Manifold::Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o, std::vector<olc::vf2d>& points) {

	float d1 = n.dot(v1) - o;
	float d2 = n.dot(v2) - o;

	if (d1 >= 0.0f) points.push_back(v1);
	if (d2 >= 0.0f) points.push_back(v2);

	if (d1* d2 < 0.0f) {
		points.push_back(v1 + d1 / (d1 - d2) * (v2 - v1));
	}

	return points.size() > 1;
}

void Manifold::GenerateContacts() {
	GetRefIncEdges();
	const olc::vf2d& ref_edge = ref.edge.norm();

	std::vector<olc::vf2d> cp0;
	if (!Clip(inc.v1, inc.v2, ref_edge, ref_edge.dot(ref.v1), cp0)) return;

	std::vector<olc::vf2d> cp1;
	if (!Clip(cp0[0], cp0[1], -ref_edge, -ref_edge.dot(ref.v2), cp1)) return;

	olc::vf2d ref_norm = ref_edge.perp();
	if ((ref.poly_pos - inc.poly_pos).dot(ref_norm) <= 0.0f) ref_norm *= -1.0f;

	float d = ref_norm.dot(ref.farthest);
	if (ref_norm.dot(cp1[1]) < d) cp1.erase(cp1.begin() + 1);
	else if (ref_norm.dot(cp1[0]) < d) cp1.erase(cp1.begin() + 0);

	contacts = std::move(cp1);
	for (auto& c : contacts) debug.pge->FillCircle(c, 5, olc::MAGENTA);
}

void Manifold::DynamicResolution() {
	if (contacts.size() == 0) return;

	auto VectorProduct = [](float a, const olc::vf2d& v) -> olc::vf2d { return olc::vf2d(v.y * -a, v.x * a); };

	for (auto& p : contacts) {
		const olc::vf2d& m1 = p - p1->position;
		const olc::vf2d& m2 = p - p2->position;

		const olc::vf2d& v1 = VectorProduct(p1->angular_velocity, m1);
		const olc::vf2d& v2 = VectorProduct(p2->angular_velocity, m2);

		const olc::vf2d& rv = (p2->velocity + v2) - (p1->velocity + v1);
		float rv_normal = rv.dot(normal);

		if (rv_normal > 0.0f) return;

		float m1_norm = m1.cross(normal);
		float m2_norm = m2.cross(normal);

		float inv_mass_sum = (
			p1->inv_mass + m1_norm * m1_norm * p1->inv_I +
			p2->inv_mass + m2_norm * m2_norm * p2->inv_I
			);

		float e = std::fminf(p1->e, p2->e);
		float j = -(1.0f + e) * rv_normal / inv_mass_sum;
		j /= contacts.size();

		const olc::vf2d& impulse = j * normal;
		p1->ApplyImpulse(-impulse, m1);
		p2->ApplyImpulse(impulse, m2);
		if (contacts.size() == 1) continue;

		olc::vf2d tangent = normal.perp();
		if (tangent.dot(rv) < 0.0f) tangent *= -1.0f;
		float rv_tangent = rv.dot(tangent);

		float jt = -rv_tangent / inv_mass_sum;
		jt /= contacts.size();
	
		olc::vf2d friction_impulse;
		float sf = (p1->sf + p2->sf) / 2.0f;
		float df = (p1->df + p2->df) / 2.0f;

		if (std::fabsf(jt) <= j * sf) friction_impulse = jt * tangent;
		else friction_impulse = -j * df * tangent;

		p1->ApplyImpulse(-friction_impulse, m1);
		p2->ApplyImpulse(friction_impulse, m2);
	}
}

void Manifold::StaticResolution() {

	float p = 0.5f;
	float overlap_depth = std::fmaxf(depth, 0.0f) * p / (p1->mass + p2->mass);

	p1->position -= normal * overlap_depth * p1->mass;
	p2->position += normal * overlap_depth * p2->mass;
}

// Collisions

bool Collisions::CircleTest(PolygonShape& p1, PolygonShape& p2) {
	return (
		(p1.centroid.x - p2.centroid.x) * (p1.centroid.x - p2.centroid.x) +
		(p1.centroid.y - p2.centroid.y) * (p1.centroid.y - p2.centroid.y) <=
		((p1.poly_len_max + p2.poly_len_max) * (p1.poly_len_max + p2.poly_len_max))
	);
}

bool Collisions::SAT(PolygonShape& p1, PolygonShape& p2, std::vector<Manifold>& manifolds) {

	PolygonShape* poly1 = &p1, * poly2 = &p2;
	p1.color = p1.init_color;
	p2.color = p2.init_color;
	bool is_intersect = false;

	if (!CircleTest(p1, p2)) return false;

	auto TriangleSAT = [](std::vector<olc::vf2d> vertices_a, std::vector<olc::vf2d> vertices_b,
		const olc::vf2d& pos_a, const olc::vf2d& pos_b, float& overlap, olc::vf2d& res_normal) -> bool {

		for (int n = 0; n < 2; n++) {
			if (n) std::swap(vertices_a, vertices_b);

			for (int a = 0; a < 3; a++) {
				int b = (a + 1) % 3;
				olc::vf2d normal = (vertices_a[b] - vertices_a[a]).perp().norm();

				float min1 = INFINITY, max1 = -INFINITY;

				for (int k = 0; k < 3; k++) {
					float q = vertices_a[k].dot(normal);
					min1 = std::fminf(min1, q);
					max1 = std::fmaxf(max1, q);
				}

				float min2 = INFINITY, max2 = -INFINITY;

				for (int k = 0; k < 3; k++) {
					float q = vertices_b[k].dot(normal);
					min2 = std::fminf(min2, q);
					max2 = std::fmaxf(max2, q);
				}

				float new_overlap = std::fminf(max1, max2) - std::fmaxf(min1, min2);

				if (!(max1 >= min2 && max2 >= min1)) return false;

				if (new_overlap < overlap) {
					overlap = new_overlap;
					res_normal = normal;
				}
			}
		}

		if ((pos_b - pos_a).dot(res_normal) <= 0.0f) res_normal *= -1.0f;

		return true;
	};

	for (int i = 0; i < poly1->triangles.size() / 3; i++) {

		std::vector<olc::vf2d> poly1_vertices = {
			poly1->vertices[poly1->triangles[3 * i + 0]],
			poly1->vertices[poly1->triangles[3 * i + 1]],
			poly1->vertices[poly1->triangles[3 * i + 2]]
		};

		for (int j = 0; j < poly2->triangles.size() / 3; j++) {
			std::vector<olc::vf2d> poly2_vertices = {
				poly2->vertices[poly2->triangles[3 * j + 0]],
				poly2->vertices[poly2->triangles[3 * j + 1]],
				poly2->vertices[poly2->triangles[3 * j + 2]]
			};

			float overlap = INFINITY;
			olc::vf2d res_normal;

			if (TriangleSAT(poly1_vertices, poly2_vertices, poly1->position, poly2->position, overlap, res_normal)) {
				is_intersect = true;
				manifolds.push_back(Manifold(&p1, &p2, res_normal, overlap,
					ManifoldPolygonData{ poly1->position, poly1_vertices }, ManifoldPolygonData{ poly2->position, poly2_vertices }));
			}
		}
	}

	return is_intersect;
}

// Scene
void Scene::Logic(float dt, bool is_wrap, bool is_gravity) {
	accumulator = std::fminf(accumulator + dt, delay);

	while (accumulator > inv_FPS) {
		accumulator -= inv_FPS;

		for (int i = shapes.size() - 1; i >= 0; i--) {
			shapes[i].Update(inv_FPS, is_gravity);
			if (is_wrap) {
				shapes[i].WrapScreen(screen_bounds);
			}
			else {
				const olc::vf2d& p = shapes[i].position;
				if (p.x < 0.0f || p.y < 0.0f || p.x > screen_bounds.x || p.y > screen_bounds.y) shapes.erase(shapes.begin() + i);
			}
		}

		std::vector<Manifold> manifolds;
		for (int a = 0; a < shapes.size() - 1; a++) {
			for (int b = a + 1; b < shapes.size(); b++) {
				if (Collisions::Get().SAT(shapes[a], shapes[b], manifolds)) {
					//shapes[a].color = olc::WHITE;
					//shapes[b].color = olc::WHITE;
				}

			}
		}
		
		for (auto& m : manifolds) {
			m.GenerateContacts();
			m.StaticResolution();
			m.DynamicResolution();
		}
	}
}

void Scene::Draw(olc::PixelGameEngine* pge, bool is_fill, bool is_debug) {
	for (auto& s : shapes) s.Draw(pge, is_fill, is_debug);
}