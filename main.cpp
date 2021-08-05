#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "physics2.h"

class Game : public olc::PixelGameEngine {
private:
	Scene scene;
	bool is_debug = false;

	void AddRectangle(const olc::vf2d& position, float angle, float len, float mass) {
		PolygonShape s{ position, angle, 4, 0.0f, 0.0f, mass };
		s.model[0] = { -1.0f, -0.5f };
		s.model[1] = { 1.0f, -0.5f };
		s.model[2] = { 1.0f,  0.5f };
		s.model[3] = { -1.0f,  0.5f };

		for (auto& l : s.lengths) l = len;
		s.Initialize();
		scene.AddShape(s);
	}

	void AddShape(const olc::vf2d& position, float angle, float len, float mass) {
		PolygonShape s{ position, angle, 8, 0.0f, 0.0f, mass };
		switch (rand() % 2) {
		case 0:
			s.model.resize(6);
			s.model[0] = { -1.0f, -1.0f };
			s.model[1] = { 1.0f, -1.0f };
			s.model[2] = { 1.0f,  0.0f };
			s.model[3] = { 0.0f,  0.0f };
			s.model[4] = { 0.0f,  1.0f };
			s.model[5] = { -1.0f,  1.0f };
			break;
		case 1:
			s.model.resize(8);
			s.model[0] = { -1.0f, -1.0f };
			s.model[1] = { 0.0f, -1.0f };
			s.model[2] = { 0.0f, -0.5f };
			s.model[3] = { 1.0f, -0.5f };
			s.model[4] = { 1.0f, 1.0f };
			s.model[5] = { 0.0f, 1.0f };
			s.model[6] = { 0.0f, 0.5f };
			s.model[7] = { -1.0f, 0.5f };
			break;
		}

		for (auto& l : s.lengths) l = len;
		s.Initialize();
		scene.AddShape(s);
	}
public:
	Game() {
		sAppName = "Title";
	}

	bool OnUserCreate() override {

		scene = Scene{ { ScreenWidth() * 1.0f, ScreenHeight() * 1.0f } };
		for (int i = 0; i < 4; i++) {
			AddShape({ rand() % ScreenWidth() * 1.0f, rand() % ScreenHeight() * 1.0f }, Random(0.0f, 2.0f * PI), 25.0f, Random(2.0f, 10.0f));
		}
		AddRectangle({ ScreenWidth() * 0.5f, ScreenHeight() - 100.0f }, 0.0f, 100.0f, 0.0f);

		debug.pge = this;

		return true;
	}

	bool OnUserUpdate(float dt) override {

		const olc::vf2d& m_pos = GetMousePos() * 1.0f;

		// Input
		if (GetMouse(0).bPressed) {
			//scene.AddShape(PolygonShape(m_pos, Random(0.0f, 2.0f * PI), 10, 25.0f, 100.0f, Random(2.0f, 10.0f)));
			//scene.GetShapes().back().Initialize();
			AddShape(m_pos, Random(0.0f, 2.0f * PI), Random(25.0f, 75.0f), Random(2.0f, 20.0f));
		}

		if (GetKey(olc::SPACE).bPressed) is_debug = !is_debug;

		PolygonShape& player_shape = scene.GetShape(0);

		float a = 100.0f;
		if (GetKey(olc::LEFT).bHeld)  player_shape.acceleration.x -= a;
		if (GetKey(olc::RIGHT).bHeld) player_shape.acceleration.x += a;
		if (GetKey(olc::UP).bHeld)	  player_shape.acceleration.y -= a;
		if (GetKey(olc::DOWN).bHeld)  player_shape.acceleration.y += a;

		// Logic
		int iter = 2;
		Clear(olc::BLACK);
		for (int i = 0; i < iter; i++) scene.Logic(dt, false, true);

		// Render
		scene.Draw(this, false, is_debug);

		return true;
	}
};

int main() {

	srand((unsigned)time(0));

	Game game;
	if (game.Construct(512, 512, 1, 1, false, true)) {
		game.Start();
	}

	return 0;
}