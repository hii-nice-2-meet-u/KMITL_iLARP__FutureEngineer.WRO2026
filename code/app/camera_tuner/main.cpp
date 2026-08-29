#include "tunner.hpp"

int main() {
	camera::Tunner tunner(640, 640, 90.0f, 1.8f, 2.8f);

	tunner.run();

	return 0;
}
