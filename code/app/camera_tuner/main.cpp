#include "tunner.hpp"

int main() {
	camera::Tunner tunner(640, 640, 90.0f, 1.4f, 2.6f);

	// tunner.tune_white();

	// tunner.tune_red_hsv();

	// tunner.tune_green_hsv();

	// tunner.tune_magenta_hsv();
	tunner.run();

	return 0;
}