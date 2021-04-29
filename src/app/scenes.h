#include <string>
#include <vector>

namespace crl {
namespace app {
namespace rigidbody {

enum SimulationScene {
    Projectile = 0,
    FixedSprings = 1,
    RigidBodySprings = 2,
};

// note. this is static variable.
std::vector<std::string> simulationSceneNames = {
    "Projectile",    //
    "Fixed Springs",  //
    "Rigid Body Springs"
};

}  // namespace rigidbody
}  // namespace app
}  // namespace crl