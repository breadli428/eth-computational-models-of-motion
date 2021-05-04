#pragma once

#include <gui/model.h>
#include <gui/shader.h>
#include <sim/RB.h>

namespace crl {

class RBRenderer {
public:
    /* methods used to draw different types of views of the rigid body... */

    static void drawCoordFrame(const RB *rb, const gui::Shader &shader);

    static void drawMOI(const RB *rb, const gui::Shader &shader);

    static void drawMOI(const RBState &rbState, const RBProperties &rbProps,
                        const gui::Shader &shader, bool wireFrame = false);

    static void drawCollisionRB(const RB *rb, const gui::Shader &shader);
};

}  // namespace crl