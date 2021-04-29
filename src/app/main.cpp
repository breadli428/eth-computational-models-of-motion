#include <iostream>

#include "app.h"

int main() {
    crl::app::rigidbody::App app;
    app.setCallbacks();
    app.run();

    return 0;
}
