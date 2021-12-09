#include "Participant.h"

bool raptor::Participant::check_for_data() {
    float x;

    for (int i = 0; i < 30; ++i) {
        x = this->pose_.pose.position.x;
    }
    if (x == 0.0) {
        return false;
    }
    return true;
}