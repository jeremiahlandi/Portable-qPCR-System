#include "core/ui/Launcher.h"

#include <cstring>

namespace qpcr::ui {

std::size_t launcherTileIndex(const char* id) {
    if (id == nullptr) {
        return kLauncherTiles.size();
    }
    for (std::size_t i = 0; i < kLauncherTiles.size(); ++i) {
        if (std::strcmp(kLauncherTiles.at(i).id, id) == 0) {
            return i;
        }
    }
    return kLauncherTiles.size();
}

} // namespace qpcr::ui
