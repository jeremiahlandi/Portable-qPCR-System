#pragma once

#include <array>
#include <cstddef>

// Home screen tiles (issue #11). Placeholders until the operator screens (#12) exist; the ids
// are stable keys the navigation layer will route on, the labels are what the operator reads.
// Derived from the legacy Nextion flow (Kal_ATMega_New_7.ino event codes 101-109): a status
// home page, settings pages for PCR / reverse transcription / extraction protocols, a run page,
// and run logs on the SD card.

namespace qpcr::ui {

struct LauncherTile {
    const char* id;
    const char* label;
};

constexpr std::array<LauncherTile, 4> kLauncherTiles{{
    {"run", "Run"},
    {"protocols", "Protocols"},
    {"results", "Results"},
    {"settings", "Settings"},
}};

/// Index of the tile with this id, or kLauncherTiles.size() when there is none.
[[nodiscard]] std::size_t launcherTileIndex(const char* id);

} // namespace qpcr::ui
