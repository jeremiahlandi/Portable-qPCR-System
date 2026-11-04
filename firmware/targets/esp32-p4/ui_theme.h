#pragma once

#include <cstdint>

// Shared colours and metrics for every LVGL screen (issue #11). Keep screens visually one
// system: dark background, one accent, status bar on top.

namespace qpcr::target::theme {

constexpr std::uint32_t kBackground = 0x101418;
constexpr std::uint32_t kBarBackground = 0x1c2229;
constexpr std::uint32_t kPanel = 0x1a2026;
constexpr std::uint32_t kText = 0xf2f4f6;
constexpr std::uint32_t kMuted = 0x9aa4ad;
constexpr std::uint32_t kAccent = 0x2f6fde;
constexpr std::uint32_t kGood = 0x2a9d5c;
constexpr std::uint32_t kBad = 0xd63a3a;

constexpr int kBarHeight = 28;
constexpr int kHeaderHeight = 40;
constexpr int kPad = 8;

} // namespace qpcr::target::theme
