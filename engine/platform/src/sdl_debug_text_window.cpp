#include "ocs/platform/debug_text_window.hpp"

#include <SDL3/SDL.h>

#include <algorithm>

#include "ocs/core/log.hpp"

namespace ocs::platform {

DebugTextWindow::~DebugTextWindow() {
    destroy();
}

bool DebugTextWindow::create(const std::string& title, const int width, const int height) {
    destroy();

    window_ = SDL_CreateWindow(
        title.c_str(),
        width,
        height,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window_ == nullptr) {
        OCS_LOG_ERROR(std::string("Could not create debug telemetry window: ") + SDL_GetError());
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, SDL_SOFTWARE_RENDERER);
    if (renderer_ == nullptr) {
        OCS_LOG_ERROR(std::string("Could not create debug telemetry renderer: ") + SDL_GetError());
        destroy();
        return false;
    }


    OCS_LOG_INFO("Debug telemetry window initialized");
    return true;
}

void DebugTextWindow::destroy() noexcept {
    if (renderer_ != nullptr) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}

bool DebugTextWindow::render_lines(const std::span<const std::string> lines) noexcept {
    if (!valid()) {
        return false;
    }

    if (!SDL_SetRenderDrawColor(renderer_, 10, 13, 18, 255) || !SDL_RenderClear(renderer_)) {
        OCS_LOG_WARN(std::string("Debug telemetry clear failed: ") + SDL_GetError());
        return false;
    }

    if (!SDL_SetRenderDrawColor(renderer_, 225, 232, 240, 255)) {
        return false;
    }

    constexpr float kX = 12.0F;
    constexpr float kY = 10.0F;
    constexpr float kLineHeight = 11.0F;
    constexpr std::size_t kMaximumLines = 78;

    const std::size_t count = std::min(lines.size(), kMaximumLines);
    for (std::size_t index = 0; index < count; ++index) {
        const float y = kY + static_cast<float>(index) * kLineHeight;
        if (!SDL_RenderDebugText(renderer_, kX, y, lines[index].c_str())) {
            OCS_LOG_WARN(std::string("Debug telemetry text failed: ") + SDL_GetError());
            return false;
        }
    }

    if (lines.size() > kMaximumLines) {
        const float y = kY + static_cast<float>(kMaximumLines - 1U) * kLineHeight;
        (void)SDL_RenderDebugText(renderer_, kX, y, "... telemetry truncated ...");
    }

    if (!SDL_RenderPresent(renderer_)) {
        OCS_LOG_WARN(std::string("Debug telemetry present failed: ") + SDL_GetError());
        return false;
    }
    return true;
}

std::uint32_t DebugTextWindow::id() const noexcept {
    return window_ != nullptr ? SDL_GetWindowID(window_) : 0U;
}

} // namespace ocs::platform
