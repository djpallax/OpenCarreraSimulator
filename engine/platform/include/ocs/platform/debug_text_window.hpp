#pragma once

#include <cstdint>
#include <span>
#include <string>

struct SDL_Renderer;
struct SDL_Window;

namespace ocs::platform {

class DebugTextWindow {
public:
    DebugTextWindow() = default;
    ~DebugTextWindow();

    DebugTextWindow(const DebugTextWindow&) = delete;
    DebugTextWindow& operator=(const DebugTextWindow&) = delete;
    DebugTextWindow(DebugTextWindow&&) = delete;
    DebugTextWindow& operator=(DebugTextWindow&&) = delete;

    bool create(const std::string& title, int width = 980, int height = 920);
    void destroy() noexcept;
    bool render_lines(std::span<const std::string> lines) noexcept;

    [[nodiscard]] bool valid() const noexcept { return window_ != nullptr && renderer_ != nullptr; }
    [[nodiscard]] std::uint32_t id() const noexcept;

private:
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
};

} // namespace ocs::platform
