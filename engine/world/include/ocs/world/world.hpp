#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "ocs/core/slot_map.hpp"
#include "ocs/render/handles.hpp"
#include "ocs/render/render_scene.hpp"
#include "ocs/world/transform.hpp"

namespace ocs::world {

struct ObjectHandle {
    static constexpr std::uint32_t invalid_index = std::numeric_limits<std::uint32_t>::max();

    std::uint32_t index = invalid_index;
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const noexcept { return index != invalid_index; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return valid(); }
    auto operator<=>(const ObjectHandle&) const = default;
};

struct WorldObject {
    std::string name{};
    render::ModelHandle model{};
    Transform transform{};
    bool visible = true;
    bool movable = false;
};

class World {
public:
    [[nodiscard]] ObjectHandle spawn(std::string name,
                                     render::ModelHandle model,
                                     const Transform& transform = {},
                                     bool movable = false);
    bool destroy(ObjectHandle handle) noexcept;

    [[nodiscard]] WorldObject* get(ObjectHandle handle) noexcept;
    [[nodiscard]] const WorldObject* get(ObjectHandle handle) const noexcept;

    [[nodiscard]] render::RenderScene extract_render_scene(
        const WorldPosition& camera_origin) const;

    [[nodiscard]] std::size_t object_count() const noexcept { return objects_.size(); }

    template <typename Function>
    void for_each_object(Function&& function) {
        objects_.for_each(std::forward<Function>(function));
    }

    template <typename Function>
    void for_each_object(Function&& function) const {
        objects_.for_each(std::forward<Function>(function));
    }

private:
    core::SlotMap<ObjectHandle, WorldObject> objects_{};
};

} // namespace ocs::world
