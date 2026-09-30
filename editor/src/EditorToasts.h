#ifndef RADION_EDITOR_TOASTS_H
#define RADION_EDITOR_TOASTS_H

#include "Types.h"

#include <string>
#include <vector>

namespace Radion
{

enum class ToastKind : u8
{
    Info,
    Success,
    Warning,
    Error
};

// Corner notifications; everything shown is also logged, since a toast is gone in seconds.
class EditorToasts
{
public:
    void push(ToastKind kind, std::string message);
    void info(std::string message);
    void success(std::string message);
    void warning(std::string message);
    void error(std::string message);

    void update(f32 deltaTime);
    void draw();
    void clear();

private:
    struct Toast
    {
        ToastKind kind = ToastKind::Info;
        std::string message;
        f32 remaining = 0.0f;
        f32 total = 0.0f;
    };

    std::vector<Toast> mToasts;
};

} // namespace Radion

#endif // RADION_EDITOR_TOASTS_H
