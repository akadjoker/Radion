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

// Short-lived corner notifications for work the user started and would
// otherwise only learn about from the console: a file written, a bake
// finished, an import that fell back to something.
//
// Not a replacement for the log. Everything shown here is logged too - a
// toast is gone in seconds and cannot be scrolled back to, so it never
// carries the only copy of anything.
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
