#ifndef RADION_UI_CONTROLS_H
#define RADION_UI_CONTROLS_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "Math.h"

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace Radion
{

// Logical size UI anchors resolve against. runtime/scene has no reachable
// Window instance (Engine owns it as a plain member, not a singleton), so
// the host sets this once per frame instead - see UiSystem::setScreenSize().
// No layout or hit-testing runs while invalid, same as a control whose
// anchor base was never set would just sit at its constructed zero rect.
struct UiViewport
{
    f32 width = 1280.0f;
    f32 height = 720.0f;
    bool valid = false;
};

class UiControl;

// Registry and per-frame layout/hit-test pass for every UiControl that
// exists, plus the optional atlas texture widgets draw regions from. One
// instance for the whole process, like ScreenDraw - Scene cannot own this
// bookkeeping itself without a case per ComponentType::UiXxx in its own
// registration switch, which is outside this port's file list.
class UiSystem
{
public:
    static UiSystem& getSingleton();

    void setScreenSize(f32 width, f32 height);
    const UiViewport& viewport() const;

    // Re-derives every live control's rect and hover/press/click state from
    // this instant's input, then routes input to the single topmost
    // interactive control under the cursor. Scene::update() calls this once
    // per frame, ahead of the component update that draws the controls, so
    // each one renders the layout this just produced. Calling it per control
    // instead would cost a full pass over every control for each control
    // there is.
    void refresh();

    // Unset (the default) makes every widget fall back to a flat color
    // rect instead of sampling a region out of this texture.
    void setThemeTexture(TextureHandle texture);
    void clearThemeTexture();
    bool hasThemeTexture() const;
    TextureHandle themeTexture() const;

private:
    friend class UiControl;

    UiSystem();

    void registerControl(UiControl* control);
    void unregisterControl(UiControl* control);
    s32 nextOrder();
    void compactControls();

    UiViewport mViewport;
    TextureHandle mThemeTexture;
    std::vector<UiControl*> mControls;
    s32 mNextOrder = 0;
    // True while refresh() is walking mControls - a control destroyed from
    // inside onUiInput() (a button's own click handler removing itself, its
    // panel, or another control) must not resize the vector refresh() is
    // currently indexing into, so unregisterControl() tombstones instead
    // while this is set and compactControls() sweeps the tombstones once
    // refresh() itself is done.
    bool mRefreshing = false;
};

inline UiSystem& UiSystems()
{
    return UiSystem::getSingleton();
}

// A GameObject marking the root of a UI hierarchy. Carries no state of its
// own - anchors resolve against the screen once no ancestor carries a
// UiControl, so nothing below reads UiCanvas at all. Its only purpose is to
// give scenes and tooling something to point at.
class UiCanvas final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::UiCanvas;

private:
    friend class GameObject;

    UiCanvas();
};

// Anchor+offset rectangle, hover/press/click state and the theme-aware draw
// helpers shared by every concrete widget below. Never attached directly -
// only a subclass with its own ComponentType has a Type to addComponent<>()
// with.
class UiControl : public Component
{
public:
    const glm::vec4& anchors() const;
    const glm::vec4& offsets() const;
    void setAnchors(const glm::vec4& value);
    void setOffsets(const glm::vec4& value);
    // Anchors are reset to 0 (fully offset-driven) and offsets become the
    // exact pixel rect (x, y, x + width, y + height).
    void setRect(f32 x, f32 y, f32 width, f32 height);
    const FloatRect& rect() const;
    bool hovered() const;
    bool pressed() const;
    bool clicked() const;
    bool interactive() const;
    void setInteractive(bool value);
    // Draw order (higher draws later, i.e. on top) and, on a tie, which of
    // two overlapping interactive controls the cursor's input actually
    // reaches - the same role owner()->zIndex() plays in the reference,
    // expressed through the field Radion's GameObject actually has none of
    // and ScreenDraw already needs one of anyway.
    s32 layer() const;
    void setLayer(s32 value);

protected:
    UiControl(ComponentType type, bool interactive);

    void onAwake() override;
    void onDestroy() override;
    void onUpdate(f32 deltaTime) override final;

    // Subclasses draw here instead of overriding onUpdate() directly - the
    // base's onUpdate() owns the once-per-active-control UiSystem::refresh()
    // call every subclass instance would otherwise have to repeat.
    virtual void onUiRender();
    virtual void onUiInput(bool down, bool pressed, bool released);

    void drawSolidRect(f32 x, f32 y, f32 width, f32 height, Color color) const;
    void drawThemeRect(f32 x, f32 y, f32 width, f32 height, f32 srcX, f32 srcY, f32 srcWidth,
                       f32 srcHeight, Color color) const;
    void drawText(f32 x, f32 y, f32 size, const std::string& text, Color color) const;

private:
    friend class UiSystem;

    void updateLayout();
    bool contains(f32 x, f32 y) const;
    void resetInput();
    void handleInput(f32 x, f32 y, bool down, bool pressed, bool released);

    glm::vec4 mAnchors{0.0f, 0.0f, 0.0f, 0.0f};
    glm::vec4 mOffsets{0.0f, 0.0f, 120.0f, 32.0f};
    FloatRect mRect;
    s32 mLayer = 0;
    s32 mOrder = 0;
    bool mInteractive;
    bool mHovered = false;
    bool mPressed = false;
    bool mClicked = false;
};

class UiPanel final : public UiControl
{
public:
    static constexpr ComponentType Type = ComponentType::UiPanel;

    Color color() const;
    void setColor(Color value);

private:
    friend class GameObject;

    UiPanel();
    void onUiRender() override;

    Color mColor;
};

class UiLabel final : public UiControl
{
public:
    static constexpr ComponentType Type = ComponentType::UiLabel;

    const std::string& text() const;
    void setText(const std::string& value);
    f32 fontSize() const;
    void setFontSize(f32 value);
    Color color() const;
    void setColor(Color value);

private:
    friend class GameObject;

    UiLabel();
    void onUiRender() override;

    std::string mText;
    f32 mFontSize = 16.0f;
    Color mColor;
};

class UiButton final : public UiControl
{
public:
    static constexpr ComponentType Type = ComponentType::UiButton;

    const std::string& text() const;
    void setText(const std::string& value);
    // One-shot: true the first time it is called after a completed click,
    // false every time after until the next one.
    bool consumeClick();

private:
    friend class GameObject;

    UiButton();
    void onUiRender() override;
    void onUiInput(bool down, bool pressed, bool released) override;

    std::string mText;
    bool mActivated = false;
};

class UiCheckBox final : public UiControl
{
public:
    static constexpr ComponentType Type = ComponentType::UiCheckBox;

    const std::string& text() const;
    void setText(const std::string& value);
    bool checked() const;
    void setChecked(bool value);
    bool consumeChanged();

private:
    friend class GameObject;

    UiCheckBox();
    void onUiRender() override;
    void onUiInput(bool down, bool pressed, bool released) override;

    std::string mText;
    bool mChecked = false;
    bool mChanged = false;
};

class UiSlider final : public UiControl
{
public:
    static constexpr ComponentType Type = ComponentType::UiSlider;

    f32 value() const;
    void setValue(f32 value);
    f32 minimum() const;
    f32 maximum() const;
    void setRange(f32 minimum, f32 maximum);
    bool consumeChanged();

private:
    friend class GameObject;

    UiSlider();
    void onUiRender() override;
    void onUiInput(bool down, bool pressed, bool released) override;

    f32 mMinimum = 0.0f;
    f32 mMaximum = 1.0f;
    f32 mValue = 0.5f;
    bool mChanged = false;
};

} // namespace Radion

#endif // RADION_UI_CONTROLS_H
