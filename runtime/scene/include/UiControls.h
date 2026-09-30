#ifndef RADION_UI_CONTROLS_H
#define RADION_UI_CONTROLS_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "Math.h"


#include <string>
#include <vector>

namespace Radion
{

// Logical size UI anchors resolve against, set by the host each frame (UiSystem::setScreenSize()). No layout or hit-testing while invalid.
struct UiViewport
{
    f32 width = 1280.0f;
    f32 height = 720.0f;
    bool valid = false;
};

class UiControl;

// One instance per process, like ScreenDraw.
class UiSystem
{
public:
    static UiSystem& getSingleton();

    void setScreenSize(f32 width, f32 height);
    const UiViewport& viewport() const;

    // Called once per frame by Scene::update(), before components draw, so each control renders this layout. Input goes to the topmost interactive control under the cursor.
    void refresh();

    // Unset falls back to flat color rects.
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
    // True while refresh() walks mControls: unregisterControl() tombstones instead of resizing; compactControls() sweeps afterwards.
    bool mRefreshing = false;
};

inline UiSystem& UiSystems()
{
    return UiSystem::getSingleton();
}

// Carries no state; only gives scenes and tooling something to point at.
class UiCanvas final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::UiCanvas;

private:
    friend class GameObject;

    UiCanvas();
};

// Never attached directly; only a subclass with its own ComponentType can be addComponent<>()ed.
class UiControl : public Component
{
public:
    const Math::vec4& anchors() const;
    const Math::vec4& offsets() const;
    void setAnchors(const Math::vec4& value);
    void setOffsets(const Math::vec4& value);
    // Anchors reset to 0 (offset-driven); offsets become the exact pixel rect.
    void setRect(f32 x, f32 y, f32 width, f32 height);
    const FloatRect& rect() const;
    bool hovered() const;
    bool pressed() const;
    bool clicked() const;
    bool interactive() const;
    void setInteractive(bool value);
    // Higher draws later (on top); on a tie, decides which overlapping interactive control gets input.
    s32 layer() const;
    void setLayer(s32 value);

protected:
    UiControl(ComponentType type, bool interactive);

    void onAwake() override;
    void onDestroy() override;
    void onUpdate(f32 deltaTime) override final;

    // Subclasses draw here, not in onUpdate(): the base onUpdate() owns the once-per-control UiSystem::refresh() call.
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

    Math::vec4 mAnchors{0.0f, 0.0f, 0.0f, 0.0f};
    Math::vec4 mOffsets{0.0f, 0.0f, 120.0f, 32.0f};
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
    // One-shot: true once after a completed click, then false until the next.
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
